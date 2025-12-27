#!/usr/bin/env bun
/**
 * onshape2urdf.ts — minimal Onshape Assembly → URDF exporter (single file)
 *
 * Runs on Bun (recommended) or Node 18+ (needs TS runner).
 *
 * Auth:
 *   export ONSHAPE_ACCESS_KEY="..."
 *   export ONSHAPE_SECRET_KEY="..."
 *
 * Example:
 *   bun run onshape2urdf.ts --url "https://cad.onshape.com/documents/<did>/w/<wid>/e/<eid>" --out robot.urdf --download-meshes --mesh-dir meshes
 *
 * Notes / limitations:
 * - URDF cannot represent closed loops → we build a spanning tree and ignore extra mates.
 * - Multi-DOF mates (PLANAR, BALL, CYLINDRICAL, PIN_SLOT) are approximated with joint chains.
 * - Subassemblies may appear as mate endpoints; we export kinematics, but only Part instances get meshes.
 */

import { writeFileSync, mkdirSync, existsSync } from "fs";
import { join as pathJoin } from "path";

type Vec3 = [number, number, number];
type Mat4 = number[]; // row-major length 16

type CS = {
	origin: number[];
	xAxis: number[];
	yAxis: number[];
	zAxis: number[];
};

type Instance = {
	id: string;
	name?: string;
	type?: string; // "Part" | "Assembly" | ...
	suppressed?: boolean;

	// Part refs (present when type==="Part")
	documentId?: string;
	documentMicroversion?: string;
	elementId?: string;
	partId?: string;
};

type MateFeature = {
	featureType: string; // "mate"
	id?: string;
	suppressed?: boolean;
	featureData?: {
		name?: string;
		mateType?: string; // FASTENED, REVOLUTE, SLIDER, ...
		matedEntities?: Array<{
			matedOccurrence?: string[]; // [] means "root"/world-ish
			matedCS?: CS;
		}>;
	};
};

type AssemblyResponse = {
	rootAssembly: {
		instances: Instance[];
		occurrences?: any[];
		features?: MateFeature[];
	};
	subAssemblies?: Array<{
		instances: Instance[];
	}>;
};

type Edge = {
	name: string;
	mateType: string;
	aKey: string;
	bKey: string;
	aCS: CS;
	bCS: CS;
};

type NodeInfo = {
	key: string;
	leafId?: string;
	instance?: Instance; // leaf instance (best-effort)
	linkName: string;
	partFromLink: Mat4; // T_part_from_link (link -> part/origin/mesh frame)
	meshFile?: string; // for Part instances when downloaded
};

function die(msg: string): never {
	console.error(`\nERROR: ${msg}\n`);
	process.exit(1);
}

function parseArgs(argv: string[]) {
	const out: Record<string, any> = {};
	for (let i = 0; i < argv.length; i++) {
		const a = argv[i];
		if (!a.startsWith("--")) continue;
		const key = a.slice(2);
		const next = argv[i + 1];
		if (next && !next.startsWith("--")) {
			out[key] = next;
			i++;
		} else {
			out[key] = true;
		}
	}
	return out;
}

function sanitizeName(s: string): string {
	const cleaned = s
		.trim()
		.replace(/\s+/g, "_")
		.replace(/[^a-zA-Z0-9_]/g, "_")
		.replace(/^_+/, "")
		.replace(/_+$/, "");
	return cleaned.length ? cleaned : "item";
}

function uniqueName(base: string, used: Map<string, number>): string {
	const b = sanitizeName(base);
	const n = used.get(b) ?? 0;
	used.set(b, n + 1);
	return n === 0 ? b : `${b}_${n + 1}`;
}

function ident4(): Mat4 {
	return [1, 0, 0, 0,
		0, 1, 0, 0,
		0, 0, 1, 0,
		0, 0, 0, 1];
}

function matMul(a: Mat4, b: Mat4): Mat4 {
	const o = new Array<number>(16).fill(0);
	for (let r = 0; r < 4; r++) {
		for (let c = 0; c < 4; c++) {
			o[r * 4 + c] =
				a[r * 4 + 0] * b[0 * 4 + c] +
				a[r * 4 + 1] * b[1 * 4 + c] +
				a[r * 4 + 2] * b[2 * 4 + c] +
				a[r * 4 + 3] * b[3 * 4 + c];
		}
	}
	return o;
}

// Inverse of rigid transform (rotation orthonormal)
function invRigid(m: Mat4): Mat4 {
	// R in row-major:
	const r00 = m[0], r01 = m[1], r02 = m[2];
	const r10 = m[4], r11 = m[5], r12 = m[6];
	const r20 = m[8], r21 = m[9], r22 = m[10];
	const tx = m[3], ty = m[7], tz = m[11];

	// R^T
	const t00 = r00, t01 = r10, t02 = r20;
	const t10 = r01, t11 = r11, t12 = r21;
	const t20 = r02, t21 = r12, t22 = r22;

	// -R^T * t
	const itx = -(t00 * tx + t01 * ty + t02 * tz);
	const ity = -(t10 * tx + t11 * ty + t12 * tz);
	const itz = -(t20 * tx + t21 * ty + t22 * tz);

	return [t00, t01, t02, itx,
		t10, t11, t12, ity,
		t20, t21, t22, itz,
		0, 0, 0, 1];
}

// Onshape gives CS axes as vectors in the part/occurrence frame.
// Build T_part_from_connector (link frame = connector; mesh frame = part/origin).
function csToPartFromConnector(cs: CS): Mat4 {
	const ox = cs.origin?.[0] ?? 0;
	const oy = cs.origin?.[1] ?? 0;
	const oz = cs.origin?.[2] ?? 0;

	// Columns are xAxis, yAxis, zAxis expressed in PART frame
	const x = cs.xAxis ?? [1, 0, 0];
	const y = cs.yAxis ?? [0, 1, 0];
	const z = cs.zAxis ?? [0, 0, 1];

	return [
		x[0], y[0], z[0], ox,
		x[1], y[1], z[1], oy,
		x[2], y[2], z[2], oz,
		0, 0, 0, 1
	];
}

// URDF uses rpy with R = Rz(yaw) * Ry(pitch) * Rx(roll)
function rotToRPY(m: Mat4): Vec3 {
	const r00 = m[0], r01 = m[1], r02 = m[2];
	const r10 = m[4], r11 = m[5], r12 = m[6];
	const r20 = m[8], r21 = m[9], r22 = m[10];

	const roll = Math.atan2(r21, r22);
	const pitch = Math.atan2(-r20, Math.sqrt(r21 * r21 + r22 * r22));
	const yaw = Math.atan2(r10, r00);
	return [roll, pitch, yaw];
}

function matToXyzRpy(childFromParent: Mat4): { xyz: Vec3; rpy: Vec3 } {
	const xyz: Vec3 = [childFromParent[3], childFromParent[7], childFromParent[11]];
	const rpy = rotToRPY(childFromParent);
	return { xyz, rpy };
}

function fmt(n: number): string {
	// stable-ish printing (avoid scientific noise)
	const s = n.toFixed(9);
	return s.replace(/\.?0+$/, "");
}

function pathKey(path: string[] | undefined): string {
	if (!path || path.length === 0) return "world";
	return path.join("/");
}

function leafIdFromPath(path: string[] | undefined): string | undefined {
	if (!path || path.length === 0) return undefined;
	return path[path.length - 1];
}

function parseOnshapeUrl(u: string): { stack: string; did: string; wvm: string; wvmid: string; eid: string } {
	let url: URL;
	try { url = new URL(u); } catch { die(`Bad --url: ${u}`); }

	const stack = `${url.protocol}//${url.host}`;

	// Typical: /documents/<did>/<wvm>/<wvmid>/e/<eid>
	const m = url.pathname.match(/\/documents\/([^/]+)\/(w|v|m)\/([^/]+)\/e\/([^/]+)/);
	if (!m) die(`Could not parse Onshape URL path: ${url.pathname}`);
	const [, did, wvm, wvmid, eid] = m;
	return { stack, did, wvm, wvmid, eid };
}

function basicAuthHeader(accessKey: string, secretKey: string): string {
	const b64 = Buffer.from(`${accessKey}:${secretKey}`, "utf8").toString("base64");
	return `Basic ${b64}`;
}

async function fetchJson(url: string, headers: Record<string, string>): Promise<any> {
	const res = await fetch(url, { headers, redirect: "follow" as any });
	if (!res.ok) {
		const txt = await res.text().catch(() => "");
		die(`HTTP ${res.status} ${res.statusText}\n${txt}`);
	}
	return await res.json();
}

async function fetchBytesFollowRedirect(url: string, headers: Record<string, string>): Promise<Uint8Array> {
	// first try manual redirects (Onshape often redirects downloads to signed URLs)
	const res1 = await fetch(url, { headers, redirect: "manual" as any });
	if (res1.status >= 300 && res1.status < 400) {
		const loc = res1.headers.get("location");
		if (!loc) die(`Redirect without Location header for: ${url}`);
		const res2 = await fetch(loc, { headers, redirect: "follow" as any });
		if (!res2.ok) die(`Mesh download failed: HTTP ${res2.status} ${res2.statusText}`);
		return new Uint8Array(await res2.arrayBuffer());
	}
	if (!res1.ok) die(`Mesh download failed: HTTP ${res1.status} ${res1.statusText}`);
	return new Uint8Array(await res1.arrayBuffer());
}

function mateToJointChain(mateType: string): Array<{ type: string; axis?: Vec3 }> {
	const t = (mateType || "FASTENED").toUpperCase();

	// Onshape mate axes are defined by the mate connector CS:
	// revolute/slider/cylindrical mainly along Z; pin-slot along X for translation. 
	switch (t) {
		case "FASTENED":
			return [{ type: "fixed" }];
		case "REVOLUTE":
			// Could be limited; we default to continuous to not lie about limits
			return [{ type: "continuous", axis: [0, 0, 1] }];
		case "SLIDER":
			return [{ type: "prismatic", axis: [0, 0, 1] }];
		case "CYLINDRICAL":
			// URDF can't do rotation+translation in one joint → chain
			return [
				{ type: "prismatic", axis: [0, 0, 1] },
				{ type: "continuous", axis: [0, 0, 1] },
			];
		case "PIN_SLOT":
			return [
				{ type: "prismatic", axis: [1, 0, 0] },
				{ type: "continuous", axis: [0, 0, 1] },
			];
		case "PLANAR":
			return [
				{ type: "prismatic", axis: [1, 0, 0] },
				{ type: "prismatic", axis: [0, 1, 0] },
				{ type: "continuous", axis: [0, 0, 1] },
			];
		case "BALL":
			return [
				{ type: "continuous", axis: [1, 0, 0] },
				{ type: "continuous", axis: [0, 1, 0] },
				{ type: "continuous", axis: [0, 0, 1] },
			];
		default:
			// PARALLEL, TANGENT, etc. — best-effort: fixed
			return [{ type: "fixed" }];
	}
}

function xmlEscape(s: string): string {
	return s.replace(/&/g, "&amp;").replace(/</g, "&lt;").replace(/"/g, "&quot;");
}

async function main() {
	const args = parseArgs(process.argv.slice(2));

	const url = args.url as string | undefined;
	const did = args.did as string | undefined;
	const wvm = args.wvm as string | undefined;
	const wvmid = args.wvmid as string | undefined;
	const eid = args.eid as string | undefined;

	const apiVersion = (args["api-version"] as string | undefined) ?? "v10";
	const outPath = (args.out as string | undefined) ?? "robot.urdf";
	const robotName = (args.name as string | undefined) ?? "onshape_robot";
	const downloadMeshes = !!args["download-meshes"];
	const meshDir = (args["mesh-dir"] as string | undefined) ?? "meshes";
	const meshRefPrefix = (args["mesh-prefix"] as string | undefined) ?? ""; // e.g. "package://my_pkg/"

	const accessKey = process.env.ONSHAPE_ACCESS_KEY;
	const secretKey = process.env.ONSHAPE_SECRET_KEY;
	if (!accessKey || !secretKey) {
		die(`Set ONSHAPE_ACCESS_KEY and ONSHAPE_SECRET_KEY in env.`);
	}

	let stack = (args.stack as string | undefined) ?? "https://cad.onshape.com";
	let ids: { did: string; wvm: string; wvmid: string; eid: string } | undefined;

	if (url) {
		const parsed = parseOnshapeUrl(url);
		ids = { did: parsed.did, wvm: parsed.wvm, wvmid: parsed.wvmid, eid: parsed.eid };
		// default stack from URL unless user overrides
		if (!args.stack) stack = parsed.stack;
	} else {
		if (!did || !wvm || !wvmid || !eid) {
			die(`Provide --url or (--did --wvm --wvmid --eid).`);
		}
		ids = { did, wvm, wvmid, eid };
	}

	const apiBase = `${stack.replace(/\/+$/, "")}/api/${apiVersion}`;
	const headers: Record<string, string> = {
		Authorization: basicAuthHeader(accessKey, secretKey),
	};

	const assemblyUrl = new URL(`${apiBase}/assemblies/d/${ids.did}/${ids.wvm}/${ids.wvmid}/e/${ids.eid}`);
	assemblyUrl.searchParams.set("includeMateFeatures", "true");
	assemblyUrl.searchParams.set("includeMateConnectors", "true");
	assemblyUrl.searchParams.set("includeNonSolids", "true");

	console.log(`Fetching assembly: ${assemblyUrl.toString()}`);
	const asm = (await fetchJson(assemblyUrl.toString(), headers)) as AssemblyResponse;

	// --- NEW: capture all occurrence paths + transforms so we can include "unmated" parts
	const occTfByKey = new Map<string, Mat4>();
	const occKeys = new Set<string>();
	const occs = asm.rootAssembly?.occurrences ?? [];
	for (const o of occs) {
		const p = (o.path ?? o.occurrencePath ?? o.occurrence) as string[] | undefined;
		const key = pathKey(p);
		const m =
			(o.transform?.matrix ?? o.transform?.mat4 ?? o.transform) as number[] | undefined;

		if (key !== "world") {
			occKeys.add(key);
		}
		if (Array.isArray(m) && m.length === 16) {
			occTfByKey.set(key, m);
		}
	}

	const allInstances: Instance[] = [
		...(asm.rootAssembly?.instances ?? []),
		...((asm.subAssemblies ?? []).flatMap((s) => s.instances ?? [])),
	];

	const instanceById = new Map<string, Instance>();
	for (const inst of allInstances) {
		if (inst?.id) instanceById.set(inst.id, inst);
	}

	// Extract mate edges from rootAssembly.features
	const features = asm.rootAssembly?.features ?? [];
	const edges: Edge[] = [];
	for (const f of features) {
		if (f?.featureType !== "mate") continue;
		if (f.suppressed) continue;
		const fd = f.featureData;
		if (!fd?.matedEntities || fd.matedEntities.length < 2) continue;
		const mateType = (fd.mateType ?? "FASTENED").toUpperCase();
		const name = fd.name ?? f.id ?? `mate_${edges.length + 1}`;

		const ea = fd.matedEntities[0];
		const eb = fd.matedEntities[1];
		if (!ea?.matedCS || !eb?.matedCS) continue;

		const aKey = pathKey(ea.matedOccurrence);
		const bKey = pathKey(eb.matedOccurrence);

		edges.push({
			name,
			mateType,
			aKey,
			bKey,
			aCS: ea.matedCS,
			bCS: eb.matedCS,
		});
	}

	if (edges.length === 0) {
		console.warn(
			"No mate features found. Exporting all occurrences as fixed to world."
		);
	}

	// Build adjacency
	const adj = new Map<string, Edge[]>();
	function addAdj(k: string, e: Edge) {
		const arr = adj.get(k) ?? [];
		arr.push(e);
		adj.set(k, arr);
	}
	for (const e of edges) {
		addAdj(e.aKey, e);
		addAdj(e.bKey, e);
	}

	// Pick a root: prefer something mated to world
	let rootKey: string | undefined;
	for (const e of edges) {
		if (e.aKey === "world" && e.bKey !== "world") { rootKey = e.bKey; break; }
		if (e.bKey === "world" && e.aKey !== "world") { rootKey = e.aKey; break; }
	}
	if (!rootKey) {
		// fall back to first non-world node
		rootKey = edges.find((e) => e.aKey !== "world")?.aKey ?? Array.from(occKeys)[0];
	}
	if (!rootKey) {
		rootKey = "world";
	}

	// Create node infos (names first)
	const usedNames = new Map<string, number>();
	const nodes = new Map<string, NodeInfo>();

	function ensureNode(key: string) {
		if (nodes.has(key)) return;
		const leafId = key === "world" ? undefined : key.split("/").slice(-1)[0];
		const inst = leafId ? instanceById.get(leafId) : undefined;
		const baseName = key === "world" ? "world" : (inst?.name ?? leafId ?? key);
		const linkName = uniqueName(baseName, usedNames);

		nodes.set(key, {
			key,
			leafId,
			instance: inst,
			linkName,
			partFromLink: ident4(), // will be set during spanning-tree build for non-root nodes
		});
	}

	ensureNode("world");
	for (const key of occKeys) { ensureNode(key); }
	for (const e of edges) { ensureNode(e.aKey); ensureNode(e.bKey); }

	// Spanning tree BFS from "world" if connected, else from rootKey and then attach to world
	const visited = new Set<string>();
	const parent = new Map<string, string>(); // childKey -> parentKey
	const parentEdge = new Map<string, Edge>(); // childKey -> edge used
	const childSideCS = new Map<string, CS>(); // childKey -> CS used as child's incoming link frame
	const loopNotes: string[] = [];

	// Start from world if world has edges, otherwise from rootKey
	const start = (adj.get("world")?.length ?? 0) > 0 ? "world" : rootKey;

	const q: string[] = [];
	visited.add(start);
	q.push(start);

	while (q.length) {
		const cur = q.shift()!;
		const es = adj.get(cur) ?? [];
		for (const e of es) {
			const nxt = e.aKey === cur ? e.bKey : e.aKey;
			if (!nodes.has(nxt)) continue;
			if (visited.has(nxt)) {
				// loop edge (ignored in spanning tree)
				if (cur !== parent.get(nxt) && nxt !== parent.get(cur)) {
					loopNotes.push(`Ignored extra mate (loop): ${e.name} (${e.mateType}) between ${cur} and ${nxt}`);
				}
				continue;
			}
			visited.add(nxt);
			parent.set(nxt, cur);
			parentEdge.set(nxt, e);

			// record which CS belongs to the CHILD side (nxt)
			const nxtIsA = e.aKey === nxt;
			const csForNxt = nxtIsA ? e.aCS : e.bCS;
			childSideCS.set(nxt, csForNxt);

			q.push(nxt);
		}
	}

	// If we didn't start from world, ensure root is attached to world with a fixed joint (identity)
	if (start !== "world") {
		if (!visited.has("world")) visited.add("world");
		parent.set(start, "world");
		// synthetic edge
		parentEdge.set(start, {
			name: "fixed_to_world",
			mateType: "FASTENED",
			aKey: "world",
			bKey: start,
			aCS: { origin: [0, 0, 0], xAxis: [1, 0, 0], yAxis: [0, 1, 0], zAxis: [0, 0, 1] },
			bCS: { origin: [0, 0, 0], xAxis: [1, 0, 0], yAxis: [0, 1, 0], zAxis: [0, 0, 1] },
		});
		childSideCS.set(start, { origin: [0, 0, 0], xAxis: [1, 0, 0], yAxis: [0, 1, 0], zAxis: [0, 0, 1] });
	}

	// --- NEW: attach anything not reached by mates directly to world as fixed
	const originOverrideByChildKey = new Map<string, Mat4>();

	const identityCS: CS = {
		origin: [0, 0, 0],
		xAxis: [1, 0, 0],
		yAxis: [0, 1, 0],
		zAxis: [0, 0, 1],
	};

	for (const [k, n] of nodes) {
		if (k === "world") continue;
		if (visited.has(k)) continue;

		visited.add(k);
		parent.set(k, "world");
		childSideCS.set(k, identityCS);
		originOverrideByChildKey.set(k, occTfByKey.get(k) ?? ident4());

		parentEdge.set(k, {
			name: `unmated_${n.linkName}`,
			mateType: "FASTENED",
			aKey: "world",
			bKey: k,
			aCS: identityCS,
			bCS: identityCS,
		});
	}

	// Assign partFromLink for each visited node (except world): link frame = connector used to connect to its parent
	// So partFromLink = T_part_from_connector(childSideCS)
	for (const key of nodes.keys()) {
		if (!visited.has(key)) continue;
		if (key === "world") continue;
		const cs = childSideCS.get(key);
		if (!cs) continue;
		nodes.get(key)!.partFromLink = csToPartFromConnector(cs);
	}

	// Optional: download meshes for Part instances
	if (downloadMeshes) {
		if (!existsSync(meshDir)) mkdirSync(meshDir, { recursive: true });

		const meshByPartKey = new Map<string, string>();
		for (const n of nodes.values()) {
			if (!visited.has(n.key)) continue;
			if (n.key === "world") continue;
			const inst = n.instance;
			if (!inst || inst.type !== "Part") continue;
			if (!inst.documentId || !inst.documentMicroversion || !inst.elementId || !inst.partId) continue;

			const partKey = `${inst.documentId}:${inst.documentMicroversion}:${inst.elementId}:${inst.partId}`;
			let file = meshByPartKey.get(partKey);
			if (!file) {
				const base = sanitizeName(inst.name ?? `part_${inst.partId}`);
				file = `${base}.stl`;
				// avoid collisions
				let attempt = 1;
				while (existsSync(pathJoin(meshDir, file))) {
					attempt++;
					file = `${base}_${attempt}.stl`;
				}

				const stlUrl = new URL(
					`${apiBase}/parts/d/${inst.documentId}/m/${inst.documentMicroversion}/e/${inst.elementId}/partid/${inst.partId}/stl`
				);
				stlUrl.searchParams.set("mode", "binary");
				stlUrl.searchParams.set("units", "meter");
				stlUrl.searchParams.set("scale", "1");
				stlUrl.searchParams.set("grouping", "true");

				console.log(`Downloading STL: ${inst.name ?? inst.partId} -> ${pathJoin(meshDir, file)}`);
				const bytes = await fetchBytesFollowRedirect(stlUrl.toString(), headers);
				writeFileSync(pathJoin(meshDir, file), bytes);

				meshByPartKey.set(partKey, file);
			}
			n.meshFile = file;
		}
	}

	// Build URDF links & joints
	const urdfLinks: string[] = [];
	const urdfJoints: string[] = [];
	const usedJointNames = new Map<string, number>();
	const usedLinkNames2 = new Set<string>([nodes.get("world")!.linkName]);

	function addLink(linkName: string, visualXml?: string) {
		if (usedLinkNames2.has(linkName)) return;
		usedLinkNames2.add(linkName);
		urdfLinks.push(
			`  <link name="${xmlEscape(linkName)}">` +
			(visualXml ? `\n${visualXml}\n  ` : "") +
			`</link>`
		);
	}

	// Ensure world link
	addLink(nodes.get("world")!.linkName);

	// Add actual links
	for (const n of nodes.values()) {
		if (!visited.has(n.key)) continue;
		if (n.key === "world") continue;

		let visual = "";
		const inst = n.instance;

		if (inst?.type === "Part" && n.meshFile) {
			const { xyz, rpy } = matToXyzRpy(n.partFromLink);
			const meshPath = meshRefPrefix ? `${meshRefPrefix}${meshDir}/${n.meshFile}` : `${meshDir}/${n.meshFile}`;
			visual =
				`    <visual>\n` +
				`      <origin xyz="${fmt(xyz[0])} ${fmt(xyz[1])} ${fmt(xyz[2])}" rpy="${fmt(rpy[0])} ${fmt(rpy[1])} ${fmt(rpy[2])}"/>\n` +
				`      <geometry>\n` +
				`        <mesh filename="${xmlEscape(meshPath)}"/>\n` +
				`      </geometry>\n` +
				`    </visual>\n` +
				`    <collision>\n` +
				`      <origin xyz="${fmt(xyz[0])} ${fmt(xyz[1])} ${fmt(xyz[2])}" rpy="${fmt(rpy[0])} ${fmt(rpy[1])} ${fmt(rpy[2])}"/>\n` +
				`      <geometry>\n` +
				`        <mesh filename="${xmlEscape(meshPath)}"/>\n` +
				`      </geometry>\n` +
				`    </collision>`;
		} else if (inst?.type === "Part" && !downloadMeshes) {
			// still output link, just no geometry
			visual = "";
		}

		addLink(n.linkName, visual || undefined);
	}

	// Now joints from spanning tree
	// We must process in a topological-ish order: easiest is to iterate nodes and use parent map until stable.
	const orderedKeys = Array.from(visited.values()).filter((k) => k !== "world");
	// crude: sort by distance to root by walking parents
	orderedKeys.sort((a, b) => {
		const da = depth(a), db = depth(b);
		return da - db;
	});

	function depth(k: string): number {
		let d = 0;
		let cur = k;
		while (cur !== "world" && parent.has(cur)) {
			d++;
			cur = parent.get(cur)!;
			if (d > 10000) break;
		}
		return d;
	}

	// For intermediate links we create new link names
	function createIntermediateLink(base: string): string {
		let name = sanitizeName(base);
		let i = 1;
		while (usedLinkNames2.has(name)) {
			i++;
			name = `${sanitizeName(base)}_${i}`;
		}
		addLink(name);
		return name;
	}

	for (const childKey of orderedKeys) {
		const pKey = parent.get(childKey);
		if (!pKey) continue;
		const e = parentEdge.get(childKey);
		if (!e) continue;

		const pNode = nodes.get(pKey)!;
		const cNode = nodes.get(childKey)!;

		// Determine which CS is on parent side for this mate
		const parentIsA = e.aKey === pKey;
		const pCS = parentIsA ? e.aCS : e.bCS;

		const chain = mateToJointChain(e.mateType);

		// parentConnectorFromParentLink = inv(T_part_from_connector(pCS)) * T_part_from_link(parent)
		const parentPartFromLink = pKey === "world" ? ident4() : pNode.partFromLink;
		const parentPartFromParentConn = csToPartFromConnector(pCS);
		const parentConnFromParentLink = matMul(invRigid(parentPartFromParentConn), parentPartFromLink);

		// First joint origin attaches at mate connector point
		const firstOrigin = originOverrideByChildKey.get(childKey) ?? parentConnFromParentLink;

		// Create joint(s)
		if (chain.length === 1) {
			const jName = uniqueName(`joint_${e.name}_${e.mateType}`, usedJointNames);
			const { xyz, rpy } = matToXyzRpy(firstOrigin);

			const axisXml = chain[0].axis
				? `\n    <axis xyz="${fmt(chain[0].axis![0])} ${fmt(chain[0].axis![1])} ${fmt(chain[0].axis![2])}"/>`
				: "";

			urdfJoints.push(
				`  <joint name="${xmlEscape(jName)}" type="${xmlEscape(chain[0].type)}">\n` +
				`    <parent link="${xmlEscape(pNode.linkName)}"/>\n` +
				`    <child link="${xmlEscape(cNode.linkName)}"/>\n` +
				`    <origin xyz="${fmt(xyz[0])} ${fmt(xyz[1])} ${fmt(xyz[2])}" rpy="${fmt(rpy[0])} ${fmt(rpy[1])} ${fmt(rpy[2])}"/>` +
				axisXml + `\n` +
				`  </joint>`
			);
		} else {
			// multi-joint chain: parent -> i1 -> i2 -> ... -> child
			let prevLink = pNode.linkName;

			for (let i = 0; i < chain.length; i++) {
				const isLast = i === chain.length - 1;
				const nextLink = isLast
					? cNode.linkName
					: createIntermediateLink(`${cNode.linkName}__${sanitizeName(e.name)}__i${i + 1}`);

				const jName = uniqueName(`joint_${e.name}_${e.mateType}_dof${i + 1}`, usedJointNames);

				const originMat = i === 0 ? firstOrigin : ident4();
				const { xyz, rpy } = matToXyzRpy(originMat);

				const axis = chain[i].axis;
				const axisXml = axis
					? `\n    <axis xyz="${fmt(axis[0])} ${fmt(axis[1])} ${fmt(axis[2])}"/>`
					: "";

				urdfJoints.push(
					`  <joint name="${xmlEscape(jName)}" type="${xmlEscape(chain[i].type)}">\n` +
					`    <parent link="${xmlEscape(prevLink)}"/>\n` +
					`    <child link="${xmlEscape(nextLink)}"/>\n` +
					`    <origin xyz="${fmt(xyz[0])} ${fmt(xyz[1])} ${fmt(xyz[2])}" rpy="${fmt(rpy[0])} ${fmt(rpy[1])} ${fmt(rpy[2])}"/>` +
					axisXml + `\n` +
					`  </joint>`
				);

				prevLink = nextLink;
			}
		}
	}

	// Add loop notes as XML comments
	const comments: string[] = [];
	for (const note of loopNotes.slice(0, 200)) {
		comments.push(`  <!-- ${xmlEscape(note)} -->`);
	}

	const urdf =
		`<?xml version="1.0"?>\n` +
		`<robot name="${xmlEscape(robotName)}">\n` +
		(comments.length ? comments.join("\n") + "\n" : "") +
		`${urdfLinks.join("\n")}\n` +
		`${urdfJoints.join("\n")}\n` +
		`</robot>\n`;

	writeFileSync(outPath, urdf, "utf8");
	console.log(`\nWrote URDF: ${outPath}`);
	if (downloadMeshes) console.log(`Meshes: ${meshDir}/`);
}

main().catch((e) => die(String(e)));
