#!/usr/bin/env bun
/**
 * onshape_fetch_snapshot_recursive.ts
 *
 * Recursively fetch all referenced subassemblies (by microversion) and download all Part STLs.
 *
 * Auth:
 *   export ONSHAPE_ACCESS_KEY="..."
 *   export ONSHAPE_SECRET_KEY="..."
 *
 * Example:
 *   bun run onshape_fetch_snapshot_recursive.ts \
 *     --url "https://cad.onshape.com/documents/<did>/w/<wid>/e/<eid>" \
 *     --outdir dump --download-meshes --mesh-dir meshes --pin-microversion
 */

import { writeFileSync, mkdirSync, existsSync, appendFileSync } from "fs";
import { join as pathJoin } from "path";

type Dict<T = any> = Record<string, T>;

type Instance = {
  id: string;
  type?: string; // "Part" | "Assembly" | ...
  name?: string;
  suppressed?: boolean;

  documentId?: string;
  documentMicroversion?: string; // IMPORTANT for immutable recursion
  elementId?: string;

  // Part-only:
  partId?: string;
};

type AssemblyResponse = {
  rootAssembly?: { instances?: Instance[]; occurrences?: any[]; features?: any[] };
  subAssemblies?: Array<{ instances?: Instance[] }>;
};

type AssemblyRef = { did: string; mv: string; eid: string };

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

function ensureDir(p: string) {
  if (!existsSync(p)) mkdirSync(p, { recursive: true });
}

function sanitizeName(s: string): string {
  const cleaned = (s ?? "")
    .trim()
    .replace(/\s+/g, "_")
    .replace(/[^a-zA-Z0-9_]/g, "_")
    .replace(/^_+/, "")
    .replace(/_+$/, "");
  return cleaned.length ? cleaned : "item";
}

function basicAuthHeader(accessKey: string, secretKey: string): string {
  const b64 = Buffer.from(`${accessKey}:${secretKey}`, "utf8").toString("base64");
  return `Basic ${b64}`;
}

function parseOnshapeUrl(u: string): { stack: string; did: string; wvm: "w" | "v" | "m"; wvmid: string; eid: string } {
  let url: URL;
  try { url = new URL(u); } catch { die(`Bad --url: ${u}`); }

  const stack = `${url.protocol}//${url.host}`;
  const m = url.pathname.match(/\/documents\/([^/]+)\/(w|v|m)\/([^/]+)\/e\/([^/]+)/);
  if (!m) die(`Could not parse Onshape URL path: ${url.pathname}`);
  const [, did, wvm, wvmid, eid] = m;
  return { stack, did, wvm: wvm as any, wvmid, eid };
}

async function fetchJson(url: string, headers: Record<string, string>): Promise<any> {
  const res = await fetch(url, { headers, redirect: "follow" as any });
  if (!res.ok) {
    const txt = await res.text().catch(() => "");
    throw new Error(`HTTP ${res.status} ${res.statusText}\n${txt}`);
  }
  return await res.json();
}

async function fetchBytesFollowRedirect(url: string, headers: Record<string, string>): Promise<Uint8Array> {
  const res1 = await fetch(url, { headers, redirect: "manual" as any });
  if (res1.status >= 300 && res1.status < 400) {
    const loc = res1.headers.get("location");
    if (!loc) throw new Error(`Redirect without Location header for: ${url}`);
    const res2 = await fetch(loc, { headers, redirect: "follow" as any });
    if (!res2.ok) throw new Error(`Download failed: HTTP ${res2.status} ${res2.statusText}`);
    return new Uint8Array(await res2.arrayBuffer());
  }
  if (!res1.ok) throw new Error(`Download failed: HTTP ${res1.status} ${res1.statusText}`);
  return new Uint8Array(await res1.arrayBuffer());
}

function logError(outdir: string, ctx: Dict) {
  appendFileSync(pathJoin(outdir, "errors.jsonl"), JSON.stringify({ t: new Date().toISOString(), ...ctx }) + "\n", "utf8");
}

function allInstancesFromAsm(asm: AssemblyResponse): Instance[] {
  return [
    ...((asm.rootAssembly?.instances ?? []) as Instance[]),
    ...((asm.subAssemblies ?? []).flatMap((s) => (s.instances ?? []) as Instance[])),
  ];
}

function asmRefKey(r: AssemblyRef): string {
  return `${r.did}:${r.mv}:${r.eid}`;
}

function partKey(inst: Instance): string | null {
  if (!inst.documentId || !inst.documentMicroversion || !inst.elementId || !inst.partId) return null;
  return `${inst.documentId}:${inst.documentMicroversion}:${inst.elementId}:${inst.partId}`;
}

async function main() {
  const args = parseArgs(process.argv.slice(2));

  const accessKey = process.env.ONSHAPE_ACCESS_KEY;
  const secretKey = process.env.ONSHAPE_SECRET_KEY;
  if (!accessKey || !secretKey) die(`Set ONSHAPE_ACCESS_KEY and ONSHAPE_SECRET_KEY in env.`);

  const apiVersion = (args["api-version"] as string | undefined) ?? "v10";
  const outdir = (args.outdir as string | undefined) ?? "dump";
  const downloadMeshes = !!args["download-meshes"];
  const meshDirRel = (args["mesh-dir"] as string | undefined) ?? "meshes";
  const pinMicroversion = !!args["pin-microversion"];

  let stack = (args.stack as string | undefined) ?? "https://cad.onshape.com";
  let did = args.did as string | undefined;
  let wvm = args.wvm as "w" | "v" | "m" | undefined;
  let wvmid = args.wvmid as string | undefined;
  let eid = args.eid as string | undefined;

  if (args.url) {
    const parsed = parseOnshapeUrl(String(args.url));
    if (!args.stack) stack = parsed.stack;
    did = parsed.did;
    wvm = parsed.wvm;
    wvmid = parsed.wvmid;
    eid = parsed.eid;
  }
  if (!did || !wvm || !wvmid || !eid) die(`Provide --url or (--did --wvm --wvmid --eid).`);

  ensureDir(outdir);
  ensureDir(pathJoin(outdir, "assemblies"));

  const headers: Record<string, string> = { Authorization: basicAuthHeader(accessKey, secretKey) };
  const apiBase = `${stack.replace(/\/+$/, "")}/api/${apiVersion}`;

  // Optionally pin top workspace to microversion (immutable snapshot)
  let effectiveWvm = wvm;
  let effectiveWvmid = wvmid;
  let pinnedMicroversion: string | null = null;

  if (pinMicroversion && wvm === "w") {
    try {
      const cmvUrl = `${apiBase}/documents/d/${did}/w/${wvmid}/currentmicroversion`;
      console.log(`Pinning microversion via: ${cmvUrl}`);
      const cmv = await fetchJson(cmvUrl, headers);
      const mv = cmv?.microversionId ?? cmv?.id ?? cmv?.microversion ?? null;
      if (!mv) die(`Could not read microversionId from currentmicroversion response`);
      pinnedMicroversion = String(mv);
      effectiveWvm = "m";
      effectiveWvmid = pinnedMicroversion;
      console.log(`Pinned workspace w/${wvmid} → microversion m/${pinnedMicroversion}`);
    } catch (e: any) {
      die(`Failed to pin microversion: ${String(e?.message ?? e)}`);
    }
  }

  const meshDirAbs = pathJoin(outdir, meshDirRel);
  if (downloadMeshes) ensureDir(meshDirAbs);

  // Output index: partKey -> relative path (meshDirRel/file.stl)
  const meshIndex: Record<string, string> = {};
  const downloadedPartKeys = new Set<string>();

  // BFS over assemblies
  const seenAsm = new Set<string>();
  const q: Array<{ kind: "root" | "sub"; ref: any }> = [];

  // Root fetch uses (did, effectiveWvm/effectiveWvmid, eid)
  q.push({ kind: "root", ref: { did, wvm: effectiveWvm, wvmid: effectiveWvmid, eid } });

  let fetchedAssemblies = 0;

  while (q.length) {
    const item = q.shift()!;
    try {
      let asm: AssemblyResponse | null = null;

      if (item.kind === "root") {
        const r = item.ref as { did: string; wvm: "w" | "v" | "m"; wvmid: string; eid: string };
        const url = new URL(`${apiBase}/assemblies/d/${r.did}/${r.wvm}/${r.wvmid}/e/${r.eid}`);
        url.searchParams.set("includeMateFeatures", "true");
        url.searchParams.set("includeMateConnectors", "true");
        url.searchParams.set("includeNonSolids", "true");
        console.log(`Fetch ROOT assembly: ${url.toString()}`);
        asm = (await fetchJson(url.toString(), headers)) as AssemblyResponse;

        // save
        writeFileSync(pathJoin(outdir, "assembly_root.json"), JSON.stringify(asm, null, 2), "utf8");
      } else {
        const r = item.ref as AssemblyRef;
        const key = asmRefKey(r);
        if (seenAsm.has(key)) continue;
        seenAsm.add(key);

        const url = new URL(`${apiBase}/assemblies/d/${r.did}/m/${r.mv}/e/${r.eid}`);
        url.searchParams.set("includeMateFeatures", "true");
        url.searchParams.set("includeMateConnectors", "true");
        url.searchParams.set("includeNonSolids", "true");
        console.log(`Fetch SUB assembly: ${r.did} m/${r.mv} e/${r.eid}`);
        asm = (await fetchJson(url.toString(), headers)) as AssemblyResponse;

        const file = `${sanitizeName(r.did)}__${sanitizeName(r.mv)}__${sanitizeName(r.eid)}.json`;
        writeFileSync(pathJoin(outdir, "assemblies", file), JSON.stringify(asm, null, 2), "utf8");
      }

      if (!asm) continue;
      fetchedAssemblies++;

      const instances = allInstancesFromAsm(asm);

      // Enqueue subassemblies
      for (const inst of instances) {
        if (inst?.suppressed) continue;
        if ((inst?.type ?? "").toLowerCase() !== "assembly") continue;

        // We can only recurse if Onshape gave us immutable identifiers
        if (!inst.documentId || !inst.documentMicroversion || !inst.elementId) continue;

        q.push({ kind: "sub", ref: { did: inst.documentId, mv: inst.documentMicroversion, eid: inst.elementId } });
      }

      // Download part meshes
      if (downloadMeshes) {
        for (const inst of instances) {
          if (inst?.suppressed) continue;
          if ((inst?.type ?? "").toLowerCase() !== "part") continue;
          const pk = partKey(inst);
          if (!pk) continue;
          if (downloadedPartKeys.has(pk)) continue;

          downloadedPartKeys.add(pk);

          // choose filename
          const base = sanitizeName(inst.name ?? `part_${inst.partId}`);
          let file = `${base}.stl`;
          let attempt = 1;
          while (existsSync(pathJoin(meshDirAbs, file))) {
            attempt++;
            file = `${base}_${attempt}.stl`;
          }

          // STL endpoint requires microversion "m/<documentMicroversion>"
          const stlUrl = new URL(
            `${apiBase}/parts/d/${inst.documentId}/m/${inst.documentMicroversion}/e/${inst.elementId}/partid/${inst.partId}/stl`
          );
          stlUrl.searchParams.set("mode", "binary");
          stlUrl.searchParams.set("units", "meter");
          stlUrl.searchParams.set("scale", "1");
          stlUrl.searchParams.set("grouping", "true");

          try {
            console.log(`  STL: ${inst.name ?? inst.partId} -> ${meshDirRel}/${file}`);
            const bytes = await fetchBytesFollowRedirect(stlUrl.toString(), headers);
            writeFileSync(pathJoin(meshDirAbs, file), bytes);
            meshIndex[pk] = `${meshDirRel}/${file}`;
          } catch (e: any) {
            logError(outdir, { kind: "mesh_download_failed", partKey: pk, error: String(e?.message ?? e) });
          }
        }
      }
    } catch (e: any) {
      logError(outdir, { kind: "assembly_fetch_failed", item, error: String(e?.message ?? e) });
    }
  }

  const manifest = {
    createdAt: new Date().toISOString(),
    stack,
    apiVersion,
    root: { did, sourceWvm: wvm, sourceWvmid: wvmid, eid },
    effectiveRoot: { wvm: effectiveWvm, wvmid: effectiveWvmid, pinnedMicroversion },
    fetchedAssemblies,
    fetchedSubassemblies: seenAsm.size,
    downloadMeshes,
    meshDir: meshDirRel,
    meshesDownloaded: Object.keys(meshIndex).length,
  };

  writeFileSync(pathJoin(outdir, "manifest.json"), JSON.stringify(manifest, null, 2), "utf8");
  writeFileSync(pathJoin(outdir, "mesh_index.json"), JSON.stringify(meshIndex, null, 2), "utf8");

  console.log(`\nDone.`);
  console.log(`Snapshot: ${outdir}`);
  console.log(`Assemblies saved: assembly_root.json + assemblies/*.json`);
  if (downloadMeshes) console.log(`Meshes: ${meshDirRel}/ (count: ${Object.keys(meshIndex).length})`);
  console.log(`Errors (if any): errors.jsonl`);
}

main().catch((e) => die(String(e)));