use std::collections::BTreeSet;
use std::fs;
use std::path::{Path, PathBuf};

const STRICT_CUTOVER_ENV: &str = "PGE_PHYSICS_STRICT_CUTOVER_API";
const FORBIDDEN_PUBLIC_SYMBOLS: &[&str] = &[
    "rapier3d",
    "RapierPhysicsWorld",
    "RigidBody",
    "RigidBodyHandle",
    "ColliderHandle",
    "RigidBodySet",
    "ColliderSet",
    "ImpulseJointSet",
    "MultibodyJointSet",
    "PhysicsPipeline",
    "NarrowPhase",
    "QueryPipeline",
    "GenericJoint",
    "SharedShape",
    "PhysicsStep",
    "PhysicsSystem",
    "SharedPhysicsSystem",
    "LegacyPhysicsWorld",
];

fn rust_sources(root: &Path) -> Vec<PathBuf> {
    fn visit(path: &Path, sources: &mut Vec<PathBuf>) {
        for entry in fs::read_dir(path).expect("read PGE source directory") {
            let entry = entry.expect("read PGE source entry");
            let path = entry.path();
            if path.file_name().is_some_and(|name| name == "target") {
                continue;
            }
            if path.is_dir() {
                visit(&path, sources);
            } else if path.extension().is_some_and(|extension| extension == "rs") {
                sources.push(path);
            }
        }
    }

    let mut sources = Vec::new();
    visit(root, &mut sources);
    sources.sort();
    sources
}

fn normalized_signature(signature: &str) -> String {
    signature.split_whitespace().collect::<Vec<_>>().join(" ")
}

fn contains_identifier(signature: &str, symbol: &str) -> bool {
    signature
        .split(|character: char| !character.is_ascii_alphanumeric() && character != '_')
        .any(|identifier| identifier == symbol)
}

fn forbidden_public_signatures(root: &Path) -> BTreeSet<(String, String, String)> {
    let mut findings = BTreeSet::new();
    for path in rust_sources(root) {
        let source = fs::read_to_string(&path).expect("read PGE Rust source");
        let relative = path
            .strip_prefix(root)
            .expect("source is below PGE root")
            .to_string_lossy()
            .replace('\\', "/");
        let mut public_signature = String::new();
        for line in source.lines() {
            let trimmed = line.trim_start();
            if public_signature.is_empty() {
                if !trimmed.starts_with("pub ") {
                    continue;
                }
                public_signature.push_str(trimmed);
            } else {
                public_signature.push(' ');
                public_signature.push_str(trimmed);
            }
            let public_field = !public_signature.contains('(') && trimmed.ends_with(',');
            if !public_signature.contains('{') && !public_signature.contains(';') && !public_field {
                continue;
            }

            let signature = normalized_signature(&public_signature);
            for symbol in FORBIDDEN_PUBLIC_SYMBOLS {
                if *symbol == "rapier3d" && !signature.starts_with("pub use rapier3d") {
                    continue;
                }
                if contains_identifier(&signature, symbol) {
                    findings.insert((relative.clone(), (*symbol).to_string(), signature.clone()));
                }
            }
            public_signature.clear();
        }
    }
    findings
}

#[test]
fn strict_cutover_public_api_contains_no_backend_or_legacy_symbols() {
    let pge_root = Path::new(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .expect("pge-physics is inside the PGE workspace");
    let findings = forbidden_public_signatures(pge_root);
    assert!(
        findings.is_empty(),
        "strict cutover API exposes backend or legacy physics symbols:\n{findings:#?}\n{STRICT_CUTOVER_ENV}=1 is enabled in CI"
    );
}

#[test]
fn backend_imports_are_confined_to_the_private_runtime() {
    let pge_root = Path::new(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .expect("pge-physics is inside the PGE workspace");
    let allowed_backend_file = "pge-physics/src/runtime.rs";
    let scanner_file = "pge-physics/tests/public_api_boundary.rs";
    let mut findings = BTreeSet::new();
    for path in rust_sources(pge_root) {
        let relative = path
            .strip_prefix(pge_root)
            .expect("source is below PGE root")
            .to_string_lossy()
            .replace('\\', "/");
        if relative == allowed_backend_file || relative == scanner_file {
            continue;
        }
        let source = fs::read_to_string(&path).expect("read PGE Rust source");
        for (line_index, line) in source.lines().enumerate() {
            for symbol in [
                "rapier3d",
                "RapierPhysicsWorld",
                "PhysicsStep",
                "SharedPhysicsSystem",
                "LegacyPhysicsWorld",
            ] {
                if contains_identifier(line, symbol) {
                    findings.insert((relative.clone(), line_index + 1, symbol.to_string()));
                }
            }
        }
    }
    assert!(
        findings.is_empty(),
        "backend imports or removed legacy physics identifiers escaped the private runtime:\n{findings:#?}"
    );
}
