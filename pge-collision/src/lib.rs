//! Format-neutral, deterministic collision-shape candidate generation.
//!
//! The input mesh is expressed in its link or object's local coordinate frame.
//! Generated shapes deliberately remain candidates: mesh importers and product
//! tooling must select and persist reviewed shapes, mass properties, collision
//! groups, and material settings separately.

use std::collections::BTreeMap;
use std::fmt;
use std::path::{Path, PathBuf};

use serde::{Deserialize, Serialize};

/// A local-space point, in metres by convention.
pub type Point3 = [f32; 3];

/// Indexed triangle geometry in a single local coordinate frame.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct TriangleMesh {
    pub vertices: Vec<Point3>,
    pub triangles: Vec<[u32; 3]>,
}

/// Provenance recorded when local triangle geometry is loaded from an asset file.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct AssetMeshProvenance {
    pub source_path: PathBuf,
    pub format: AssetFormat,
    pub scene_index: usize,
    pub node_instance_count: usize,
    pub primitive_count: usize,
    pub source_vertex_count: usize,
    pub triangle_count: usize,
    /// All selected scene-node transforms were multiplied into the local positions.
    pub node_transforms_applied: bool,
}

/// Asset formats accepted by the PGE collision-asset loader.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum AssetFormat {
    Gltf,
    Glb,
}

/// Local geometry and provenance resolved from a GLTF or GLB asset.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct AssetTriangleMesh {
    pub mesh: TriangleMesh,
    pub provenance: AssetMeshProvenance,
}

/// Asset provenance paired with the existing collision-generation output.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct AssetCollisionCandidates {
    pub provenance: AssetMeshProvenance,
    pub candidates: CollisionCandidates,
}

/// URDF- or caller-owned placement of one visual asset in a robot link frame.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct LinkAssetTransform {
    pub translation_m: Point3,
    pub rotation_rpy_rad: Point3,
    pub scale: Point3,
}

impl Default for LinkAssetTransform {
    fn default() -> Self {
        Self {
            translation_m: [0.0; 3],
            rotation_rpy_rad: [0.0; 3],
            scale: [1.0; 3],
        }
    }
}

/// One visual asset to generate in its owning link's local frame.
///
/// `asset_id` is unique within `link_name`. It lets a physical link own multiple
/// visual meshes without losing the identity or placement of any source asset.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct LinkAssetColliderRequest {
    pub link_name: String,
    pub asset_id: String,
    pub asset_path: PathBuf,
    pub asset_to_link: LinkAssetTransform,
}

/// Successful collision generation for one link asset.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct LinkAssetCollisionCandidates {
    pub link_name: String,
    pub asset_id: String,
    pub asset_to_link: LinkAssetTransform,
    pub provenance: AssetMeshProvenance,
    pub candidates: CollisionCandidates,
}

/// A batch result, preserved in the input request order.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(tag = "status", rename_all = "snake_case")]
pub enum LinkAssetCollisionResult {
    Generated(LinkAssetCollisionCandidates),
    Failed(LinkAssetCollisionFailure),
}

/// Explicit failure record for a link asset; a failure never suppresses later assets.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct LinkAssetCollisionFailure {
    pub link_name: String,
    pub asset_id: String,
    pub asset_path: PathBuf,
    pub reason: LinkAssetCollisionFailureReason,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum LinkAssetCollisionFailureReason {
    EmptyLinkName,
    EmptyAssetId,
    DuplicateAssetId,
    InvalidAssetTransform,
    AssetLoad(String),
    CandidateGeneration(String),
}

/// A source asset included in a complete physical-link aggregate.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct LinkCollisionSourceAsset {
    pub asset_id: String,
    pub asset_to_link: LinkAssetTransform,
    pub provenance: AssetMeshProvenance,
}

/// Collision candidates generated from every visual asset belonging to one link.
///
/// `candidates` is generated from the merged, transformed source meshes, so its
/// compound candidate and reviewed-profile selection preserve one physical-link
/// semantic rather than creating unrelated bodies for individual visual meshes.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct LinkCollisionCandidates {
    pub link_name: String,
    pub source_assets: Vec<LinkCollisionSourceAsset>,
    pub candidates: CollisionCandidates,
}

/// One per-link aggregate result, ordered by its link's first request.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(tag = "status", rename_all = "snake_case")]
pub enum LinkCollisionAggregateResult {
    Generated(LinkCollisionCandidates),
    Failed(LinkCollisionAggregateFailure),
}

/// A link cannot safely get a whole-link collider because one or more assets did
/// not produce geometry. Per-asset results retain the individual diagnostics.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct LinkCollisionAggregateFailure {
    pub link_name: String,
    pub asset_count: usize,
    pub generated_asset_count: usize,
    pub reason: LinkCollisionAggregateFailureReason,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum LinkCollisionAggregateFailureReason {
    AssetFailures { failed_asset_ids: Vec<String> },
    CandidateGeneration(String),
}

/// Deterministic per-asset and physical-link generation report.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct LinkAssetCollisionReport {
    /// One result per request, in input order, retaining per-asset provenance or
    /// a specific failure.
    pub results: Vec<LinkAssetCollisionResult>,
    /// One aggregate per distinct link, ordered by first request. A generated
    /// aggregate always includes every source asset belonging to that link.
    pub link_results: Vec<LinkCollisionAggregateResult>,
}

impl LinkAssetCollisionReport {
    pub fn generated(&self) -> impl Iterator<Item = &LinkAssetCollisionCandidates> {
        self.results.iter().filter_map(|result| match result {
            LinkAssetCollisionResult::Generated(generated) => Some(generated),
            LinkAssetCollisionResult::Failed(_) => None,
        })
    }

    pub fn failures(&self) -> impl Iterator<Item = &LinkAssetCollisionFailure> {
        self.results.iter().filter_map(|result| match result {
            LinkAssetCollisionResult::Generated(_) => None,
            LinkAssetCollisionResult::Failed(failure) => Some(failure),
        })
    }

    pub fn generated_links(&self) -> impl Iterator<Item = &LinkCollisionCandidates> {
        self.link_results.iter().filter_map(|result| match result {
            LinkCollisionAggregateResult::Generated(generated) => Some(generated),
            LinkCollisionAggregateResult::Failed(_) => None,
        })
    }

    pub fn link_failures(&self) -> impl Iterator<Item = &LinkCollisionAggregateFailure> {
        self.link_results.iter().filter_map(|result| match result {
            LinkCollisionAggregateResult::Generated(_) => None,
            LinkCollisionAggregateResult::Failed(failure) => Some(failure),
        })
    }
}

/// Why a GLTF/GLB asset could not produce local triangle geometry.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum AssetMeshLoadError {
    UnsupportedFormat {
        path: PathBuf,
    },
    Import {
        path: PathBuf,
        message: String,
    },
    MissingScene {
        path: PathBuf,
    },
    UnsupportedPrimitiveMode {
        node: usize,
        mesh: usize,
        primitive: usize,
        mode: String,
    },
    MissingPositions {
        node: usize,
        mesh: usize,
        primitive: usize,
    },
    InvalidTriangleIndexCount {
        node: usize,
        mesh: usize,
        primitive: usize,
        count: usize,
    },
    TriangleIndexOutOfBounds {
        node: usize,
        mesh: usize,
        primitive: usize,
        index: u32,
        vertex_count: usize,
    },
}

impl fmt::Display for AssetMeshLoadError {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::UnsupportedFormat { path } => write!(formatter, "unsupported collision asset format: {}", path.display()),
            Self::Import { path, message } => write!(formatter, "could not import collision asset '{}': {message}", path.display()),
            Self::MissingScene { path } => write!(formatter, "collision asset '{}' has no scene", path.display()),
            Self::UnsupportedPrimitiveMode { node, mesh, primitive, mode } => write!(formatter, "node {node} mesh {mesh} primitive {primitive} uses unsupported mode {mode}"),
            Self::MissingPositions { node, mesh, primitive } => write!(formatter, "node {node} mesh {mesh} primitive {primitive} has no positions"),
            Self::InvalidTriangleIndexCount { node, mesh, primitive, count } => write!(formatter, "node {node} mesh {mesh} primitive {primitive} has {count} indices, not a triangle list"),
            Self::TriangleIndexOutOfBounds { node, mesh, primitive, index, vertex_count } => write!(formatter, "node {node} mesh {mesh} primitive {primitive} references vertex {index}, but has {vertex_count} vertices"),
        }
    }
}

impl std::error::Error for AssetMeshLoadError {}

/// Loads the default GLTF/GLB scene into one deterministic local `TriangleMesh`.
///
/// Every selected scene-node transform, including parent transforms, is baked into
/// positions. Only triangle-list primitives are accepted; non-triangle primitive
/// modes fail explicitly rather than silently producing a different collider.
pub fn load_gltf_triangle_mesh(
    path: impl AsRef<Path>,
) -> Result<AssetTriangleMesh, AssetMeshLoadError> {
    let path = path.as_ref().to_path_buf();
    let format = asset_format(&path)?;
    let (document, buffers, _) =
        gltf::import(&path).map_err(|error| AssetMeshLoadError::Import {
            path: path.clone(),
            message: error.to_string(),
        })?;
    let scene = document
        .default_scene()
        .or_else(|| document.scenes().next())
        .ok_or_else(|| AssetMeshLoadError::MissingScene { path: path.clone() })?;

    let mut mesh = TriangleMesh {
        vertices: Vec::new(),
        triangles: Vec::new(),
    };
    let mut counts = AssetMeshCounts::default();
    for node in scene.nodes() {
        append_gltf_node(node, identity_matrix(), &buffers, &mut mesh, &mut counts)?;
    }
    Ok(AssetTriangleMesh {
        provenance: AssetMeshProvenance {
            source_path: path,
            format,
            scene_index: scene.index(),
            node_instance_count: counts.node_instance_count,
            primitive_count: counts.primitive_count,
            source_vertex_count: counts.source_vertex_count,
            triangle_count: mesh.triangles.len(),
            node_transforms_applied: true,
        },
        mesh,
    })
}

/// Loads a GLTF/GLB asset and sends its resolved local mesh through the existing
/// candidate, quality, and reviewed-profile selection pipeline.
pub fn generate_collision_candidates_from_gltf(
    path: impl AsRef<Path>,
    config: CollisionGenerationConfig,
) -> Result<AssetCollisionCandidates, AssetCollisionGenerationError> {
    let asset = load_gltf_triangle_mesh(path).map_err(AssetCollisionGenerationError::AssetLoad)?;
    let candidates = generate_collision_candidates(&asset.mesh, config)
        .map_err(AssetCollisionGenerationError::Generation)?;
    Ok(AssetCollisionCandidates {
        provenance: asset.provenance,
        candidates,
    })
}

/// Generates per-asset and physical-link collision candidates without requiring a
/// URDF parser. A link may own multiple assets when every `asset_id` within that
/// link is unique. Per-asset results preserve request order; link aggregates are
/// ordered by first occurrence. An aggregate is generated only when every one of
/// its source assets generated successfully, so no physical-link mesh is silently
/// omitted from a runtime collider.
pub fn generate_link_asset_collision_candidates(
    requests: &[LinkAssetColliderRequest],
    config: CollisionGenerationConfig,
) -> LinkAssetCollisionReport {
    let mut asset_counts = BTreeMap::<(&str, &str), usize>::new();
    let mut link_request_indices = BTreeMap::<&str, Vec<usize>>::new();
    let mut link_order = Vec::new();
    for (index, request) in requests.iter().enumerate() {
        *asset_counts
            .entry((&request.link_name, &request.asset_id))
            .or_default() += 1;
        if !link_request_indices.contains_key(request.link_name.as_str()) {
            link_order.push(request.link_name.as_str());
        }
        link_request_indices
            .entry(&request.link_name)
            .or_default()
            .push(index);
    }
    let generated_assets = requests
        .iter()
        .map(|request| generate_link_asset_collision_candidate(request, config, &asset_counts))
        .collect::<Vec<_>>();
    let results = generated_assets
        .iter()
        .map(|generated| generated.result.clone())
        .collect::<Vec<_>>();
    let link_results = link_order
        .into_iter()
        .map(|link_name| {
            aggregate_link_collision_candidates(
                link_name,
                &link_request_indices[link_name],
                requests,
                &generated_assets,
                config,
            )
        })
        .collect();
    LinkAssetCollisionReport {
        results,
        link_results,
    }
}

#[derive(Clone, Debug)]
struct GeneratedLinkAsset {
    result: LinkAssetCollisionResult,
    transformed_mesh: Option<TriangleMesh>,
}

fn generate_link_asset_collision_candidate(
    request: &LinkAssetColliderRequest,
    config: CollisionGenerationConfig,
    asset_counts: &BTreeMap<(&str, &str), usize>,
) -> GeneratedLinkAsset {
    let failure = |reason| GeneratedLinkAsset {
        result: LinkAssetCollisionResult::Failed(LinkAssetCollisionFailure {
            link_name: request.link_name.clone(),
            asset_id: request.asset_id.clone(),
            asset_path: request.asset_path.clone(),
            reason,
        }),
        transformed_mesh: None,
    };
    if request.link_name.trim().is_empty() {
        return failure(LinkAssetCollisionFailureReason::EmptyLinkName);
    }
    if request.asset_id.trim().is_empty() {
        return failure(LinkAssetCollisionFailureReason::EmptyAssetId);
    }
    if asset_counts[&(request.link_name.as_str(), request.asset_id.as_str())] > 1 {
        return failure(LinkAssetCollisionFailureReason::DuplicateAssetId);
    }
    if !valid_link_asset_transform(request.asset_to_link) {
        return failure(LinkAssetCollisionFailureReason::InvalidAssetTransform);
    }
    let mut asset = match load_gltf_triangle_mesh(&request.asset_path) {
        Ok(asset) => asset,
        Err(error) => {
            return failure(LinkAssetCollisionFailureReason::AssetLoad(
                error.to_string(),
            ))
        }
    };
    for point in &mut asset.mesh.vertices {
        *point = transform_link_asset_point(*point, request.asset_to_link);
    }
    let candidates = match generate_collision_candidates(&asset.mesh, config) {
        Ok(candidates) => candidates,
        Err(error) => {
            return failure(LinkAssetCollisionFailureReason::CandidateGeneration(
                error.to_string(),
            ))
        }
    };
    GeneratedLinkAsset {
        result: LinkAssetCollisionResult::Generated(LinkAssetCollisionCandidates {
            link_name: request.link_name.clone(),
            asset_id: request.asset_id.clone(),
            asset_to_link: request.asset_to_link,
            provenance: asset.provenance,
            candidates,
        }),
        transformed_mesh: Some(asset.mesh),
    }
}

fn aggregate_link_collision_candidates(
    link_name: &str,
    request_indices: &[usize],
    requests: &[LinkAssetColliderRequest],
    generated_assets: &[GeneratedLinkAsset],
    config: CollisionGenerationConfig,
) -> LinkCollisionAggregateResult {
    let generated_asset_count = request_indices
        .iter()
        .filter(|&&index| {
            matches!(
                generated_assets[index].result,
                LinkAssetCollisionResult::Generated(_)
            )
        })
        .count();
    if generated_asset_count != request_indices.len() {
        let failed_asset_ids = request_indices
            .iter()
            .filter_map(|&index| match generated_assets[index].result {
                LinkAssetCollisionResult::Generated(_) => None,
                LinkAssetCollisionResult::Failed(_) => Some(requests[index].asset_id.clone()),
            })
            .collect();
        return LinkCollisionAggregateResult::Failed(LinkCollisionAggregateFailure {
            link_name: link_name.to_owned(),
            asset_count: request_indices.len(),
            generated_asset_count,
            reason: LinkCollisionAggregateFailureReason::AssetFailures { failed_asset_ids },
        });
    }

    let mut merged_mesh = TriangleMesh {
        vertices: Vec::new(),
        triangles: Vec::new(),
    };
    let mut source_assets = Vec::with_capacity(request_indices.len());
    for &index in request_indices {
        let generated = match &generated_assets[index].result {
            LinkAssetCollisionResult::Generated(generated) => generated,
            LinkAssetCollisionResult::Failed(_) => unreachable!("checked all assets above"),
        };
        append_triangle_mesh(
            &mut merged_mesh,
            generated_assets[index]
                .transformed_mesh
                .as_ref()
                .expect("generated assets retain their transformed mesh"),
        );
        source_assets.push(LinkCollisionSourceAsset {
            asset_id: generated.asset_id.clone(),
            asset_to_link: generated.asset_to_link,
            provenance: generated.provenance.clone(),
        });
    }
    match generate_collision_candidates(&merged_mesh, config) {
        Ok(candidates) => LinkCollisionAggregateResult::Generated(LinkCollisionCandidates {
            link_name: link_name.to_owned(),
            source_assets,
            candidates,
        }),
        Err(error) => LinkCollisionAggregateResult::Failed(LinkCollisionAggregateFailure {
            link_name: link_name.to_owned(),
            asset_count: request_indices.len(),
            generated_asset_count,
            reason: LinkCollisionAggregateFailureReason::CandidateGeneration(error.to_string()),
        }),
    }
}

fn append_triangle_mesh(output: &mut TriangleMesh, input: &TriangleMesh) {
    let base = output.vertices.len() as u32;
    output.vertices.extend_from_slice(&input.vertices);
    output.triangles.extend(
        input
            .triangles
            .iter()
            .map(|triangle| [base + triangle[0], base + triangle[1], base + triangle[2]]),
    );
}

fn valid_link_asset_transform(transform: LinkAssetTransform) -> bool {
    transform
        .translation_m
        .iter()
        .chain(transform.rotation_rpy_rad.iter())
        .chain(transform.scale.iter())
        .all(|value| value.is_finite())
        && transform.scale.iter().all(|value| *value != 0.0)
}

fn transform_link_asset_point(point: Point3, transform: LinkAssetTransform) -> Point3 {
    let scaled = [
        point[0] * transform.scale[0],
        point[1] * transform.scale[1],
        point[2] * transform.scale[2],
    ];
    let rotation = rpy_matrix(transform.rotation_rpy_rad);
    add(transform_point(rotation, scaled), transform.translation_m)
}

fn rpy_matrix([roll, pitch, yaw]: Point3) -> [[f32; 4]; 4] {
    let (sin_roll, cos_roll) = roll.sin_cos();
    let (sin_pitch, cos_pitch) = pitch.sin_cos();
    let (sin_yaw, cos_yaw) = yaw.sin_cos();
    [
        [cos_yaw * cos_pitch, sin_yaw * cos_pitch, -sin_pitch, 0.0],
        [
            cos_yaw * sin_pitch * sin_roll - sin_yaw * cos_roll,
            sin_yaw * sin_pitch * sin_roll + cos_yaw * cos_roll,
            cos_pitch * sin_roll,
            0.0,
        ],
        [
            cos_yaw * sin_pitch * cos_roll + sin_yaw * sin_roll,
            sin_yaw * sin_pitch * cos_roll - cos_yaw * sin_roll,
            cos_pitch * cos_roll,
            0.0,
        ],
        [0.0, 0.0, 0.0, 1.0],
    ]
}

/// Errors spanning asset resolution and existing candidate generation.
#[derive(Clone, Debug, PartialEq)]
pub enum AssetCollisionGenerationError {
    AssetLoad(AssetMeshLoadError),
    Generation(CollisionGenerationError),
}

impl fmt::Display for AssetCollisionGenerationError {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::AssetLoad(error) => error.fmt(formatter),
            Self::Generation(error) => error.fmt(formatter),
        }
    }
}

impl std::error::Error for AssetCollisionGenerationError {}

#[derive(Default)]
struct AssetMeshCounts {
    node_instance_count: usize,
    primitive_count: usize,
    source_vertex_count: usize,
}

fn asset_format(path: &Path) -> Result<AssetFormat, AssetMeshLoadError> {
    match path.extension().and_then(|extension| extension.to_str()) {
        Some(extension) if extension.eq_ignore_ascii_case("gltf") => Ok(AssetFormat::Gltf),
        Some(extension) if extension.eq_ignore_ascii_case("glb") => Ok(AssetFormat::Glb),
        _ => Err(AssetMeshLoadError::UnsupportedFormat {
            path: path.to_path_buf(),
        }),
    }
}

fn append_gltf_node(
    node: gltf::Node<'_>,
    parent_transform: [[f32; 4]; 4],
    buffers: &[gltf::buffer::Data],
    output: &mut TriangleMesh,
    counts: &mut AssetMeshCounts,
) -> Result<(), AssetMeshLoadError> {
    counts.node_instance_count += 1;
    let transform = multiply_matrices(parent_transform, node.transform().matrix());
    if let Some(mesh) = node.mesh() {
        for primitive in mesh.primitives() {
            if primitive.mode() != gltf::mesh::Mode::Triangles {
                return Err(AssetMeshLoadError::UnsupportedPrimitiveMode {
                    node: node.index(),
                    mesh: mesh.index(),
                    primitive: primitive.index(),
                    mode: format!("{:?}", primitive.mode()),
                });
            }
            let reader = primitive.reader(|buffer| Some(&buffers[buffer.index()].0));
            let positions = reader
                .read_positions()
                .ok_or(AssetMeshLoadError::MissingPositions {
                    node: node.index(),
                    mesh: mesh.index(),
                    primitive: primitive.index(),
                })?
                .map(|position| transform_point(transform, position))
                .collect::<Vec<_>>();
            let vertex_count = positions.len();
            let indices = reader
                .read_indices()
                .map(|indices| indices.into_u32().collect::<Vec<_>>())
                .unwrap_or_else(|| (0..positions.len() as u32).collect());
            if indices.len() % 3 != 0 {
                return Err(AssetMeshLoadError::InvalidTriangleIndexCount {
                    node: node.index(),
                    mesh: mesh.index(),
                    primitive: primitive.index(),
                    count: indices.len(),
                });
            }
            for &index in &indices {
                if index as usize >= positions.len() {
                    return Err(AssetMeshLoadError::TriangleIndexOutOfBounds {
                        node: node.index(),
                        mesh: mesh.index(),
                        primitive: primitive.index(),
                        index,
                        vertex_count: positions.len(),
                    });
                }
            }
            let base = output.vertices.len() as u32;
            output.vertices.extend(positions);
            output.triangles.extend(
                indices
                    .chunks_exact(3)
                    .map(|triangle| [base + triangle[0], base + triangle[1], base + triangle[2]]),
            );
            counts.primitive_count += 1;
            counts.source_vertex_count += vertex_count;
        }
    }
    for child in node.children() {
        append_gltf_node(child, transform, buffers, output, counts)?;
    }
    Ok(())
}

fn identity_matrix() -> [[f32; 4]; 4] {
    [
        [1.0, 0.0, 0.0, 0.0],
        [0.0, 1.0, 0.0, 0.0],
        [0.0, 0.0, 1.0, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ]
}

fn multiply_matrices(left: [[f32; 4]; 4], right: [[f32; 4]; 4]) -> [[f32; 4]; 4] {
    let mut result = [[0.0; 4]; 4];
    for column in 0..4 {
        for row in 0..4 {
            result[column][row] = (0..4)
                .map(|index| left[index][row] * right[column][index])
                .sum();
        }
    }
    result
}

fn transform_point(matrix: [[f32; 4]; 4], point: Point3) -> Point3 {
    let homogeneous = [point[0], point[1], point[2], 1.0];
    let transformed = (0..4)
        .map(|row| {
            (0..4)
                .map(|column| matrix[column][row] * homogeneous[column])
                .sum::<f32>()
        })
        .collect::<Vec<_>>();
    let divisor = transformed[3];
    if divisor != 0.0 && divisor != 1.0 {
        [
            transformed[0] / divisor,
            transformed[1] / divisor,
            transformed[2] / divisor,
        ]
    } else {
        [transformed[0], transformed[1], transformed[2]]
    }
}

/// Settings controlling conservative generated candidates.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct CollisionGenerationConfig {
    /// Maximum number of axis-aligned boxes in the compound candidate.
    pub max_compound_parts: usize,
    /// Minimum non-zero size for every generated box dimension, in local units.
    pub minimum_extent: f32,
    /// Maximum disconnected components eligible for exact compound merging.
    /// Larger meshes emit a conservative bounds fallback with explicit evidence.
    pub maximum_exact_partitions: usize,
    /// Maximum usable triangles eligible for exact compound decomposition.
    /// Larger meshes emit a conservative bounds fallback with explicit evidence.
    pub maximum_exact_triangles: usize,
}

impl Default for CollisionGenerationConfig {
    fn default() -> Self {
        Self {
            max_compound_parts: 4,
            minimum_extent: 0.001,
            maximum_exact_partitions: 32,
            maximum_exact_triangles: 20_000,
        }
    }
}

/// The cardinal local axis used by an axis-aligned primitive or partition.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum Axis {
    X,
    Y,
    Z,
}

impl Axis {
    fn index(self) -> usize {
        match self {
            Self::X => 0,
            Self::Y => 1,
            Self::Z => 2,
        }
    }
}

/// A conservative axis-aligned box in the input mesh's local frame.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct BoxCandidate {
    pub center: Point3,
    pub size: Point3,
}

/// A conservative local-space sphere.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct SphereCandidate {
    pub center: Point3,
    pub radius: f32,
}

/// A cylinder whose central axis is aligned with a local cardinal axis.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct CylinderCandidate {
    pub center: Point3,
    pub axis: Axis,
    pub radius: f32,
    pub height: f32,
}

/// A capsule described by its local-space segment endpoints and radius.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct CapsuleCandidate {
    pub a: Point3,
    pub b: Point3,
    pub radius: f32,
}

/// A single conservative primitive candidate.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub enum PrimitiveCandidate {
    Box(BoxCandidate),
    Sphere(SphereCandidate),
    Cylinder(CylinderCandidate),
    Capsule(CapsuleCandidate),
}

/// A group of boxes whose union conservatively covers every usable input triangle.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct CompoundCandidate {
    /// Axis used for the first volume-reducing recursive split, or the longest
    /// bounds axis if no split reduced volume.
    pub partition_axis: Axis,
    pub parts: Vec<BoxCandidate>,
    /// Deterministic evidence for reviewing or automatically selecting this candidate.
    pub quality: CompoundCandidateQuality,
}

/// Conservative quality measurements for one compound candidate.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct CompoundCandidateQuality {
    /// Number of usable source triangles evaluated for coverage.
    pub source_triangle_count: usize,
    /// Source triangles fully enclosed by at least one compound part.
    pub covered_triangle_count: usize,
    /// Sum of conservative part volumes in local cubic units.
    pub conservative_volume: f32,
    /// `conservative_volume / source_bounds_volume`; lower is tighter.
    pub bounds_volume_ratio: f32,
    /// Whether exact decomposition ran or an explicit conservative fallback was
    /// used to keep large-mesh work bounded.
    pub generation_path: CompoundGenerationPath,
}

/// How the compound candidate was generated.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum CompoundGenerationPath {
    Exact,
    BoundsFallback {
        reason: CompoundGenerationFallbackReason,
    },
}

/// A transparent reason exact compound decomposition was skipped.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum CompoundGenerationFallbackReason {
    TriangleLimit {
        source_triangle_count: usize,
        maximum_exact_triangles: usize,
    },
    PartitionLimit {
        connected_partition_count: usize,
        maximum_exact_partitions: usize,
    },
}

/// A deterministic point cloud to pass to a physics backend's convex-hull builder.
///
/// Points are finite, locally expressed, lexicographically sorted, and deduplicated.
/// This crate does not invent a backend-specific hull topology. Consumers such as a
/// physics adapter should construct the convex hull and report a backend failure when
/// these points are coplanar or otherwise cannot form a solid hull.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct ConvexHullCandidate {
    pub points: Vec<Point3>,
}

/// All generated options for a mesh. Serializing this value gives tooling a stable,
/// reviewable artifact before any candidate becomes an approved runtime collider.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct CollisionCandidates {
    pub bounds: BoxCandidate,
    pub primitives: Vec<PrimitiveCandidate>,
    pub compounds: Vec<CompoundCandidate>,
    pub convex_hull: ConvexHullCandidate,
}

/// A narrow, reviewable JSON profile that can be consumed by RobotDreams' vehicle
/// collider loader. It intentionally contains only primitive types supported by that
/// loader and never contains generated mesh or convex-hull data.
///
/// Its JSON form is exactly:
///
/// ```json
/// { "colliders": [{ "shape": "box", "size": [0.2, 0.1, 0.1],
///                   "offset": [0.0, 0.0, 0.05], "rotation": [0.0, 0.0, 0.0] }] }
/// ```
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct ReviewedCollisionProfile {
    pub colliders: Vec<ReviewedCollider>,
}

/// Why the bounded compound selector chose its emitted profile.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum CompoundSelectionDecision {
    Compound { candidate_index: usize },
    BoundsFallback { reason: CompoundFallbackReason },
}

/// A reason that no fully-covered compound candidate fit the requested budget.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum CompoundFallbackReason {
    NoCompoundCandidate,
    ExceedsColliderBudget,
    IncompleteSourceCoverage,
}

/// Evidence returned alongside an automatically selected reviewed profile.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct CompoundSelectionEvidence {
    pub decision: CompoundSelectionDecision,
    pub maximum_colliders: usize,
    pub source_triangle_count: usize,
    pub covered_triangle_count: usize,
    pub conservative_volume: f32,
    pub bounds_volume_ratio: f32,
    pub collider_count: usize,
    /// Whether the selected compound was exactly decomposed or conservatively
    /// fell back to source bounds because its decomposition work was bounded.
    pub compound_generation_path: CompoundGenerationPath,
}

/// A profile plus deterministic selection evidence. Persist the `profile` after human
/// review and retain `evidence` with generation provenance for later audit.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct ReviewedCompoundProfileSelection {
    pub profile: ReviewedCollisionProfile,
    pub evidence: CompoundSelectionEvidence,
}

impl ReviewedCollisionProfile {
    /// Serializes the exact interchange JSON accepted by RobotDreams.
    pub fn to_json_pretty(&self) -> Result<String, serde_json::Error> {
        serde_json::to_string_pretty(self)
    }
}

/// A transformed primitive supported by the reviewed-profile interchange.
///
/// `offset` and `rotation` are local-link translation and roll/pitch/yaw radians.
/// Box sizes and cylinder heights are full extents.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(tag = "shape", rename_all = "lowercase")]
pub enum ReviewedCollider {
    Box {
        size: Point3,
        offset: Point3,
        rotation: Point3,
    },
    Sphere {
        radius: f32,
        offset: Point3,
        rotation: Point3,
    },
    Cylinder {
        radius: f32,
        height: f32,
        offset: Point3,
        rotation: Point3,
    },
}

/// Deterministic choice used to export candidates into a supported profile.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
pub enum ReviewedProfileSelection {
    /// Use the generated compound boxes when they fit the collider budget; otherwise
    /// use the single conservative bounds box.
    #[default]
    CompoundBoxes,
    BoundsBox,
    BoundingSphere,
    Cylinder {
        axis: Axis,
    },
}

/// Limits and choice for a reviewed-profile export.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct ReviewedProfileExportConfig {
    pub selection: ReviewedProfileSelection,
    /// The maximum number of profile colliders. A compound over this budget falls
    /// back to one bounds box, preserving conservative coverage deterministically.
    pub maximum_colliders: usize,
}

impl Default for ReviewedProfileExportConfig {
    fn default() -> Self {
        Self {
            selection: ReviewedProfileSelection::CompoundBoxes,
            maximum_colliders: 4,
        }
    }
}

/// Why a reviewed-profile export could not be created.
#[derive(Clone, Debug, PartialEq)]
pub enum ReviewedProfileExportError {
    InvalidColliderBudget,
    MissingCompoundCandidate,
    MissingSphereCandidate,
    MissingCylinderCandidate(Axis),
}

impl fmt::Display for ReviewedProfileExportError {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::InvalidColliderBudget => {
                write!(formatter, "maximum_colliders must be at least one")
            }
            Self::MissingCompoundCandidate => write!(
                formatter,
                "collision candidates contain no compound box candidate"
            ),
            Self::MissingSphereCandidate => write!(
                formatter,
                "collision candidates contain no sphere candidate"
            ),
            Self::MissingCylinderCandidate(axis) => {
                write!(
                    formatter,
                    "collision candidates contain no {axis:?} cylinder candidate"
                )
            }
        }
    }
}

impl std::error::Error for ReviewedProfileExportError {}

impl CollisionCandidates {
    /// Selects the fully-covered compound with the lowest conservative volume that
    /// fits `maximum_colliders`. Ties resolve by fewer colliders and source order.
    /// If none qualifies, returns the conservative bounds box with an explicit reason.
    pub fn select_compound_profile(
        &self,
        maximum_colliders: usize,
    ) -> Result<ReviewedCompoundProfileSelection, ReviewedProfileExportError> {
        if maximum_colliders == 0 {
            return Err(ReviewedProfileExportError::InvalidColliderBudget);
        }

        let source_triangle_count = self
            .compounds
            .first()
            .map(|candidate| candidate.quality.source_triangle_count)
            .unwrap_or_default();
        let selected = self
            .compounds
            .iter()
            .enumerate()
            .filter(|(_, candidate)| {
                candidate.parts.len() <= maximum_colliders
                    && candidate.quality.covered_triangle_count
                        == candidate.quality.source_triangle_count
            })
            .min_by(|(left_index, left), (right_index, right)| {
                left.quality
                    .conservative_volume
                    .total_cmp(&right.quality.conservative_volume)
                    .then_with(|| left.parts.len().cmp(&right.parts.len()))
                    .then_with(|| left_index.cmp(right_index))
            });

        if let Some((index, candidate)) = selected {
            return Ok(ReviewedCompoundProfileSelection {
                profile: ReviewedCollisionProfile {
                    colliders: candidate.parts.iter().copied().map(profile_box).collect(),
                },
                evidence: CompoundSelectionEvidence {
                    decision: CompoundSelectionDecision::Compound {
                        candidate_index: index,
                    },
                    maximum_colliders,
                    source_triangle_count: candidate.quality.source_triangle_count,
                    covered_triangle_count: candidate.quality.covered_triangle_count,
                    conservative_volume: candidate.quality.conservative_volume,
                    bounds_volume_ratio: candidate.quality.bounds_volume_ratio,
                    collider_count: candidate.parts.len(),
                    compound_generation_path: candidate.quality.generation_path,
                },
            });
        }

        let reason = if self.compounds.is_empty() {
            CompoundFallbackReason::NoCompoundCandidate
        } else if self
            .compounds
            .iter()
            .any(|candidate| candidate.parts.len() <= maximum_colliders)
        {
            CompoundFallbackReason::IncompleteSourceCoverage
        } else {
            CompoundFallbackReason::ExceedsColliderBudget
        };
        let compound_generation_path = self
            .compounds
            .first()
            .map(|candidate| candidate.quality.generation_path)
            .unwrap_or(CompoundGenerationPath::Exact);
        Ok(ReviewedCompoundProfileSelection {
            profile: ReviewedCollisionProfile {
                colliders: vec![profile_box(self.bounds)],
            },
            evidence: CompoundSelectionEvidence {
                decision: CompoundSelectionDecision::BoundsFallback { reason },
                maximum_colliders,
                source_triangle_count,
                covered_triangle_count: source_triangle_count,
                conservative_volume: box_volume(self.bounds),
                bounds_volume_ratio: 1.0,
                collider_count: 1,
                compound_generation_path,
            },
        })
    }

    /// Exports one deterministic, conservative choice as a RobotDreams-compatible
    /// profile. The caller remains responsible for visually reviewing and approving
    /// the returned artifact before using it at simulation startup.
    pub fn export_reviewed_profile(
        &self,
        config: ReviewedProfileExportConfig,
    ) -> Result<ReviewedCollisionProfile, ReviewedProfileExportError> {
        if config.maximum_colliders == 0 {
            return Err(ReviewedProfileExportError::InvalidColliderBudget);
        }

        let colliders = match config.selection {
            ReviewedProfileSelection::CompoundBoxes => {
                self.select_compound_profile(config.maximum_colliders)?
                    .profile
                    .colliders
            }
            ReviewedProfileSelection::BoundsBox => vec![profile_box(self.bounds)],
            ReviewedProfileSelection::BoundingSphere => {
                let sphere = self
                    .primitives
                    .iter()
                    .find_map(|candidate| match candidate {
                        PrimitiveCandidate::Sphere(sphere) => Some(*sphere),
                        _ => None,
                    })
                    .ok_or(ReviewedProfileExportError::MissingSphereCandidate)?;
                vec![ReviewedCollider::Sphere {
                    radius: sphere.radius,
                    offset: sphere.center,
                    rotation: [0.0; 3],
                }]
            }
            ReviewedProfileSelection::Cylinder { axis } => {
                let cylinder = self
                    .primitives
                    .iter()
                    .find_map(|candidate| match candidate {
                        PrimitiveCandidate::Cylinder(cylinder) if cylinder.axis == axis => {
                            Some(*cylinder)
                        }
                        _ => None,
                    })
                    .ok_or(ReviewedProfileExportError::MissingCylinderCandidate(axis))?;
                vec![ReviewedCollider::Cylinder {
                    radius: cylinder.radius,
                    height: cylinder.height,
                    offset: cylinder.center,
                    rotation: cylinder_rotation(axis),
                }]
            }
        };

        Ok(ReviewedCollisionProfile { colliders })
    }
}

fn profile_box(candidate: BoxCandidate) -> ReviewedCollider {
    ReviewedCollider::Box {
        size: candidate.size,
        offset: candidate.center,
        rotation: [0.0; 3],
    }
}

fn cylinder_rotation(axis: Axis) -> Point3 {
    match axis {
        Axis::X => [0.0, 0.0, std::f32::consts::FRAC_PI_2],
        Axis::Y => [0.0; 3],
        Axis::Z => [std::f32::consts::FRAC_PI_2, 0.0, 0.0],
    }
}

/// Why candidate generation could not safely produce a result.
#[derive(Clone, Debug, PartialEq)]
pub enum CollisionGenerationError {
    InvalidConfig(&'static str),
    NonFiniteVertex { index: usize },
    TriangleIndexOutOfBounds { triangle: usize, index: u32 },
    NoUsableTriangles,
}

impl fmt::Display for CollisionGenerationError {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::InvalidConfig(message) => {
                write!(formatter, "invalid collision generation config: {message}")
            }
            Self::NonFiniteVertex { index } => {
                write!(formatter, "mesh vertex {index} is not finite")
            }
            Self::TriangleIndexOutOfBounds { triangle, index } => {
                write!(
                    formatter,
                    "triangle {triangle} references missing vertex {index}"
                )
            }
            Self::NoUsableTriangles => {
                write!(formatter, "mesh contains no non-degenerate triangles")
            }
        }
    }
}

impl std::error::Error for CollisionGenerationError {}

/// Generates conservative primitive, compound-box, and convex-hull candidates.
///
/// Degenerate triangles are ignored after index and finite-value validation. This is
/// intentional: exporters often include zero-area triangles, but they cannot affect a
/// collision volume. Every non-degenerate source triangle is covered by the generated
/// box and compound candidates.
pub fn generate_collision_candidates(
    mesh: &TriangleMesh,
    config: CollisionGenerationConfig,
) -> Result<CollisionCandidates, CollisionGenerationError> {
    validate_config(config)?;
    validate_mesh(mesh)?;

    let triangles = usable_triangles(mesh);
    if triangles.is_empty() {
        return Err(CollisionGenerationError::NoUsableTriangles);
    }

    let used_points = triangles
        .iter()
        .flat_map(|triangle| triangle.iter().map(|&index| mesh.vertices[index]))
        .collect::<Vec<_>>();
    let bounds = box_from_points(&used_points, config.minimum_extent);
    let partition_axis = longest_axis(bounds.size);

    let mut primitives = Vec::with_capacity(8);
    primitives.push(PrimitiveCandidate::Box(bounds));
    primitives.push(PrimitiveCandidate::Sphere(sphere_from_points(
        &used_points,
        bounds.center,
    )));
    for axis in [Axis::X, Axis::Y, Axis::Z] {
        primitives.push(PrimitiveCandidate::Cylinder(cylinder_from_points(
            &used_points,
            axis,
            config.minimum_extent,
        )));
        primitives.push(PrimitiveCandidate::Capsule(capsule_from_points(
            &used_points,
            axis,
        )));
    }

    let compound = partition_compound(mesh, &triangles, partition_axis, config);
    let hull_points = unique_sorted_points(&used_points);

    Ok(CollisionCandidates {
        bounds,
        primitives,
        compounds: vec![compound],
        convex_hull: ConvexHullCandidate {
            points: hull_points,
        },
    })
}

fn validate_config(config: CollisionGenerationConfig) -> Result<(), CollisionGenerationError> {
    if config.max_compound_parts == 0 {
        return Err(CollisionGenerationError::InvalidConfig(
            "max_compound_parts must be at least one",
        ));
    }
    if !config.minimum_extent.is_finite() || config.minimum_extent <= 0.0 {
        return Err(CollisionGenerationError::InvalidConfig(
            "minimum_extent must be finite and greater than zero",
        ));
    }
    if config.maximum_exact_partitions == 0 {
        return Err(CollisionGenerationError::InvalidConfig(
            "maximum_exact_partitions must be at least one",
        ));
    }
    if config.maximum_exact_triangles == 0 {
        return Err(CollisionGenerationError::InvalidConfig(
            "maximum_exact_triangles must be at least one",
        ));
    }
    Ok(())
}

fn validate_mesh(mesh: &TriangleMesh) -> Result<(), CollisionGenerationError> {
    for (index, point) in mesh.vertices.iter().enumerate() {
        if !point.iter().all(|value| value.is_finite()) {
            return Err(CollisionGenerationError::NonFiniteVertex { index });
        }
    }
    for (triangle_index, triangle) in mesh.triangles.iter().enumerate() {
        for &index in triangle {
            if index as usize >= mesh.vertices.len() {
                return Err(CollisionGenerationError::TriangleIndexOutOfBounds {
                    triangle: triangle_index,
                    index,
                });
            }
        }
    }
    Ok(())
}

fn usable_triangles(mesh: &TriangleMesh) -> Vec<[usize; 3]> {
    mesh.triangles
        .iter()
        .filter_map(|triangle| {
            let indices = triangle.map(|index| index as usize);
            let a = mesh.vertices[indices[0]];
            let b = mesh.vertices[indices[1]];
            let c = mesh.vertices[indices[2]];
            (length_squared(cross(sub(b, a), sub(c, a))) > 0.0).then_some(indices)
        })
        .collect()
}

fn box_from_points(points: &[Point3], minimum_extent: f32) -> BoxCandidate {
    let (mut min, mut max) = (points[0], points[0]);
    for point in &points[1..] {
        for axis in 0..3 {
            min[axis] = min[axis].min(point[axis]);
            max[axis] = max[axis].max(point[axis]);
        }
    }
    let mut size = sub(max, min);
    for axis in 0..3 {
        if size[axis] < minimum_extent {
            let expansion = (minimum_extent - size[axis]) * 0.5;
            min[axis] -= expansion;
            max[axis] += expansion;
            size[axis] = minimum_extent;
        }
    }
    BoxCandidate {
        center: scale(add(min, max), 0.5),
        size,
    }
}

fn sphere_from_points(points: &[Point3], center: Point3) -> SphereCandidate {
    let radius = points
        .iter()
        .map(|&point| length(sub(point, center)))
        .fold(0.0_f32, f32::max);
    SphereCandidate { center, radius }
}

fn cylinder_from_points(points: &[Point3], axis: Axis, minimum_extent: f32) -> CylinderCandidate {
    let index = axis.index();
    let mut minimum = points[0][index];
    let mut maximum = minimum;
    let mut radial_center_min = [f32::INFINITY; 3];
    let mut radial_center_max = [f32::NEG_INFINITY; 3];
    for point in points {
        minimum = minimum.min(point[index]);
        maximum = maximum.max(point[index]);
        for component in 0..3 {
            if component != index {
                radial_center_min[component] = radial_center_min[component].min(point[component]);
                radial_center_max[component] = radial_center_max[component].max(point[component]);
            }
        }
    }
    let mut center = [0.0; 3];
    center[index] = (minimum + maximum) * 0.5;
    for component in 0..3 {
        if component != index {
            center[component] = (radial_center_min[component] + radial_center_max[component]) * 0.5;
        }
    }
    let mut radial_squared: f32 = 0.0;
    for point in points {
        radial_squared = radial_squared.max(
            point
                .iter()
                .enumerate()
                .filter(|(component, _)| *component != index)
                .map(|(component, value)| (value - center[component]).powi(2))
                .sum(),
        );
    }
    CylinderCandidate {
        center,
        axis,
        radius: radial_squared.sqrt().max(minimum_extent * 0.5),
        height: (maximum - minimum).max(minimum_extent),
    }
}

fn capsule_from_points(points: &[Point3], axis: Axis) -> CapsuleCandidate {
    let index = axis.index();
    let mut minimum = points[0][index];
    let mut maximum = minimum;
    let mut radial_center_min = [f32::INFINITY; 3];
    let mut radial_center_max = [f32::NEG_INFINITY; 3];
    for point in points {
        minimum = minimum.min(point[index]);
        maximum = maximum.max(point[index]);
        for component in 0..3 {
            if component != index {
                radial_center_min[component] = radial_center_min[component].min(point[component]);
                radial_center_max[component] = radial_center_max[component].max(point[component]);
            }
        }
    }
    let mut a = [0.0; 3];
    let mut b = [0.0; 3];
    a[index] = minimum;
    b[index] = maximum;
    for component in 0..3 {
        if component != index {
            let center = (radial_center_min[component] + radial_center_max[component]) * 0.5;
            a[component] = center;
            b[component] = center;
        }
    }
    let mut radial_squared: f32 = 0.0;
    for point in points {
        radial_squared = radial_squared.max(
            point
                .iter()
                .enumerate()
                .filter(|(component, _)| *component != index)
                .map(|(component, value)| (value - a[component]).powi(2))
                .sum(),
        );
    }
    CapsuleCandidate {
        a,
        b,
        radius: radial_squared.sqrt(),
    }
}

fn partition_compound(
    mesh: &TriangleMesh,
    triangles: &[[usize; 3]],
    fallback_axis: Axis,
    config: CollisionGenerationConfig,
) -> CompoundCandidate {
    let source_bounds = bounds_for_triangles(mesh, triangles, config.minimum_extent);
    if triangles.len() > config.maximum_exact_triangles {
        return compound_bounds_fallback(
            mesh,
            triangles,
            source_bounds,
            fallback_axis,
            CompoundGenerationFallbackReason::TriangleLimit {
                source_triangle_count: triangles.len(),
                maximum_exact_triangles: config.maximum_exact_triangles,
            },
        );
    }

    let mut partitions = connected_triangle_partitions(mesh, triangles, config.minimum_extent);
    if partitions.len() > config.maximum_exact_partitions {
        return compound_bounds_fallback(
            mesh,
            triangles,
            source_bounds,
            fallback_axis,
            CompoundGenerationFallbackReason::PartitionLimit {
                connected_partition_count: partitions.len(),
                maximum_exact_partitions: config.maximum_exact_partitions,
            },
        );
    }

    merge_partitions_to_budget(
        mesh,
        &mut partitions,
        config.max_compound_parts,
        config.minimum_extent,
    );
    let mut first_split_axis = None;

    while partitions.len() < config.max_compound_parts {
        let next_split = partitions
            .iter()
            .enumerate()
            .filter_map(|(index, partition)| {
                best_partition_split(mesh, partition, config.minimum_extent)
                    .map(|split| (index, split))
            })
            .max_by(|(left_index, left), (right_index, right)| {
                compare_partition_splits(left, right).then_with(|| right_index.cmp(left_index))
            });
        let Some((partition_index, split)) = next_split else {
            break;
        };

        first_split_axis.get_or_insert(split.axis);
        partitions.swap_remove(partition_index);
        partitions.push(split.left);
        partitions.push(split.right);
        partitions.sort_by(|left, right| compare_boxes(&left.bounds, &right.bounds));
    }

    let parts = partitions
        .into_iter()
        .map(|partition| partition.bounds)
        .collect::<Vec<_>>();
    CompoundCandidate {
        partition_axis: first_split_axis.unwrap_or(fallback_axis),
        quality: compound_quality(
            mesh,
            triangles,
            &parts,
            source_bounds,
            CompoundGenerationPath::Exact,
        ),
        parts,
    }
}

fn compound_bounds_fallback(
    mesh: &TriangleMesh,
    triangles: &[[usize; 3]],
    source_bounds: BoxCandidate,
    fallback_axis: Axis,
    reason: CompoundGenerationFallbackReason,
) -> CompoundCandidate {
    let conservative_bounds = expand_box_for_roundoff(source_bounds);
    CompoundCandidate {
        partition_axis: fallback_axis,
        parts: vec![conservative_bounds],
        quality: compound_quality(
            mesh,
            triangles,
            &[conservative_bounds],
            source_bounds,
            CompoundGenerationPath::BoundsFallback { reason },
        ),
    }
}

/// Expands an AABB enough to cover arithmetic roundoff when its center/half-size
/// representation is reconstructed by a physics backend. This is used only by the
/// large-mesh bounds fallback; source vertices are never sampled or discarded.
fn expand_box_for_roundoff(mut candidate: BoxCandidate) -> BoxCandidate {
    for axis in 0..3 {
        let magnitude = (candidate.center[axis].abs() + candidate.size[axis]).max(1.0);
        let expansion = magnitude * f32::EPSILON * 8.0;
        candidate.size[axis] += expansion * 2.0;
    }
    candidate
}

fn compound_quality(
    mesh: &TriangleMesh,
    triangles: &[[usize; 3]],
    parts: &[BoxCandidate],
    source_bounds: BoxCandidate,
    generation_path: CompoundGenerationPath,
) -> CompoundCandidateQuality {
    let covered_triangle_count = triangles
        .iter()
        .filter(|triangle| {
            parts.iter().any(|part| {
                triangle
                    .iter()
                    .all(|&index| box_contains_point(*part, mesh.vertices[index]))
            })
        })
        .count();
    let conservative_volume = parts.iter().copied().map(box_volume).sum::<f32>();
    CompoundCandidateQuality {
        source_triangle_count: triangles.len(),
        covered_triangle_count,
        conservative_volume,
        bounds_volume_ratio: conservative_volume / box_volume(source_bounds),
        generation_path,
    }
}

fn connected_triangle_partitions(
    mesh: &TriangleMesh,
    triangles: &[[usize; 3]],
    minimum_extent: f32,
) -> Vec<TrianglePartition> {
    let mut sets = DisjointSets::new(triangles.len());
    let mut first_triangle_by_vertex = BTreeMap::new();
    for (triangle_index, triangle) in triangles.iter().enumerate() {
        for &vertex in triangle {
            if let Some(first) = first_triangle_by_vertex.insert(vertex, triangle_index) {
                sets.union(first, triangle_index);
            }
        }
    }

    let mut groups = BTreeMap::<usize, Vec<[usize; 3]>>::new();
    for (triangle_index, &triangle) in triangles.iter().enumerate() {
        groups
            .entry(sets.find(triangle_index))
            .or_default()
            .push(triangle);
    }
    let mut partitions = groups
        .into_values()
        .map(|triangles| TrianglePartition {
            bounds: bounds_for_triangles(mesh, &triangles, minimum_extent),
            triangles,
        })
        .collect::<Vec<_>>();
    partitions.sort_by(|left, right| compare_boxes(&left.bounds, &right.bounds));
    partitions
}

fn merge_partitions_to_budget(
    mesh: &TriangleMesh,
    partitions: &mut Vec<TrianglePartition>,
    maximum_parts: usize,
    minimum_extent: f32,
) {
    while partitions.len() > maximum_parts {
        let mut best: Option<(usize, usize, f32)> = None;
        for left in 0..partitions.len() {
            for right in left + 1..partitions.len() {
                let merged_bounds = merged_bounds(
                    mesh,
                    &partitions[left].triangles,
                    &partitions[right].triangles,
                    minimum_extent,
                );
                let cost = box_volume(merged_bounds)
                    - box_volume(partitions[left].bounds)
                    - box_volume(partitions[right].bounds);
                let is_better = best.is_none_or(|(best_left, best_right, best_cost)| {
                    cost.total_cmp(&best_cost)
                        .then_with(|| {
                            compare_boxes(&partitions[left].bounds, &partitions[best_left].bounds)
                        })
                        .then_with(|| {
                            compare_boxes(&partitions[right].bounds, &partitions[best_right].bounds)
                        })
                        .is_lt()
                });
                if is_better {
                    best = Some((left, right, cost));
                }
            }
        }
        let Some((left, right, _)) = best else {
            return;
        };
        let mut merged_triangles = partitions[left].triangles.clone();
        merged_triangles.extend_from_slice(&partitions[right].triangles);
        let merged = TrianglePartition {
            bounds: bounds_for_triangles(mesh, &merged_triangles, minimum_extent),
            triangles: merged_triangles,
        };
        partitions.swap_remove(right);
        partitions.swap_remove(left);
        partitions.push(merged);
        partitions.sort_by(|left, right| compare_boxes(&left.bounds, &right.bounds));
    }
}

fn merged_bounds(
    mesh: &TriangleMesh,
    left: &[[usize; 3]],
    right: &[[usize; 3]],
    minimum_extent: f32,
) -> BoxCandidate {
    let mut triangles = Vec::with_capacity(left.len() + right.len());
    triangles.extend_from_slice(left);
    triangles.extend_from_slice(right);
    bounds_for_triangles(mesh, &triangles, minimum_extent)
}

#[derive(Debug)]
struct DisjointSets {
    parents: Vec<usize>,
}

impl DisjointSets {
    fn new(length: usize) -> Self {
        Self {
            parents: (0..length).collect(),
        }
    }

    fn find(&mut self, item: usize) -> usize {
        if self.parents[item] != item {
            self.parents[item] = self.find(self.parents[item]);
        }
        self.parents[item]
    }

    fn union(&mut self, left: usize, right: usize) {
        let left_root = self.find(left);
        let right_root = self.find(right);
        if left_root != right_root {
            self.parents[right_root] = left_root;
        }
    }
}

#[derive(Clone, Debug)]
struct TrianglePartition {
    triangles: Vec<[usize; 3]>,
    bounds: BoxCandidate,
}

#[derive(Clone, Debug)]
struct PartitionSplit {
    axis: Axis,
    reduction: f32,
    left: TrianglePartition,
    right: TrianglePartition,
}

fn best_partition_split(
    mesh: &TriangleMesh,
    partition: &TrianglePartition,
    minimum_extent: f32,
) -> Option<PartitionSplit> {
    if partition.triangles.len() < 2 {
        return None;
    }

    let parent_volume = box_volume(partition.bounds);
    let mut best = None;
    for axis in [Axis::X, Axis::Y, Axis::Z] {
        let mut sorted = partition.triangles.clone();
        sorted.sort_by(|left, right| compare_triangles_on_axis(mesh, *left, *right, axis));
        for split_index in 1..sorted.len() {
            let left_triangles = sorted[..split_index].to_vec();
            let right_triangles = sorted[split_index..].to_vec();
            let left = TrianglePartition {
                bounds: bounds_for_triangles(mesh, &left_triangles, minimum_extent),
                triangles: left_triangles,
            };
            let right = TrianglePartition {
                bounds: bounds_for_triangles(mesh, &right_triangles, minimum_extent),
                triangles: right_triangles,
            };
            let reduction = parent_volume - box_volume(left.bounds) - box_volume(right.bounds);
            if reduction <= 0.0 {
                continue;
            }
            let candidate = PartitionSplit {
                axis,
                reduction,
                left,
                right,
            };
            if best
                .as_ref()
                .is_none_or(|current| compare_partition_splits(&candidate, current).is_gt())
            {
                best = Some(candidate);
            }
        }
    }
    best
}

fn compare_partition_splits(left: &PartitionSplit, right: &PartitionSplit) -> std::cmp::Ordering {
    left.reduction
        .total_cmp(&right.reduction)
        .then_with(|| right.axis.index().cmp(&left.axis.index()))
        .then_with(|| compare_boxes(&right.left.bounds, &left.left.bounds))
        .then_with(|| compare_boxes(&right.right.bounds, &left.right.bounds))
}

fn bounds_for_triangles(
    mesh: &TriangleMesh,
    triangles: &[[usize; 3]],
    minimum_extent: f32,
) -> BoxCandidate {
    let points = triangles
        .iter()
        .flat_map(|triangle| triangle.iter().map(|&index| mesh.vertices[index]))
        .collect::<Vec<_>>();
    box_from_points(&points, minimum_extent)
}

fn compare_triangles_on_axis(
    mesh: &TriangleMesh,
    left: [usize; 3],
    right: [usize; 3],
    axis: Axis,
) -> std::cmp::Ordering {
    triangle_centroid(mesh, left)[axis.index()]
        .total_cmp(&triangle_centroid(mesh, right)[axis.index()])
        .then_with(|| compare_triangle_geometry(mesh, left, right))
}

fn triangle_centroid(mesh: &TriangleMesh, triangle: [usize; 3]) -> Point3 {
    scale(
        triangle
            .iter()
            .map(|&index| mesh.vertices[index])
            .fold([0.0; 3], add),
        1.0 / 3.0,
    )
}

fn compare_triangle_geometry(
    mesh: &TriangleMesh,
    left: [usize; 3],
    right: [usize; 3],
) -> std::cmp::Ordering {
    let mut left_points = left.map(|index| canonical_point(mesh.vertices[index]));
    let mut right_points = right.map(|index| canonical_point(mesh.vertices[index]));
    left_points.sort_by(compare_points);
    right_points.sort_by(compare_points);
    left_points
        .iter()
        .zip(right_points)
        .map(|(left, right)| compare_points(left, &right))
        .find(|comparison| !comparison.is_eq())
        .unwrap_or(std::cmp::Ordering::Equal)
}

fn box_volume(candidate: BoxCandidate) -> f32 {
    candidate.size.iter().product()
}

fn box_contains_point(candidate: BoxCandidate, point: Point3) -> bool {
    (0..3).all(|axis| (point[axis] - candidate.center[axis]).abs() <= candidate.size[axis] * 0.5)
}

fn unique_sorted_points(points: &[Point3]) -> Vec<Point3> {
    let mut points = points
        .iter()
        .copied()
        .map(canonical_point)
        .collect::<Vec<_>>();
    points.sort_by(compare_points);
    points.dedup();
    points
}

fn canonical_point(mut point: Point3) -> Point3 {
    for value in &mut point {
        if *value == 0.0 {
            *value = 0.0;
        }
    }
    point
}

fn compare_points(a: &Point3, b: &Point3) -> std::cmp::Ordering {
    a[0].total_cmp(&b[0])
        .then_with(|| a[1].total_cmp(&b[1]))
        .then_with(|| a[2].total_cmp(&b[2]))
}

fn compare_boxes(a: &BoxCandidate, b: &BoxCandidate) -> std::cmp::Ordering {
    compare_points(&a.center, &b.center).then_with(|| compare_points(&a.size, &b.size))
}

fn longest_axis(size: Point3) -> Axis {
    if size[0] >= size[1] && size[0] >= size[2] {
        Axis::X
    } else if size[1] >= size[2] {
        Axis::Y
    } else {
        Axis::Z
    }
}

fn add(a: Point3, b: Point3) -> Point3 {
    [a[0] + b[0], a[1] + b[1], a[2] + b[2]]
}

fn sub(a: Point3, b: Point3) -> Point3 {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}

fn scale(point: Point3, scalar: f32) -> Point3 {
    [point[0] * scalar, point[1] * scalar, point[2] * scalar]
}

fn cross(a: Point3, b: Point3) -> Point3 {
    [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]
}

fn length_squared(point: Point3) -> f32 {
    point.iter().map(|value| value * value).sum()
}

fn length(point: Point3) -> f32 {
    length_squared(point).sqrt()
}

#[cfg(test)]
mod tests {
    use super::*;

    fn tetrahedron() -> TriangleMesh {
        TriangleMesh {
            vertices: vec![
                [0.0, 0.0, 0.0],
                [2.0, 0.0, 0.0],
                [0.0, 1.0, 0.0],
                [0.0, 0.0, 1.0],
            ],
            triangles: vec![[0, 2, 1], [0, 1, 3], [0, 3, 2], [1, 2, 3]],
        }
    }

    #[test]
    fn emits_conservative_primitive_compound_and_hull_options() {
        let candidates = generate_collision_candidates(
            &tetrahedron(),
            CollisionGenerationConfig {
                max_compound_parts: 2,
                minimum_extent: 0.01,
                ..CollisionGenerationConfig::default()
            },
        )
        .expect("valid tetrahedron");

        assert_eq!(
            candidates.bounds,
            BoxCandidate {
                center: [1.0, 0.5, 0.5],
                size: [2.0, 1.0, 1.0],
            }
        );
        assert!(matches!(
            candidates.primitives[0],
            PrimitiveCandidate::Box(_)
        ));
        assert_eq!(candidates.primitives.len(), 8);
        assert_eq!(candidates.compounds[0].partition_axis, Axis::X);
        assert_eq!(candidates.compounds[0].parts.len(), 1);
        assert_eq!(candidates.compounds[0].quality.source_triangle_count, 4);
        assert_eq!(candidates.compounds[0].quality.covered_triangle_count, 4);
        assert_eq!(candidates.compounds[0].quality.bounds_volume_ratio, 1.0);
        assert_eq!(
            candidates.convex_hull.points,
            vec![
                [0.0, 0.0, 0.0],
                [0.0, 0.0, 1.0],
                [0.0, 1.0, 0.0],
                [2.0, 0.0, 0.0],
            ]
        );
    }

    #[test]
    fn generated_hull_and_compound_are_deterministic_despite_mesh_order() {
        let mesh = tetrahedron();
        let mut reordered = mesh.clone();
        reordered.vertices.reverse();
        reordered.triangles = mesh
            .triangles
            .iter()
            .rev()
            .map(|triangle| triangle.map(|index| 3 - index))
            .collect();

        let config = CollisionGenerationConfig {
            max_compound_parts: 3,
            minimum_extent: 0.01,
            ..CollisionGenerationConfig::default()
        };
        assert_eq!(
            generate_collision_candidates(&mesh, config),
            generate_collision_candidates(&reordered, config)
        );
    }

    #[test]
    fn ignores_degenerate_triangles_but_rejects_a_mesh_without_any_area() {
        let mut mesh = tetrahedron();
        mesh.triangles.push([0, 0, 1]);
        assert!(generate_collision_candidates(&mesh, CollisionGenerationConfig::default()).is_ok());

        let flat_mesh = TriangleMesh {
            vertices: vec![[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [2.0, 0.0, 0.0]],
            triangles: vec![[0, 1, 2]],
        };
        assert_eq!(
            generate_collision_candidates(&flat_mesh, CollisionGenerationConfig::default()),
            Err(CollisionGenerationError::NoUsableTriangles)
        );
    }

    #[test]
    fn rejects_bad_input_without_panicking() {
        let invalid_index = TriangleMesh {
            vertices: vec![[0.0, 0.0, 0.0]],
            triangles: vec![[0, 1, 2]],
        };
        assert_eq!(
            generate_collision_candidates(&invalid_index, CollisionGenerationConfig::default()),
            Err(CollisionGenerationError::TriangleIndexOutOfBounds {
                triangle: 0,
                index: 1
            })
        );

        let invalid_config = CollisionGenerationConfig {
            max_compound_parts: 0,
            ..CollisionGenerationConfig::default()
        };
        assert!(matches!(
            generate_collision_candidates(&tetrahedron(), invalid_config),
            Err(CollisionGenerationError::InvalidConfig(_))
        ));
    }

    #[test]
    fn normalizes_negative_zero_when_deduplicating_hull_points() {
        let mesh = TriangleMesh {
            vertices: vec![
                [-0.0, 0.0, 0.0],
                [1.0, 0.0, 0.0],
                [0.0, 1.0, 0.0],
                [0.0, 0.0, 1.0],
            ],
            triangles: vec![[0, 1, 2], [0, 2, 3]],
        };
        let candidates = generate_collision_candidates(&mesh, CollisionGenerationConfig::default())
            .expect("valid mesh");
        assert_eq!(candidates.convex_hull.points[0], [0.0, 0.0, 0.0]);
    }

    #[test]
    fn every_primitive_and_the_compound_cover_the_source_mesh() {
        let mesh = TriangleMesh {
            vertices: vec![
                [10.0, -2.0, 4.0],
                [12.0, -2.0, 4.0],
                [10.0, -1.0, 4.0],
                [10.0, -2.0, 5.0],
            ],
            triangles: vec![[0, 2, 1], [0, 1, 3], [0, 3, 2], [1, 2, 3]],
        };
        let candidates = generate_collision_candidates(&mesh, CollisionGenerationConfig::default())
            .expect("valid tetrahedron");

        for primitive in &candidates.primitives {
            for &point in &mesh.vertices {
                assert!(
                    primitive_contains(*primitive, point),
                    "{primitive:?} misses {point:?}"
                );
            }
        }
        for &point in &mesh.vertices {
            assert!(candidates.compounds[0]
                .parts
                .iter()
                .any(|part| box_contains(*part, point)));
        }
    }

    #[test]
    fn preserves_valid_sub_millimetre_triangles() {
        let mesh = TriangleMesh {
            vertices: vec![
                [0.0, 0.0, 0.0],
                [0.000_001, 0.0, 0.0],
                [0.0, 0.000_001, 0.0],
            ],
            triangles: vec![[0, 1, 2]],
        };
        assert!(generate_collision_candidates(&mesh, CollisionGenerationConfig::default()).is_ok());
    }

    #[test]
    fn decomposes_a_concave_u_shape_into_tighter_compound_boxes() {
        let mesh = u_shape_mesh();
        let candidates = generate_collision_candidates(
            &mesh,
            CollisionGenerationConfig {
                max_compound_parts: 3,
                minimum_extent: 0.001,
                ..CollisionGenerationConfig::default()
            },
        )
        .expect("valid concave mesh");
        let compound = &candidates.compounds[0];
        let compound_volume = compound.parts.iter().copied().map(box_volume).sum::<f32>();

        assert_eq!(compound.parts.len(), 3);
        assert!(
            compound_volume < box_volume(candidates.bounds) * 0.75,
            "compound {compound_volume} did not materially improve bounds {}",
            box_volume(candidates.bounds)
        );
        assert_eq!(compound.quality.source_triangle_count, mesh.triangles.len());
        assert_eq!(
            compound.quality.covered_triangle_count,
            mesh.triangles.len()
        );
        assert_eq!(compound.quality.conservative_volume, compound_volume);
        assert!(compound.quality.bounds_volume_ratio < 0.75);
        for &point in &mesh.vertices {
            assert!(compound.parts.iter().any(|part| box_contains(*part, point)));
        }
    }

    #[test]
    fn disconnected_large_mesh_uses_an_explicit_conservative_partition_fallback() {
        let mesh = disconnected_triangle_mesh(
            CollisionGenerationConfig::default().maximum_exact_partitions + 1,
        );
        let candidates = generate_collision_candidates(&mesh, CollisionGenerationConfig::default())
            .expect("valid disconnected mesh");
        let compound = &candidates.compounds[0];

        assert!(compound.parts[0]
            .size
            .iter()
            .zip(candidates.bounds.size)
            .all(|(fallback, bounds)| fallback > &bounds));
        assert_eq!(compound.quality.source_triangle_count, mesh.triangles.len());
        assert_eq!(
            compound.quality.covered_triangle_count,
            mesh.triangles.len()
        );
        assert!(compound.quality.bounds_volume_ratio > 1.0);
        assert_eq!(
            compound.quality.generation_path,
            CompoundGenerationPath::BoundsFallback {
                reason: CompoundGenerationFallbackReason::PartitionLimit {
                    connected_partition_count: mesh.triangles.len(),
                    maximum_exact_partitions: CollisionGenerationConfig::default()
                        .maximum_exact_partitions,
                },
            }
        );
    }

    #[test]
    fn puppybot_esp32_asset_generates_with_full_coverage_and_bounded_evidence() {
        let path = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("../../PuppyBot/models/puppybot/final2/meshes/esp32_Wroom_30pins_C_Type_73.gltf");
        let asset = load_gltf_triangle_mesh(&path).expect("load PuppyBot ESP32 asset");
        let candidates =
            generate_collision_candidates(&asset.mesh, CollisionGenerationConfig::default())
                .expect("generate ESP32 candidates");
        let compound = &candidates.compounds[0];
        let usable_triangle_count = usable_triangles(&asset.mesh).len();

        assert_eq!(
            compound.quality.source_triangle_count,
            usable_triangle_count
        );
        assert_eq!(
            compound.quality.covered_triangle_count,
            compound.quality.source_triangle_count
        );
        assert!(matches!(
            compound.quality.generation_path,
            CompoundGenerationPath::BoundsFallback {
                reason: CompoundGenerationFallbackReason::PartitionLimit { .. }
                    | CompoundGenerationFallbackReason::TriangleLimit { .. },
            }
        ));
        let selection = candidates
            .select_compound_profile(4)
            .expect("select conservative ESP32 compound");
        assert_eq!(
            selection.evidence.compound_generation_path,
            compound.quality.generation_path
        );
    }

    #[test]
    fn loads_a_real_gltf_fixture_with_parent_node_transforms_and_provenance() {
        let fixture = write_gltf_fixture();
        let loaded = load_gltf_triangle_mesh(&fixture.path).expect("load fixture");

        assert_eq!(
            loaded.mesh.vertices,
            [[1.0, 3.0, 3.0], [3.0, 3.0, 3.0], [1.0, 4.0, 3.0]]
        );
        assert_eq!(loaded.mesh.triangles, vec![[0, 1, 2]]);
        assert_eq!(loaded.provenance.format, AssetFormat::Gltf);
        assert_eq!(loaded.provenance.scene_index, 0);
        assert_eq!(loaded.provenance.node_instance_count, 2);
        assert_eq!(loaded.provenance.primitive_count, 1);
        assert_eq!(loaded.provenance.source_vertex_count, 3);
        assert_eq!(loaded.provenance.triangle_count, 1);
        assert!(loaded.provenance.node_transforms_applied);

        let generated = generate_collision_candidates_from_gltf(
            &fixture.path,
            CollisionGenerationConfig::default(),
        )
        .expect("generate from asset");
        let selected = generated
            .candidates
            .select_compound_profile(1)
            .expect("select generated compound");
        assert_eq!(selected.evidence.source_triangle_count, 1);
        assert_eq!(selected.evidence.covered_triangle_count, 1);
        fixture.remove();
    }

    #[test]
    fn loads_a_real_glb_fixture_with_provenance() {
        let fixture = write_glb_fixture();
        let loaded = load_gltf_triangle_mesh(&fixture.path).expect("load fixture");

        assert_eq!(
            loaded.mesh.vertices,
            [[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]]
        );
        assert_eq!(loaded.mesh.triangles, vec![[0, 1, 2]]);
        assert_eq!(loaded.provenance.format, AssetFormat::Glb);
        assert_eq!(loaded.provenance.node_instance_count, 1);
        assert_eq!(loaded.provenance.primitive_count, 1);
        assert_eq!(loaded.provenance.source_vertex_count, 3);
        fixture.remove();
    }

    #[test]
    fn batch_generation_applies_link_placement_and_preserves_later_failures() {
        let fixture = write_gltf_fixture();
        let missing = fixture.directory.join("does-not-exist.gltf");
        let requests = vec![
            LinkAssetColliderRequest {
                link_name: "base".to_owned(),
                asset_id: "chassis".to_owned(),
                asset_path: fixture.path.clone(),
                asset_to_link: LinkAssetTransform {
                    translation_m: [10.0, 0.0, 0.0],
                    ..LinkAssetTransform::default()
                },
            },
            LinkAssetColliderRequest {
                link_name: "missing".to_owned(),
                asset_id: "missing-mesh".to_owned(),
                asset_path: missing,
                asset_to_link: LinkAssetTransform::default(),
            },
            LinkAssetColliderRequest {
                link_name: "arm".to_owned(),
                asset_id: "arm-shell".to_owned(),
                asset_path: fixture.path.clone(),
                asset_to_link: LinkAssetTransform {
                    translation_m: [0.0, 1.0, 0.0],
                    rotation_rpy_rad: [0.0, 0.0, std::f32::consts::FRAC_PI_2],
                    scale: [2.0, 1.0, 1.0],
                },
            },
            LinkAssetColliderRequest {
                link_name: "invalid-placement".to_owned(),
                asset_id: "invalid-transform".to_owned(),
                asset_path: fixture.path.clone(),
                asset_to_link: LinkAssetTransform {
                    scale: [1.0, 0.0, 1.0],
                    ..LinkAssetTransform::default()
                },
            },
        ];
        let report = generate_link_asset_collision_candidates(
            &requests,
            CollisionGenerationConfig::default(),
        );

        assert_eq!(report.generated().count(), 2);
        assert_eq!(report.failures().count(), 2);
        assert_eq!(report.generated_links().count(), 2);
        assert_eq!(report.link_failures().count(), 2);
        assert!(matches!(
            &report.results[0],
            LinkAssetCollisionResult::Generated(generated)
                if generated.link_name == "base" && generated.candidates.bounds.center == [12.0, 3.5, 3.0]
        ));
        assert!(matches!(
            &report.results[1],
            LinkAssetCollisionResult::Failed(LinkAssetCollisionFailure {
                reason: LinkAssetCollisionFailureReason::AssetLoad(_),
                ..
            })
        ));
        assert!(matches!(
            &report.results[2],
            LinkAssetCollisionResult::Generated(generated)
                if generated.link_name == "arm" && generated.candidates.bounds.center == [-3.5, 5.0, 3.0]
        ));
        assert!(matches!(
            &report.results[3],
            LinkAssetCollisionResult::Failed(LinkAssetCollisionFailure {
                reason: LinkAssetCollisionFailureReason::InvalidAssetTransform,
                ..
            })
        ));
        assert_eq!(
            report,
            generate_link_asset_collision_candidates(
                &requests,
                CollisionGenerationConfig::default()
            )
        );
        fixture.remove();
    }

    #[test]
    fn batch_generation_merges_multiple_assets_for_one_physical_link() {
        let gltf = write_gltf_fixture();
        let glb = write_glb_fixture();
        let requests = vec![
            LinkAssetColliderRequest {
                link_name: "root".to_owned(),
                asset_id: "main-shell".to_owned(),
                asset_path: gltf.path.clone(),
                asset_to_link: LinkAssetTransform::default(),
            },
            LinkAssetColliderRequest {
                link_name: "root".to_owned(),
                asset_id: "sensor-cover".to_owned(),
                asset_path: glb.path.clone(),
                asset_to_link: LinkAssetTransform {
                    translation_m: [10.0, 0.0, 0.0],
                    ..LinkAssetTransform::default()
                },
            },
        ];
        let report = generate_link_asset_collision_candidates(
            &requests,
            CollisionGenerationConfig::default(),
        );

        assert_eq!(report.generated().count(), 2);
        assert!(report.failures().next().is_none());
        assert_eq!(report.generated_links().count(), 1);
        assert!(report.link_failures().next().is_none());
        assert!(matches!(
            &report.link_results[0],
            LinkCollisionAggregateResult::Generated(generated)
                if generated.link_name == "root"
                    && generated.source_assets.iter().map(|asset| asset.asset_id.as_str()).collect::<Vec<_>>() == ["main-shell", "sensor-cover"]
                    && generated.candidates.bounds.center == [6.0, 2.0, 1.5]
                    && generated.candidates.compounds[0].quality.source_triangle_count == 2
        ));
        assert_eq!(
            report,
            generate_link_asset_collision_candidates(
                &requests,
                CollisionGenerationConfig::default()
            )
        );
        gltf.remove();
        glb.remove();
    }

    #[test]
    fn batch_generation_reports_duplicate_asset_ids_and_empty_link_names_explicitly() {
        let requests = vec![
            LinkAssetColliderRequest {
                link_name: "hip".to_owned(),
                asset_id: "shell".to_owned(),
                asset_path: "unread.gltf".into(),
                asset_to_link: LinkAssetTransform::default(),
            },
            LinkAssetColliderRequest {
                link_name: "hip".to_owned(),
                asset_id: "shell".to_owned(),
                asset_path: "also-unread.gltf".into(),
                asset_to_link: LinkAssetTransform::default(),
            },
            LinkAssetColliderRequest {
                link_name: "  ".to_owned(),
                asset_id: "empty-link".to_owned(),
                asset_path: "empty-name.gltf".into(),
                asset_to_link: LinkAssetTransform::default(),
            },
            LinkAssetColliderRequest {
                link_name: "knee".to_owned(),
                asset_id: " ".to_owned(),
                asset_path: "empty-id.gltf".into(),
                asset_to_link: LinkAssetTransform::default(),
            },
        ];
        let report = generate_link_asset_collision_candidates(
            &requests,
            CollisionGenerationConfig::default(),
        );

        assert!(report.generated().next().is_none());
        assert!(matches!(
            &report.results[0],
            LinkAssetCollisionResult::Failed(LinkAssetCollisionFailure {
                reason: LinkAssetCollisionFailureReason::DuplicateAssetId,
                ..
            })
        ));
        assert!(matches!(
            &report.results[1],
            LinkAssetCollisionResult::Failed(LinkAssetCollisionFailure {
                reason: LinkAssetCollisionFailureReason::DuplicateAssetId,
                ..
            })
        ));
        assert!(matches!(
            &report.results[2],
            LinkAssetCollisionResult::Failed(LinkAssetCollisionFailure {
                reason: LinkAssetCollisionFailureReason::EmptyLinkName,
                ..
            })
        ));
        assert!(matches!(
            &report.results[3],
            LinkAssetCollisionResult::Failed(LinkAssetCollisionFailure {
                reason: LinkAssetCollisionFailureReason::EmptyAssetId,
                ..
            })
        ));
        assert_eq!(report.link_results.len(), 3);
        assert!(matches!(
            &report.link_results[0],
            LinkCollisionAggregateResult::Failed(LinkCollisionAggregateFailure {
                link_name,
                asset_count: 2,
                generated_asset_count: 0,
                reason: LinkCollisionAggregateFailureReason::AssetFailures { failed_asset_ids },
            }) if link_name == "hip" && failed_asset_ids.iter().map(String::as_str).eq(["shell", "shell"])
        ));
    }

    #[test]
    fn rejects_non_gltf_asset_extensions_before_importing() {
        let path = std::env::temp_dir().join("pge-collision-not-a-gltf.obj");
        assert_eq!(
            load_gltf_triangle_mesh(&path),
            Err(AssetMeshLoadError::UnsupportedFormat { path })
        );
    }

    #[test]
    fn exports_robotdreams_reviewed_profile_with_deterministic_budget_fallback() {
        let candidates = CollisionCandidates {
            bounds: BoxCandidate {
                center: [0.0, 0.0, 0.05],
                size: [0.24, 0.18, 0.10],
            },
            primitives: Vec::new(),
            compounds: vec![CompoundCandidate {
                partition_axis: Axis::X,
                quality: CompoundCandidateQuality {
                    source_triangle_count: 2,
                    covered_triangle_count: 2,
                    conservative_volume: 0.00252,
                    bounds_volume_ratio: 0.583_333_3,
                    generation_path: CompoundGenerationPath::Exact,
                },
                parts: vec![
                    BoxCandidate {
                        center: [-0.04, 0.0, 0.05],
                        size: [0.08, 0.18, 0.10],
                    },
                    BoxCandidate {
                        center: [0.06, 0.0, 0.05],
                        size: [0.06, 0.18, 0.10],
                    },
                ],
            }],
            convex_hull: ConvexHullCandidate { points: Vec::new() },
        };

        let profile = candidates
            .export_reviewed_profile(ReviewedProfileExportConfig::default())
            .expect("compound fits the default budget");
        let json = serde_json::from_str::<serde_json::Value>(
            &profile.to_json_pretty().expect("serializable profile"),
        )
        .expect("valid profile JSON");
        assert_eq!(
            json,
            serde_json::json!({
                "colliders": [
                    {
                        "shape": "box",
                        "size": [0.08, 0.18, 0.10],
                        "offset": [-0.04, 0.0, 0.05],
                        "rotation": [0.0, 0.0, 0.0]
                    },
                    {
                        "shape": "box",
                        "size": [0.06, 0.18, 0.10],
                        "offset": [0.06, 0.0, 0.05],
                        "rotation": [0.0, 0.0, 0.0]
                    }
                ]
            })
        );

        let selected = candidates
            .select_compound_profile(4)
            .expect("compound selection");
        assert_eq!(
            selected.evidence.decision,
            CompoundSelectionDecision::Compound { candidate_index: 0 }
        );
        assert_eq!(selected.evidence.source_triangle_count, 2);
        assert_eq!(selected.evidence.covered_triangle_count, 2);
        assert_eq!(selected.evidence.collider_count, 2);
        assert_eq!(selected.evidence.bounds_volume_ratio, 0.583_333_3);

        let fallback = candidates
            .export_reviewed_profile(ReviewedProfileExportConfig {
                maximum_colliders: 1,
                ..ReviewedProfileExportConfig::default()
            })
            .expect("budget fallback remains conservative");
        assert_eq!(fallback.colliders, vec![profile_box(candidates.bounds)]);
        let fallback_selection = candidates
            .select_compound_profile(1)
            .expect("budget fallback selection");
        assert_eq!(
            fallback_selection.evidence.decision,
            CompoundSelectionDecision::BoundsFallback {
                reason: CompoundFallbackReason::ExceedsColliderBudget
            }
        );
        assert_eq!(fallback_selection.evidence.bounds_volume_ratio, 1.0);
    }

    #[test]
    fn exports_cardinal_cylinders_with_robotdreams_rpy_rotation() {
        let candidates =
            generate_collision_candidates(&tetrahedron(), CollisionGenerationConfig::default())
                .expect("valid tetrahedron");
        let profile = candidates
            .export_reviewed_profile(ReviewedProfileExportConfig {
                selection: ReviewedProfileSelection::Cylinder { axis: Axis::Z },
                maximum_colliders: 1,
            })
            .expect("generated z cylinder");
        assert!(matches!(
            profile.colliders.as_slice(),
            [ReviewedCollider::Cylinder {
                rotation,
                ..
            }] if *rotation == [std::f32::consts::FRAC_PI_2, 0.0, 0.0]
        ));
        assert_eq!(
            candidates.export_reviewed_profile(ReviewedProfileExportConfig {
                maximum_colliders: 0,
                ..ReviewedProfileExportConfig::default()
            }),
            Err(ReviewedProfileExportError::InvalidColliderBudget)
        );
    }

    fn primitive_contains(primitive: PrimitiveCandidate, point: Point3) -> bool {
        const EPSILON: f32 = 0.000_01;
        match primitive {
            PrimitiveCandidate::Box(candidate) => box_contains(candidate, point),
            PrimitiveCandidate::Sphere(candidate) => {
                length(sub(point, candidate.center)) <= candidate.radius + EPSILON
            }
            PrimitiveCandidate::Cylinder(candidate) => {
                let axis = candidate.axis.index();
                let axial = (point[axis] - candidate.center[axis]).abs();
                let radial_squared = point
                    .iter()
                    .enumerate()
                    .filter(|(component, _)| *component != axis)
                    .map(|(component, value)| (value - candidate.center[component]).powi(2))
                    .sum::<f32>();
                axial <= candidate.height * 0.5 + EPSILON
                    && radial_squared.sqrt() <= candidate.radius + EPSILON
            }
            PrimitiveCandidate::Capsule(candidate) => {
                let segment = sub(candidate.b, candidate.a);
                let length_squared = length_squared(segment);
                let fraction = if length_squared > 0.0 {
                    (dot(sub(point, candidate.a), segment) / length_squared).clamp(0.0, 1.0)
                } else {
                    0.0
                };
                length(sub(point, add(candidate.a, scale(segment, fraction))))
                    <= candidate.radius + EPSILON
            }
        }
    }

    fn box_contains(candidate: BoxCandidate, point: Point3) -> bool {
        const EPSILON: f32 = 0.000_01;
        (0..3).all(|axis| {
            (point[axis] - candidate.center[axis]).abs() <= candidate.size[axis] * 0.5 + EPSILON
        })
    }

    fn dot(a: Point3, b: Point3) -> f32 {
        a.iter().zip(b).map(|(a, b)| a * b).sum()
    }

    fn u_shape_mesh() -> TriangleMesh {
        let mut mesh = TriangleMesh {
            vertices: Vec::new(),
            triangles: Vec::new(),
        };
        append_box(&mut mesh, [-1.0, 0.0, 0.0], [0.5, 3.0, 1.0]);
        append_box(&mut mesh, [1.0, 0.0, 0.0], [0.5, 3.0, 1.0]);
        append_box(&mut mesh, [0.0, -1.25, 0.0], [2.5, 0.5, 1.0]);
        mesh
    }

    fn disconnected_triangle_mesh(count: usize) -> TriangleMesh {
        let mut mesh = TriangleMesh {
            vertices: Vec::with_capacity(count * 3),
            triangles: Vec::with_capacity(count),
        };
        for index in 0..count {
            let x = index as f32 * 2.0;
            let base = mesh.vertices.len() as u32;
            mesh.vertices
                .extend([[x, 0.0, 0.0], [x + 1.0, 0.0, 0.0], [x, 1.0, 0.0]]);
            mesh.triangles.push([base, base + 1, base + 2]);
        }
        mesh
    }

    fn append_box(mesh: &mut TriangleMesh, center: Point3, size: Point3) {
        let base = mesh.vertices.len() as u32;
        let half = scale(size, 0.5);
        for x in [-half[0], half[0]] {
            for y in [-half[1], half[1]] {
                for z in [-half[2], half[2]] {
                    mesh.vertices.push(add(center, [x, y, z]));
                }
            }
        }
        for face in [
            [0, 1, 3, 2],
            [4, 6, 7, 5],
            [0, 4, 5, 1],
            [2, 3, 7, 6],
            [0, 2, 6, 4],
            [1, 5, 7, 3],
        ] {
            mesh.triangles
                .push([base + face[0], base + face[1], base + face[2]]);
            mesh.triangles
                .push([base + face[0], base + face[2], base + face[3]]);
        }
    }

    struct GltfFixture {
        directory: std::path::PathBuf,
        path: std::path::PathBuf,
    }

    impl GltfFixture {
        fn remove(self) {
            std::fs::remove_dir_all(self.directory).expect("remove fixture directory");
        }
    }

    fn write_gltf_fixture() -> GltfFixture {
        let directory = fixture_directory("gltf");
        let _ = std::fs::remove_dir_all(&directory);
        std::fs::create_dir_all(&directory).expect("create fixture directory");

        let mut bytes = Vec::new();
        for value in [0.0_f32, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0] {
            bytes.extend_from_slice(&value.to_le_bytes());
        }
        for value in [0_u16, 1, 2] {
            bytes.extend_from_slice(&value.to_le_bytes());
        }
        std::fs::write(directory.join("triangle.bin"), bytes).expect("write fixture buffer");
        let path = directory.join("transformed-triangle.gltf");
        std::fs::write(
            &path,
            r#"{
                "asset": {"version": "2.0"},
                "buffers": [{"uri": "triangle.bin", "byteLength": 42}],
                "bufferViews": [
                    {"buffer": 0, "byteOffset": 0, "byteLength": 36},
                    {"buffer": 0, "byteOffset": 36, "byteLength": 6}
                ],
                "accessors": [
                    {"bufferView": 0, "componentType": 5126, "count": 3, "type": "VEC3", "min": [0, 0, 0], "max": [1, 1, 0]},
                    {"bufferView": 1, "componentType": 5123, "count": 3, "type": "SCALAR"}
                ],
                "meshes": [{"primitives": [{"attributes": {"POSITION": 0}, "indices": 1}]}],
                "nodes": [
                    {"translation": [1, 2, 3], "scale": [2, 1, 1], "children": [1]},
                    {"translation": [0, 1, 0], "mesh": 0}
                ],
                "scenes": [{"nodes": [0]}],
                "scene": 0
            }"#,
        )
        .expect("write fixture gltf");
        GltfFixture { directory, path }
    }

    fn write_glb_fixture() -> GltfFixture {
        let directory = fixture_directory("glb");
        let _ = std::fs::remove_dir_all(&directory);
        std::fs::create_dir_all(&directory).expect("create fixture directory");

        let mut binary = Vec::new();
        for value in [0.0_f32, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0] {
            binary.extend_from_slice(&value.to_le_bytes());
        }
        for value in [0_u16, 1, 2] {
            binary.extend_from_slice(&value.to_le_bytes());
        }
        let json = br#"{
            "asset":{"version":"2.0"},
            "buffers":[{"byteLength":42}],
            "bufferViews":[
                {"buffer":0,"byteOffset":0,"byteLength":36},
                {"buffer":0,"byteOffset":36,"byteLength":6}
            ],
            "accessors":[
                {"bufferView":0,"componentType":5126,"count":3,"type":"VEC3","min":[0,0,0],"max":[1,1,0]},
                {"bufferView":1,"componentType":5123,"count":3,"type":"SCALAR"}
            ],
            "meshes":[{"primitives":[{"attributes":{"POSITION":0},"indices":1}]}],
            "nodes":[{"mesh":0}],
            "scenes":[{"nodes":[0]}],
            "scene":0
        }"#;
        let path = directory.join("triangle.glb");
        std::fs::write(&path, glb_bytes(json, &binary)).expect("write fixture glb");
        GltfFixture { directory, path }
    }

    fn fixture_directory(kind: &str) -> std::path::PathBuf {
        use std::sync::atomic::{AtomicUsize, Ordering};

        static NEXT_FIXTURE: AtomicUsize = AtomicUsize::new(0);
        let sequence = NEXT_FIXTURE.fetch_add(1, Ordering::Relaxed);
        std::env::temp_dir().join(format!(
            "pge-collision-{kind}-fixture-{}-{sequence}",
            std::process::id()
        ))
    }

    fn glb_bytes(json: &[u8], binary: &[u8]) -> Vec<u8> {
        const GLB_MAGIC: u32 = 0x4654_6C67;
        const GLB_VERSION: u32 = 2;
        const JSON_CHUNK: u32 = 0x4E4F_534A;
        const BIN_CHUNK: u32 = 0x004E_4942;

        let mut json = json.to_vec();
        while json.len() % 4 != 0 {
            json.push(b' ');
        }
        let mut binary = binary.to_vec();
        while binary.len() % 4 != 0 {
            binary.push(0);
        }
        let total_length = 12 + 8 + json.len() + 8 + binary.len();
        let mut bytes = Vec::with_capacity(total_length);
        for value in [
            GLB_MAGIC,
            GLB_VERSION,
            total_length as u32,
            json.len() as u32,
            JSON_CHUNK,
        ] {
            bytes.extend_from_slice(&value.to_le_bytes());
        }
        bytes.extend_from_slice(&json);
        for value in [binary.len() as u32, BIN_CHUNK] {
            bytes.extend_from_slice(&value.to_le_bytes());
        }
        bytes.extend_from_slice(&binary);
        bytes
    }
}
