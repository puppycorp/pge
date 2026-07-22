use std::cmp::Ordering;
use std::collections::{BTreeMap, BTreeSet, HashMap, HashSet};
use std::fmt;
use std::time::Instant;

use parry3d::query::{cast_shapes, distance, ShapeCastOptions};
use rapier3d::na::{Matrix3, Quaternion, UnitQuaternion};
use rapier3d::prelude::*;

/// Experimental API version. This becomes `1` only after the migration gates pass.
pub const PHYSICS_API_VERSION: u32 = 0;
pub const PHYSICS_CHECKPOINT_VERSION: u32 = 1;
const PHYSICS_IMPLEMENTATION_ID: &str = "rapier3d-0.23";

#[derive(Clone, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct BodyId(pub String);

impl BodyId {
    pub fn new(value: impl Into<String>) -> Self {
        Self(value.into())
    }
}

#[derive(Clone, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct ColliderId(pub String);

impl ColliderId {
    pub fn new(value: impl Into<String>) -> Self {
        Self(value.into())
    }
}

#[derive(Clone, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct JointId(pub String);

impl JointId {
    pub fn new(value: impl Into<String>) -> Self {
        Self(value.into())
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Pose {
    pub translation: [f32; 3],
    /// Quaternion in `[x, y, z, w]` order.
    pub rotation_xyzw: [f32; 4],
}

impl Default for Pose {
    fn default() -> Self {
        Self {
            translation: [0.0; 3],
            rotation_xyzw: [0.0, 0.0, 0.0, 1.0],
        }
    }
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum BodyMode {
    Static,
    #[default]
    Dynamic,
    KinematicPosition,
    KinematicVelocity,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct MassPropertiesDesc {
    pub mass_kg: f32,
    pub center_of_mass_m: [f32; 3],
    pub principal_inertia_kg_m2: [f32; 3],
    /// Principal inertia frame in `[x, y, z, w]` quaternion order.
    pub principal_inertia_frame_xyzw: [f32; 4],
    /// Optional full inertia tensor in the body's local frame. When present,
    /// this takes precedence over `principal_inertia_kg_m2` and preserves
    /// rotated or asymmetric compound-body mass properties.
    pub inertia_tensor_kg_m2: Option<[[f32; 3]; 3]>,
}

impl Default for MassPropertiesDesc {
    fn default() -> Self {
        Self {
            mass_kg: 1.0,
            center_of_mass_m: [0.0; 3],
            principal_inertia_kg_m2: [1.0; 3],
            principal_inertia_frame_xyzw: [0.0, 0.0, 0.0, 1.0],
            inertia_tensor_kg_m2: None,
        }
    }
}

#[derive(Clone, Debug, PartialEq)]
pub struct BodyDesc {
    pub mode: BodyMode,
    pub pose: Pose,
    pub linear_velocity_mps: [f32; 3],
    pub angular_velocity_rps: [f32; 3],
    pub linear_damping: f32,
    pub angular_damping: f32,
    pub gravity_scale: f32,
    pub mass: Option<MassPropertiesDesc>,
    pub ccd_enabled: bool,
    pub lock_translation: [bool; 3],
    pub lock_rotation: [bool; 3],
    pub sleeping: bool,
}

impl Default for BodyDesc {
    fn default() -> Self {
        Self {
            mode: BodyMode::Dynamic,
            pose: Pose::default(),
            linear_velocity_mps: [0.0; 3],
            angular_velocity_rps: [0.0; 3],
            linear_damping: 0.0,
            angular_damping: 0.0,
            gravity_scale: 1.0,
            mass: None,
            ccd_enabled: false,
            lock_translation: [false; 3],
            lock_rotation: [false; 3],
            sleeping: false,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct BoundedKinematicTarget {
    pub pose: Pose,
    pub maximum_linear_speed_mps: f32,
    pub maximum_angular_speed_rps: f32,
    pub maximum_linear_acceleration_mps2: f32,
    pub maximum_angular_acceleration_rps2: f32,
}

/// Selects how a bounded kinematic pose target advances its two pose domains.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum KinematicTargetMode {
    /// Translation and rotation consume their independently bounded fractions.
    #[default]
    Independent,
    /// Translation and rotation consume the smaller bounded fraction together.
    CoupledPose,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct JointLimitDesc {
    pub minimum: f32,
    pub maximum: f32,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct JointMotorDesc {
    pub target_position: f32,
    pub target_velocity: f32,
    pub stiffness: f32,
    pub damping: f32,
    pub maximum_force: f32,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum JointMotorAxis {
    Primary,
    AngularX,
    AngularY,
    AngularZ,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum JointLimitState {
    BelowMinimum,
    WithinLimits,
    AboveMaximum,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct JointBreakThresholds {
    pub maximum_force_n: Option<f32>,
    pub maximum_torque_nm: Option<f32>,
    pub maximum_linear_impulse_ns: Option<f32>,
    pub maximum_angular_impulse_nms: Option<f32>,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum JointBreakCause {
    Force,
    Torque,
    LinearImpulse,
    AngularImpulse,
}

#[derive(Clone, Debug, PartialEq)]
pub struct JointBreakEvent {
    pub joint: JointId,
    pub cause: JointBreakCause,
    pub observed: f32,
    pub threshold: f32,
    /// Resulting fixed-step index, starting at one.
    pub step_index: u64,
    /// Zero-based fixed substep where the threshold was exceeded.
    pub substep_index: u32,
    /// Stable zero-based order within this step's joint-break stream.
    pub sequence: u32,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum PhysicsCapability {
    MultibodyJointChains,
    JointFriction,
    AutomaticJointBreakThresholds,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct PhysicsCapabilities {
    pub multibody_joint_chains: bool,
    pub joint_friction: bool,
    pub automatic_joint_break_thresholds: bool,
    pub in_place_joint_motor_updates: bool,
    pub joint_limit_observation: bool,
    pub checkpoint_state_digest: bool,
    pub backend_phase_timings: bool,
    pub exact_resource_bytes: bool,
}

#[derive(Clone, Debug, PartialEq)]
pub enum JointKindDesc {
    Fixed,
    /// Rotation about `axis`, expressed in each joint frame.
    Revolute {
        axis: [f32; 3],
        limits: Option<JointLimitDesc>,
        motor: Option<JointMotorDesc>,
    },
    /// Translation along `axis`, expressed in each joint frame.
    Prismatic {
        axis: [f32; 3],
        limits: Option<JointLimitDesc>,
        motor: Option<JointMotorDesc>,
    },
    /// A ball-and-socket joint. Limit and motor array entries correspond to
    /// angular X, Y, and Z in the joint frames.
    Spherical {
        limits: [Option<JointLimitDesc>; 3],
        motors: [Option<JointMotorDesc>; 3],
    },
}

#[derive(Clone, Debug, PartialEq)]
pub struct JointDesc {
    pub body1: BodyId,
    pub body2: BodyId,
    pub local_frame1: Pose,
    pub local_frame2: Pose,
    pub kind: JointKindDesc,
    pub contacts_enabled: bool,
}

#[derive(Clone, Debug, PartialEq)]
pub enum ColliderShape {
    Box {
        size: [f32; 3],
    },
    Sphere {
        radius: f32,
    },
    CapsuleY {
        half_height: f32,
        radius: f32,
    },
    CylinderY {
        half_height: f32,
        radius: f32,
    },
    ConeY {
        half_height: f32,
        radius: f32,
    },
    ConvexHull {
        points: Vec<[f32; 3]>,
    },
    TriangleMesh {
        vertices: Vec<[f32; 3]>,
        indices: Vec<[u32; 3]>,
    },
    /// A row-major grid of heights over the local X-Z plane. `scale` controls
    /// the total X/Z footprint and multiplies the authored Y heights.
    HeightField {
        rows: usize,
        columns: usize,
        heights: Vec<f32>,
        scale: [f32; 3],
    },
    Compound {
        children: Vec<ColliderChildDesc>,
    },
}

#[derive(Clone, Debug, PartialEq)]
pub struct ColliderChildDesc {
    pub pose: Pose,
    pub shape: ColliderShape,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ColliderMaterial {
    pub friction: f32,
    pub restitution: f32,
    pub density_kg_m3: f32,
    pub contact_skin_m: f32,
}

impl Default for ColliderMaterial {
    fn default() -> Self {
        Self {
            friction: 0.5,
            restitution: 0.0,
            density_kg_m3: 1.0,
            contact_skin_m: 0.0,
        }
    }
}

#[derive(Clone, Debug, PartialEq)]
pub struct ColliderDesc {
    pub pose: Pose,
    pub shape: ColliderShape,
    pub material: ColliderMaterial,
    pub sensor: bool,
    pub collision_memberships: u32,
    pub collision_filter: u32,
}

impl ColliderDesc {
    pub fn new(shape: ColliderShape) -> Self {
        Self {
            pose: Pose::default(),
            shape,
            material: ColliderMaterial::default(),
            sensor: false,
            collision_memberships: u32::MAX,
            collision_filter: u32::MAX,
        }
    }
}

#[derive(Clone, Debug, PartialEq)]
pub struct PhysicsConfig {
    pub gravity_mps2: [f32; 3],
    pub fixed_dt_sec: f32,
    pub substeps: u32,
}

impl Default for PhysicsConfig {
    fn default() -> Self {
        Self {
            gravity_mps2: [0.0, 0.0, -9.81],
            fixed_dt_sec: 1.0 / 60.0,
            substeps: 1,
        }
    }
}

#[derive(Clone, Debug, PartialEq)]
pub struct EffectiveMassProperties {
    pub mass_kg: f32,
    pub center_of_mass_m: [f32; 3],
    pub inertia_tensor_kg_m2: [[f32; 3]; 3],
    /// Translational mass after axis locks are applied.
    pub effective_translation_mass_kg: [f32; 3],
}

#[derive(Clone, Debug, PartialEq)]
pub struct BodySnapshot {
    pub id: BodyId,
    pub mode: BodyMode,
    pub pose: Pose,
    pub linear_velocity_mps: [f32; 3],
    pub angular_velocity_rps: [f32; 3],
    pub sleeping: bool,
    /// Whether the backend currently includes this body in an active solver set.
    pub solver_active: bool,
    pub ccd_enabled: bool,
    pub ccd_active: bool,
    pub authored_mass: Option<MassPropertiesDesc>,
    pub effective_mass: EffectiveMassProperties,
}

#[derive(Clone, Debug, PartialEq)]
pub struct ColliderSnapshot {
    pub id: ColliderId,
    pub body_id: BodyId,
    pub world_pose: Pose,
    pub desc: ColliderDesc,
}

#[derive(Clone, Debug, PartialEq)]
pub struct JointSnapshot {
    pub id: JointId,
    pub desc: JointDesc,
    /// Scalar joint coordinate for revolute and prismatic joints.
    pub position: Option<f32>,
    /// Scalar joint velocity for revolute and prismatic joints.
    pub velocity: Option<f32>,
    /// Relative joint-frame orientation, in `[x, y, z, w]` order.
    pub relative_rotation_xyzw: [f32; 4],
    /// Constraint impulses in linear XYZ then angular XYZ order.
    pub applied_impulse: [f32; 6],
    pub limit_state: Option<JointLimitState>,
    /// Signed distance outside the configured interval, or zero inside it.
    pub limit_error: Option<f32>,
    /// Motor target minus the observed primary coordinate.
    pub motor_position_error: Option<f32>,
    /// Solved scalar effort along the primary joint axis.
    pub applied_effort: Option<f32>,
    /// Configured Coulomb friction bound. Units are N for prismatic joints and
    /// N*m for revolute/spherical joints. `None` means friction is disabled.
    pub friction_maximum_effort: Option<f32>,
    /// PGE-applied friction impulse in linear XYZ then angular XYZ order.
    pub friction_applied_impulse: [f32; 6],
    pub break_thresholds: Option<JointBreakThresholds>,
    /// Residual constrained displacement in joint-frame linear XYZ then
    /// angular XYZ order. Permitted degrees of freedom are zeroed.
    pub constraint_error: [f32; 6],
}

#[derive(Clone, Debug, PartialEq)]
pub struct ContactSnapshot {
    pub collider1: ColliderId,
    pub collider2: ColliderId,
    pub sensor: bool,
    pub normal: [f32; 3],
    pub total_impulse_ns: [f32; 3],
    pub total_impulse_magnitude_ns: f32,
    pub manifolds: Vec<ContactManifoldSnapshot>,
}

#[derive(Clone, Debug, PartialEq)]
pub struct ContactManifoldSnapshot {
    /// World-space normal whose impulse is applied to `collider1`.
    pub normal_on_collider1: [f32; 3],
    pub subshape1: u32,
    pub subshape2: u32,
    pub points: Vec<ContactPointSnapshot>,
}

#[derive(Clone, Debug, PartialEq)]
pub struct ContactPointSnapshot {
    pub point1_m: [f32; 3],
    pub point2_m: [f32; 3],
    /// Signed separation; negative values indicate penetration.
    pub distance_m: f32,
    pub penetration_depth_m: f32,
    /// Velocity of collider2 relative to collider1 at the contact points.
    pub relative_velocity_mps: [f32; 3],
    pub normal_impulse_ns: f32,
    /// Magnitude of the solved two-axis friction impulse. `None` means the
    /// backend did not produce a finite tangent solution for this point.
    pub tangent_impulse_magnitude_ns: Option<f32>,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum PhysicsEventKind {
    ContactStarted,
    ContactStopped,
    SensorStarted,
    SensorStopped,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct PhysicsEvent {
    pub kind: PhysicsEventKind,
    pub collider1: ColliderId,
    pub collider2: ColliderId,
    /// Resulting fixed-step index, starting at one.
    pub step_index: u64,
    /// Zero-based fixed substep where the transition was observed.
    pub substep_index: u32,
    /// Stable zero-based order within this step's event stream.
    pub sequence: u32,
}

#[derive(Clone, Debug, PartialEq)]
pub struct PhysicsSnapshot {
    pub api_version: u32,
    pub step_index: u64,
    pub simulation_time_sec: f64,
    pub bodies: Vec<BodySnapshot>,
    pub colliders: Vec<ColliderSnapshot>,
    pub joints: Vec<JointSnapshot>,
    pub contacts: Vec<ContactSnapshot>,
    pub debug_geometry: Vec<DebugGeometryRecord>,
}

#[derive(Clone, Debug, PartialEq)]
pub struct DebugGeometryRecord {
    pub collider: ColliderId,
    pub body_id: BodyId,
    /// Child indices from the collider root to this leaf shape.
    pub child_path: Vec<u32>,
    pub world_pose: Pose,
    pub shape: ColliderShape,
    pub sensor: bool,
    pub collision_memberships: u32,
    pub collision_filter: u32,
}

#[derive(Clone, Debug, Default, PartialEq)]
pub struct PhysicsDiagnostics {
    pub body_count: usize,
    pub collider_count: usize,
    pub joint_count: usize,
    pub active_contact_count: usize,
    pub active_sensor_count: usize,
    pub sleeping_body_count: usize,
    pub active_dynamic_body_count: usize,
    pub active_kinematic_body_count: usize,
    pub ccd_enabled_body_count: usize,
    pub ccd_active_body_count: usize,
    pub contact_manifold_count: usize,
    pub contact_point_count: usize,
    /// Rapier does not expose sleeping-island totals through its public API.
    pub sleeping_island_count: Option<usize>,
    /// Exact allocator/resource byte accounting is not exposed by the backend.
    pub estimated_resource_bytes: Option<usize>,
    /// Only whole-step wall time is available; backend phase timings are not.
    pub backend_phase_seconds: Option<BackendPhaseTimings>,
    pub last_step_seconds: f64,
}

#[derive(Clone, Debug, Default, PartialEq)]
pub struct BackendPhaseTimings {
    pub broad_phase_seconds: Option<f64>,
    pub narrow_phase_seconds: Option<f64>,
    pub solver_seconds: Option<f64>,
    pub ccd_seconds: Option<f64>,
}

#[derive(Clone, Debug, PartialEq)]
pub struct StepOutput {
    pub snapshot: PhysicsSnapshot,
    pub events: Vec<PhysicsEvent>,
    pub joint_breaks: Vec<JointBreakEvent>,
    pub diagnostics: PhysicsDiagnostics,
}

#[derive(Clone, Debug, Default, PartialEq)]
pub struct StepInput {
    /// Applied atomically in vector order immediately before the fixed step.
    pub commands: Vec<PhysicsCommand>,
}

#[derive(Clone, Debug, PartialEq)]
pub enum PhysicsCommand {
    CreateBody {
        id: BodyId,
        desc: BodyDesc,
    },
    RemoveBody {
        id: BodyId,
    },
    CreateCollider {
        id: ColliderId,
        body_id: BodyId,
        desc: ColliderDesc,
    },
    RemoveCollider {
        id: ColliderId,
    },
    CreateJoint {
        id: JointId,
        desc: JointDesc,
    },
    CreateMultibodyJoint {
        id: JointId,
        desc: JointDesc,
    },
    UpdateJoint {
        id: JointId,
        desc: JointDesc,
    },
    RemoveJoint {
        id: JointId,
    },
    SetJointFriction {
        id: JointId,
        maximum_effort: f32,
    },
    SetJointBreakThresholds {
        id: JointId,
        thresholds: JointBreakThresholds,
    },
    SetBodyMode {
        id: BodyId,
        mode: BodyMode,
        wake_up: bool,
    },
    SetBodyPose {
        id: BodyId,
        pose: Pose,
        wake_up: bool,
    },
    SetBodyVelocity {
        id: BodyId,
        linear_mps: [f32; 3],
        angular_rps: [f32; 3],
        wake_up: bool,
    },
    SetNextKinematicPose {
        id: BodyId,
        pose: Pose,
    },
    SetBoundedKinematicTarget {
        id: BodyId,
        target: BoundedKinematicTarget,
    },
    SetBoundedKinematicTargetWithMode {
        id: BodyId,
        target: BoundedKinematicTarget,
        mode: KinematicTargetMode,
    },
    ClearBoundedKinematicTarget {
        id: BodyId,
    },
    AddForce {
        id: BodyId,
        force_n: [f32; 3],
        wake_up: bool,
    },
    AddForceAtPoint {
        id: BodyId,
        force_n: [f32; 3],
        point_world_m: [f32; 3],
        wake_up: bool,
    },
    ApplyImpulse {
        id: BodyId,
        impulse_ns: [f32; 3],
        wake_up: bool,
    },
    AddTorque {
        id: BodyId,
        torque_nm: [f32; 3],
        wake_up: bool,
    },
    ApplyTorqueImpulse {
        id: BodyId,
        impulse_nms: [f32; 3],
        wake_up: bool,
    },
    WakeUp {
        id: BodyId,
    },
}

#[derive(Clone, Debug, PartialEq)]
pub struct RayHit {
    pub collider: ColliderId,
    pub distance_m: f32,
    pub point_m: [f32; 3],
    pub normal: [f32; 3],
}

/// Collision groups used by a scene query. A collider is eligible when both
/// its memberships intersect this filter and these memberships intersect the
/// collider's filter.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct PhysicsQueryGroups {
    pub memberships: u32,
    pub filter: u32,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct PhysicsQueryFilter {
    pub excluded_bodies: Vec<BodyId>,
    pub excluded_colliders: Vec<ColliderId>,
    pub groups: Option<PhysicsQueryGroups>,
    pub include_sensors: bool,
}

impl Default for PhysicsQueryFilter {
    fn default() -> Self {
        Self {
            excluded_bodies: Vec::new(),
            excluded_colliders: Vec::new(),
            groups: None,
            include_sensors: true,
        }
    }
}

#[derive(Clone, Debug, PartialEq)]
pub struct PointProjection {
    pub collider: ColliderId,
    pub point_m: [f32; 3],
    pub distance_m: f32,
    pub is_inside: bool,
}

#[derive(Clone, Debug, PartialEq)]
pub struct ShapeCastHit {
    pub collider: ColliderId,
    pub time_of_impact_sec: f32,
    pub witness1_m: [f32; 3],
    pub witness2_m: [f32; 3],
    pub normal1: [f32; 3],
    pub normal2: [f32; 3],
}

#[derive(Clone, Debug, PartialEq)]
pub struct ProximityHit {
    pub collider: ColliderId,
    pub distance_m: f32,
}

#[derive(Clone)]
pub struct PhysicsCheckpoint {
    checkpoint_version: u32,
    implementation_id: &'static str,
    config: PhysicsConfig,
    step_index: u64,
    simulation_time_sec: f64,
    backend_state: BackendCheckpoint,
    provenance: CheckpointProvenance,
}

impl PhysicsCheckpoint {
    pub fn provenance(&self) -> &CheckpointProvenance {
        &self.provenance
    }
}

#[derive(Clone, Debug, PartialEq)]
pub struct CheckpointProvenance {
    pub checkpoint_version: u32,
    pub physics_api_version: u32,
    pub config: PhysicsConfig,
    pub step_index: u64,
    pub simulation_time_sec: f64,
    pub state_digest_algorithm: &'static str,
    pub state_digest: u64,
}

#[derive(Clone, Debug, PartialEq)]
pub struct CheckpointComparison {
    pub compatible: bool,
    pub checkpoint_version_matches: bool,
    pub api_version_matches: bool,
    pub implementation_matches: bool,
    pub config_matches: bool,
    pub state_matches: bool,
    pub current_state_digest: u64,
    pub checkpoint_state_digest: u64,
    pub first_divergence: Option<&'static str>,
}

#[derive(Clone)]
struct BackendCheckpoint {
    gravity: Vector<Real>,
    integration_parameters: IntegrationParameters,
    island_manager: IslandManager,
    broad_phase: DefaultBroadPhase,
    narrow_phase: NarrowPhase,
    bodies: RigidBodySet,
    colliders: ColliderSet,
    impulse_joints: ImpulseJointSet,
    multibody_joints: MultibodyJointSet,
    ccd_solver: CCDSolver,
    query_pipeline: QueryPipeline,
    body_records: BTreeMap<BodyId, BodyRecord>,
    collider_records: BTreeMap<ColliderId, ColliderRecord>,
    joint_records: BTreeMap<JointId, JointRecord>,
    collider_ids: HashMap<ColliderHandle, ColliderId>,
    active_contacts: BTreeSet<PairKey>,
    active_sensors: BTreeSet<PairKey>,
    pending_events: Vec<PhysicsEvent>,
    retired_body_ids: BTreeSet<BodyId>,
    retired_collider_ids: BTreeSet<ColliderId>,
    retired_joint_ids: BTreeSet<JointId>,
    kinematic_targets: BTreeMap<BodyId, KinematicTargetState>,
    diagnostics: PhysicsDiagnostics,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum PhysicsError {
    DuplicateBody(BodyId),
    DuplicateCollider(ColliderId),
    DuplicateJoint(JointId),
    RetiredBody(BodyId),
    RetiredCollider(ColliderId),
    RetiredJoint(JointId),
    UnknownBody(BodyId),
    UnknownCollider(ColliderId),
    UnknownJoint(JointId),
    InvalidValue(&'static str),
    InvalidShape(String),
    UnsupportedCapability(PhysicsCapability),
    IncompatibleCheckpoint,
}

impl fmt::Display for PhysicsError {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::DuplicateBody(id) => write!(formatter, "duplicate physics body '{}'", id.0),
            Self::DuplicateCollider(id) => {
                write!(formatter, "duplicate physics collider '{}'", id.0)
            }
            Self::DuplicateJoint(id) => write!(formatter, "duplicate physics joint '{}'", id.0),
            Self::RetiredBody(id) => write!(
                formatter,
                "physics body ID '{}' was retired and cannot be reused",
                id.0
            ),
            Self::RetiredCollider(id) => write!(
                formatter,
                "physics collider ID '{}' was retired and cannot be reused",
                id.0
            ),
            Self::RetiredJoint(id) => write!(
                formatter,
                "physics joint ID '{}' was retired and cannot be reused",
                id.0
            ),
            Self::UnknownBody(id) => write!(formatter, "unknown physics body '{}'", id.0),
            Self::UnknownCollider(id) => {
                write!(formatter, "unknown physics collider '{}'", id.0)
            }
            Self::UnknownJoint(id) => write!(formatter, "unknown physics joint '{}'", id.0),
            Self::InvalidValue(name) => write!(formatter, "invalid physics value '{name}'"),
            Self::InvalidShape(message) => write!(formatter, "invalid collider shape: {message}"),
            Self::UnsupportedCapability(capability) => {
                write!(formatter, "unsupported physics capability: {capability:?}")
            }
            Self::IncompatibleCheckpoint => write!(formatter, "incompatible physics checkpoint"),
        }
    }
}

impl std::error::Error for PhysicsError {}

#[derive(Clone)]
struct BodyRecord {
    handle: RigidBodyHandle,
    desc: BodyDesc,
}

#[derive(Clone)]
struct ColliderRecord {
    handle: ColliderHandle,
    body_id: BodyId,
    desc: ColliderDesc,
}

struct ResolvedQueryFilter {
    excluded_bodies: HashSet<RigidBodyHandle>,
    excluded_colliders: HashSet<ColliderHandle>,
    groups: Option<InteractionGroups>,
    include_sensors: bool,
}

impl ResolvedQueryFilter {
    fn allows(&self, handle: ColliderHandle, collider: &Collider) -> bool {
        !self.excluded_colliders.contains(&handle)
            && collider
                .parent()
                .is_none_or(|parent| !self.excluded_bodies.contains(&parent))
            && (self.include_sensors || !collider.is_sensor())
            && self
                .groups
                .is_none_or(|groups| collider.collision_groups().test(groups))
    }
}

#[derive(Clone, Copy)]
enum JointBackendHandle {
    Impulse(ImpulseJointHandle),
    Multibody(MultibodyJointHandle),
}

#[derive(Clone)]
struct JointRecord {
    handle: JointBackendHandle,
    desc: JointDesc,
    friction_maximum_effort: Option<f32>,
    friction_applied_impulse: [f32; 6],
    break_thresholds: Option<JointBreakThresholds>,
    observed_impulse: [f32; 6],
}

#[derive(Clone, Copy)]
struct JointVelocityObservation {
    linear: Vector<Real>,
    angular: Vector<Real>,
}

#[derive(Clone, Copy)]
struct KinematicTargetState {
    target: BoundedKinematicTarget,
    mode: KinematicTargetMode,
    linear_speed_mps: f32,
    angular_speed_rps: f32,
}

#[derive(Clone, Debug, PartialEq, Eq, PartialOrd, Ord)]
struct PairKey(ColliderId, ColliderId);

impl PairKey {
    fn new(first: ColliderId, second: ColliderId) -> Self {
        if first <= second {
            Self(first, second)
        } else {
            Self(second, first)
        }
    }
}

pub struct PhysicsWorld {
    config: PhysicsConfig,
    gravity: Vector<Real>,
    integration_parameters: IntegrationParameters,
    pipeline: PhysicsPipeline,
    island_manager: IslandManager,
    broad_phase: DefaultBroadPhase,
    narrow_phase: NarrowPhase,
    bodies: RigidBodySet,
    colliders: ColliderSet,
    impulse_joints: ImpulseJointSet,
    multibody_joints: MultibodyJointSet,
    ccd_solver: CCDSolver,
    query_pipeline: QueryPipeline,
    body_records: BTreeMap<BodyId, BodyRecord>,
    collider_records: BTreeMap<ColliderId, ColliderRecord>,
    joint_records: BTreeMap<JointId, JointRecord>,
    collider_ids: HashMap<ColliderHandle, ColliderId>,
    active_contacts: BTreeSet<PairKey>,
    active_sensors: BTreeSet<PairKey>,
    pending_events: Vec<PhysicsEvent>,
    retired_body_ids: BTreeSet<BodyId>,
    retired_collider_ids: BTreeSet<ColliderId>,
    retired_joint_ids: BTreeSet<JointId>,
    kinematic_targets: BTreeMap<BodyId, KinematicTargetState>,
    step_index: u64,
    simulation_time_sec: f64,
    diagnostics: PhysicsDiagnostics,
}

impl PhysicsWorld {
    pub fn new(config: PhysicsConfig) -> Result<Self, PhysicsError> {
        validate_config(&config)?;
        let integration_parameters = IntegrationParameters {
            dt: config.fixed_dt_sec / config.substeps as f32,
            ..IntegrationParameters::default()
        };
        Ok(Self {
            gravity: vec3(config.gravity_mps2),
            config,
            integration_parameters,
            pipeline: PhysicsPipeline::new(),
            island_manager: IslandManager::new(),
            broad_phase: DefaultBroadPhase::new(),
            narrow_phase: NarrowPhase::new(),
            bodies: RigidBodySet::new(),
            colliders: ColliderSet::new(),
            impulse_joints: ImpulseJointSet::new(),
            multibody_joints: MultibodyJointSet::new(),
            ccd_solver: CCDSolver::new(),
            query_pipeline: QueryPipeline::new(),
            body_records: BTreeMap::new(),
            collider_records: BTreeMap::new(),
            joint_records: BTreeMap::new(),
            collider_ids: HashMap::new(),
            active_contacts: BTreeSet::new(),
            active_sensors: BTreeSet::new(),
            pending_events: Vec::new(),
            retired_body_ids: BTreeSet::new(),
            retired_collider_ids: BTreeSet::new(),
            retired_joint_ids: BTreeSet::new(),
            kinematic_targets: BTreeMap::new(),
            step_index: 0,
            simulation_time_sec: 0.0,
            diagnostics: PhysicsDiagnostics::default(),
        })
    }

    pub fn config(&self) -> &PhysicsConfig {
        &self.config
    }

    /// Updates the duration and internal subdivision of subsequent steps
    /// without rebuilding the persistent world.
    pub fn set_step_timing(
        &mut self,
        fixed_dt_sec: f32,
        substeps: u32,
    ) -> Result<(), PhysicsError> {
        let mut config = self.config.clone();
        config.fixed_dt_sec = fixed_dt_sec;
        config.substeps = substeps;
        validate_config(&config)?;
        self.integration_parameters.dt = fixed_dt_sec / substeps as f32;
        self.config = config;
        Ok(())
    }

    /// Starts a new public timeline without changing the solved physical
    /// state. Active contacts are intentionally retained so a caller that
    /// performed private initialization/settling does not publish synthetic
    /// `ContactStarted` events on its first visible step.
    pub fn reset_timeline(&mut self) {
        self.step_index = 0;
        self.simulation_time_sec = 0.0;
        self.pending_events.clear();
    }

    pub fn create_body(&mut self, id: BodyId, desc: BodyDesc) -> Result<(), PhysicsError> {
        if self.body_records.contains_key(&id) {
            return Err(PhysicsError::DuplicateBody(id));
        }
        if self.retired_body_ids.contains(&id) {
            return Err(PhysicsError::RetiredBody(id));
        }
        validate_body(&desc)?;
        let handle = self.bodies.insert(body_builder(&desc).build());
        self.bodies[handle].recompute_mass_properties_from_colliders(&self.colliders);
        self.body_records.insert(id, BodyRecord { handle, desc });
        self.refresh_diagnostics(0.0);
        Ok(())
    }

    pub fn remove_body(&mut self, id: &BodyId) -> Result<(), PhysicsError> {
        let record = self
            .body_records
            .remove(id)
            .ok_or_else(|| PhysicsError::UnknownBody(id.clone()))?;
        let removed_colliders = self
            .collider_records
            .iter()
            .filter(|(_, collider)| collider.body_id == *id)
            .map(|(collider_id, collider)| (collider_id.clone(), collider.handle))
            .collect::<Vec<_>>();
        let removed_joints = self
            .joint_records
            .iter()
            .filter(|(_, joint)| joint.desc.body1 == *id || joint.desc.body2 == *id)
            .map(|(joint_id, _)| joint_id.clone())
            .collect::<Vec<_>>();
        self.bodies.remove(
            record.handle,
            &mut self.island_manager,
            &mut self.colliders,
            &mut self.impulse_joints,
            &mut self.multibody_joints,
            true,
        );
        for (collider_id, handle) in removed_colliders {
            self.queue_pair_removal_events(&collider_id);
            self.collider_records.remove(&collider_id);
            self.collider_ids.remove(&handle);
            self.retired_collider_ids.insert(collider_id);
        }
        for joint_id in removed_joints {
            self.joint_records.remove(&joint_id);
            self.retired_joint_ids.insert(joint_id);
        }
        self.retired_body_ids.insert(id.clone());
        self.kinematic_targets.remove(id);
        self.discard_removed_pairs();
        self.query_pipeline.update(&self.colliders);
        self.refresh_diagnostics(0.0);
        Ok(())
    }

    pub fn create_collider(
        &mut self,
        id: ColliderId,
        body_id: &BodyId,
        desc: ColliderDesc,
    ) -> Result<(), PhysicsError> {
        if self.collider_records.contains_key(&id) {
            return Err(PhysicsError::DuplicateCollider(id));
        }
        if self.retired_collider_ids.contains(&id) {
            return Err(PhysicsError::RetiredCollider(id));
        }
        let body = self
            .body_records
            .get(body_id)
            .ok_or_else(|| PhysicsError::UnknownBody(body_id.clone()))?;
        let collider = collider_builder(&desc, body.desc.mass.is_some())?.build();
        let handle = self
            .colliders
            .insert_with_parent(collider, body.handle, &mut self.bodies);
        self.collider_ids.insert(handle, id.clone());
        self.collider_records.insert(
            id,
            ColliderRecord {
                handle,
                body_id: body_id.clone(),
                desc,
            },
        );
        self.query_pipeline.update(&self.colliders);
        self.refresh_diagnostics(0.0);
        Ok(())
    }

    pub fn remove_collider(&mut self, id: &ColliderId) -> Result<(), PhysicsError> {
        let record = self
            .collider_records
            .remove(id)
            .ok_or_else(|| PhysicsError::UnknownCollider(id.clone()))?;
        self.queue_pair_removal_events(id);
        self.colliders.remove(
            record.handle,
            &mut self.island_manager,
            &mut self.bodies,
            true,
        );
        self.collider_ids.remove(&record.handle);
        self.retired_collider_ids.insert(id.clone());
        self.discard_removed_pairs();
        self.query_pipeline.update(&self.colliders);
        self.refresh_diagnostics(0.0);
        Ok(())
    }

    pub fn create_joint(&mut self, id: JointId, desc: JointDesc) -> Result<(), PhysicsError> {
        if self.joint_records.contains_key(&id) {
            return Err(PhysicsError::DuplicateJoint(id));
        }
        if self.retired_joint_ids.contains(&id) {
            return Err(PhysicsError::RetiredJoint(id));
        }
        validate_joint(&desc)?;
        let body1 = self
            .body_records
            .get(&desc.body1)
            .ok_or_else(|| PhysicsError::UnknownBody(desc.body1.clone()))?
            .handle;
        let body2 = self
            .body_records
            .get(&desc.body2)
            .ok_or_else(|| PhysicsError::UnknownBody(desc.body2.clone()))?
            .handle;
        if body1 == body2 {
            return Err(PhysicsError::InvalidValue("joint bodies"));
        }
        let handle = self
            .impulse_joints
            .insert(body1, body2, joint_data(&desc)?, true);
        self.joint_records.insert(
            id,
            JointRecord {
                handle: JointBackendHandle::Impulse(handle),
                desc,
                friction_maximum_effort: None,
                friction_applied_impulse: [0.0; 6],
                break_thresholds: None,
                observed_impulse: [0.0; 6],
            },
        );
        self.refresh_diagnostics(0.0);
        Ok(())
    }

    pub fn update_joint(&mut self, id: &JointId, desc: JointDesc) -> Result<(), PhysicsError> {
        validate_joint(&desc)?;
        let old = self
            .joint_records
            .get(id)
            .ok_or_else(|| PhysicsError::UnknownJoint(id.clone()))?
            .clone();
        let body1 = self
            .body_records
            .get(&desc.body1)
            .ok_or_else(|| PhysicsError::UnknownBody(desc.body1.clone()))?
            .handle;
        let body2 = self
            .body_records
            .get(&desc.body2)
            .ok_or_else(|| PhysicsError::UnknownBody(desc.body2.clone()))?
            .handle;
        if body1 == body2 {
            return Err(PhysicsError::InvalidValue("joint bodies"));
        }
        let data = joint_data(&desc)?;
        let handle = match old.handle {
            JointBackendHandle::Impulse(handle) => {
                self.impulse_joints.remove(handle, true);
                JointBackendHandle::Impulse(self.impulse_joints.insert(body1, body2, data, true))
            }
            JointBackendHandle::Multibody(handle) => {
                let previous_joints = self.multibody_joints.clone();
                self.multibody_joints.remove(handle, true);
                let Some(handle) = self.multibody_joints.insert(body1, body2, data, true) else {
                    self.multibody_joints = previous_joints;
                    return Err(PhysicsError::InvalidValue("multibody joint topology"));
                };
                JointBackendHandle::Multibody(handle)
            }
        };
        self.joint_records.insert(
            id.clone(),
            JointRecord {
                handle,
                desc,
                friction_maximum_effort: old.friction_maximum_effort,
                friction_applied_impulse: [0.0; 6],
                break_thresholds: old.break_thresholds,
                observed_impulse: [0.0; 6],
            },
        );
        Ok(())
    }

    /// Updates only one motor in place, preserving the solver joint and its
    /// accumulated warm-start state.
    pub fn update_joint_motor(
        &mut self,
        id: &JointId,
        axis: JointMotorAxis,
        motor: Option<JointMotorDesc>,
    ) -> Result<(), PhysicsError> {
        validate_joint_motor(motor)?;
        let record = self
            .joint_records
            .get(id)
            .ok_or_else(|| PhysicsError::UnknownJoint(id.clone()))?;
        let mut desc = record.desc.clone();
        let handle = record.handle;
        let backend_axis = match (&mut desc.kind, axis) {
            (JointKindDesc::Revolute { motor: target, .. }, JointMotorAxis::Primary) => {
                *target = motor;
                JointAxis::AngX
            }
            (JointKindDesc::Prismatic { motor: target, .. }, JointMotorAxis::Primary) => {
                *target = motor;
                JointAxis::LinX
            }
            (JointKindDesc::Spherical { motors, .. }, JointMotorAxis::AngularX) => {
                motors[0] = motor;
                JointAxis::AngX
            }
            (JointKindDesc::Spherical { motors, .. }, JointMotorAxis::AngularY) => {
                motors[1] = motor;
                JointAxis::AngY
            }
            (JointKindDesc::Spherical { motors, .. }, JointMotorAxis::AngularZ) => {
                motors[2] = motor;
                JointAxis::AngZ
            }
            _ => return Err(PhysicsError::InvalidValue("joint motor axis")),
        };
        let joint = match handle {
            JointBackendHandle::Impulse(handle) => self
                .impulse_joints
                .get_mut(handle, true)
                .map(|joint| &mut joint.data),
            JointBackendHandle::Multibody(handle) => self
                .multibody_joints
                .get_mut(handle)
                .and_then(|(multibody, link_id)| multibody.link_mut(link_id))
                .map(|link| &mut link.joint.data),
        }
        .ok_or_else(|| PhysicsError::UnknownJoint(id.clone()))?;
        if let Some(motor) = motor {
            apply_joint_axis_config(joint, backend_axis, None, Some(motor));
        } else {
            joint.motor_axes.remove(backend_axis.into());
        }
        self.joint_records
            .get_mut(id)
            .expect("joint registry entry was resolved above")
            .desc = desc;
        Ok(())
    }

    pub fn create_multibody_joint(
        &mut self,
        id: JointId,
        desc: JointDesc,
    ) -> Result<(), PhysicsError> {
        if self.joint_records.contains_key(&id) {
            return Err(PhysicsError::DuplicateJoint(id));
        }
        if self.retired_joint_ids.contains(&id) {
            return Err(PhysicsError::RetiredJoint(id));
        }
        validate_joint(&desc)?;
        let body1 = self
            .body_records
            .get(&desc.body1)
            .ok_or_else(|| PhysicsError::UnknownBody(desc.body1.clone()))?
            .handle;
        let body2 = self
            .body_records
            .get(&desc.body2)
            .ok_or_else(|| PhysicsError::UnknownBody(desc.body2.clone()))?
            .handle;
        if body1 == body2 {
            return Err(PhysicsError::InvalidValue("joint bodies"));
        }
        let handle = self
            .multibody_joints
            .insert(body1, body2, joint_data(&desc)?, true)
            .ok_or(PhysicsError::InvalidValue("multibody joint topology"))?;
        self.joint_records.insert(
            id,
            JointRecord {
                handle: JointBackendHandle::Multibody(handle),
                desc,
                friction_maximum_effort: None,
                friction_applied_impulse: [0.0; 6],
                break_thresholds: None,
                observed_impulse: [0.0; 6],
            },
        );
        self.refresh_diagnostics(0.0);
        Ok(())
    }

    /// Configures bounded Coulomb friction on each free degree of freedom.
    /// The value is a maximum resisting force in N for prismatic joints and a
    /// maximum resisting torque in N*m for revolute/spherical joints. PGE
    /// applies a sign-opposing impulse before each solver substep and caps it
    /// both at `maximum_effort * dt` and at the impulse needed to reach zero
    /// relative speed, so friction never accelerates through rest.
    pub fn set_joint_friction(
        &mut self,
        id: &JointId,
        maximum_effort: f32,
    ) -> Result<(), PhysicsError> {
        if !maximum_effort.is_finite() || maximum_effort < 0.0 {
            return Err(PhysicsError::InvalidValue("joint friction"));
        }
        let record = self
            .joint_records
            .get_mut(id)
            .ok_or_else(|| PhysicsError::UnknownJoint(id.clone()))?;
        if matches!(record.desc.kind, JointKindDesc::Fixed) {
            return Err(PhysicsError::InvalidValue("fixed joint friction"));
        }
        record.friction_maximum_effort = (maximum_effort > 0.0).then_some(maximum_effort);
        record.friction_applied_impulse = [0.0; 6];
        Ok(())
    }

    pub fn set_joint_break_thresholds(
        &mut self,
        id: &JointId,
        thresholds: JointBreakThresholds,
    ) -> Result<(), PhysicsError> {
        let valid = [
            thresholds.maximum_force_n,
            thresholds.maximum_torque_nm,
            thresholds.maximum_linear_impulse_ns,
            thresholds.maximum_angular_impulse_nms,
        ]
        .into_iter()
        .flatten()
        .all(|value| value.is_finite() && value >= 0.0);
        if !valid {
            return Err(PhysicsError::InvalidValue("joint break thresholds"));
        }
        let record = self
            .joint_records
            .get_mut(id)
            .ok_or_else(|| PhysicsError::UnknownJoint(id.clone()))?;
        let disabled = thresholds.maximum_force_n.is_none()
            && thresholds.maximum_torque_nm.is_none()
            && thresholds.maximum_linear_impulse_ns.is_none()
            && thresholds.maximum_angular_impulse_nms.is_none();
        record.break_thresholds = (!disabled).then_some(thresholds);
        Ok(())
    }

    pub fn remove_joint(&mut self, id: &JointId) -> Result<(), PhysicsError> {
        let record = self
            .joint_records
            .remove(id)
            .ok_or_else(|| PhysicsError::UnknownJoint(id.clone()))?;
        match record.handle {
            JointBackendHandle::Impulse(handle) => {
                self.impulse_joints.remove(handle, true);
            }
            JointBackendHandle::Multibody(handle) => {
                self.multibody_joints.remove(handle, true);
            }
        }
        self.retired_joint_ids.insert(id.clone());
        self.refresh_diagnostics(0.0);
        Ok(())
    }

    pub fn joint_snapshot(&self, id: &JointId) -> Result<JointSnapshot, PhysicsError> {
        let record = self
            .joint_records
            .get(id)
            .ok_or_else(|| PhysicsError::UnknownJoint(id.clone()))?;
        snapshot_joint(
            id.clone(),
            record,
            &self.impulse_joints,
            &self.multibody_joints,
            &self.bodies,
            self.integration_parameters.dt,
        )
    }

    pub fn set_body_mode(
        &mut self,
        id: &BodyId,
        mode: BodyMode,
        wake_up: bool,
    ) -> Result<(), PhysicsError> {
        let record = self.body_record_mut(id)?;
        record.desc.mode = mode;
        let handle = record.handle;
        self.bodies[handle].set_body_type(body_type(mode), wake_up);
        if mode != BodyMode::KinematicPosition {
            self.kinematic_targets.remove(id);
        }
        Ok(())
    }

    pub fn set_body_pose(
        &mut self,
        id: &BodyId,
        pose: Pose,
        wake_up: bool,
    ) -> Result<(), PhysicsError> {
        validate_pose(pose)?;
        let record = self.body_record_mut(id)?;
        record.desc.pose = pose;
        let handle = record.handle;
        self.bodies[handle].set_position(isometry(pose), wake_up);
        self.kinematic_targets.remove(id);
        self.bodies
            .propagate_modified_body_positions_to_colliders(&mut self.colliders);
        self.query_pipeline.update(&self.colliders);
        Ok(())
    }

    pub fn set_next_kinematic_pose(&mut self, id: &BodyId, pose: Pose) -> Result<(), PhysicsError> {
        validate_pose(pose)?;
        let handle = self.body_record(id)?.handle;
        self.bodies[handle].set_next_kinematic_position(isometry(pose));
        Ok(())
    }

    /// Sets or updates a position-kinematic target. Repeated updates preserve
    /// the command's current speed so a moving target does not restart the
    /// acceleration ramp every frame. Commands are sampled before each
    /// physics substep in stable body-ID order.
    pub fn set_bounded_kinematic_target(
        &mut self,
        id: &BodyId,
        target: BoundedKinematicTarget,
    ) -> Result<(), PhysicsError> {
        self.set_bounded_kinematic_target_with_mode(id, target, KinematicTargetMode::Independent)
    }

    /// Sets a bounded position-kinematic target with explicit pose coupling.
    ///
    /// Coupled targets advance translation and rotation by one shared fraction,
    /// selected as the smaller fraction allowed by their independently bounded
    /// speed and acceleration states. The existing target API remains
    /// independent by default.
    pub fn set_bounded_kinematic_target_with_mode(
        &mut self,
        id: &BodyId,
        target: BoundedKinematicTarget,
        mode: KinematicTargetMode,
    ) -> Result<(), PhysicsError> {
        validate_kinematic_target(target)?;
        let record = self.body_record(id)?;
        if record.desc.mode != BodyMode::KinematicPosition {
            return Err(PhysicsError::InvalidValue("bounded kinematic body mode"));
        }
        let existing = self.kinematic_targets.get(id).copied();
        let body = &self.bodies[record.handle];
        self.kinematic_targets.insert(
            id.clone(),
            KinematicTargetState {
                target,
                mode,
                linear_speed_mps: existing
                    .map(|state| state.linear_speed_mps)
                    .unwrap_or_else(|| body.linvel().norm().min(target.maximum_linear_speed_mps)),
                angular_speed_rps: existing
                    .map(|state| state.angular_speed_rps)
                    .unwrap_or_else(|| body.angvel().norm().min(target.maximum_angular_speed_rps)),
            },
        );
        Ok(())
    }

    pub fn clear_bounded_kinematic_target(&mut self, id: &BodyId) -> Result<(), PhysicsError> {
        self.body_record(id)?;
        self.kinematic_targets.remove(id);
        Ok(())
    }

    pub fn set_body_velocity(
        &mut self,
        id: &BodyId,
        linear_mps: [f32; 3],
        angular_rps: [f32; 3],
        wake_up: bool,
    ) -> Result<(), PhysicsError> {
        validate_vector(linear_mps, "linear velocity")?;
        validate_vector(angular_rps, "angular velocity")?;
        let record = self.body_record_mut(id)?;
        record.desc.linear_velocity_mps = linear_mps;
        record.desc.angular_velocity_rps = angular_rps;
        let handle = record.handle;
        self.bodies[handle].set_linvel(vec3(linear_mps), wake_up);
        self.bodies[handle].set_angvel(vec3(angular_rps), wake_up);
        Ok(())
    }

    pub fn add_force(
        &mut self,
        id: &BodyId,
        force_n: [f32; 3],
        wake_up: bool,
    ) -> Result<(), PhysicsError> {
        validate_vector(force_n, "force")?;
        let handle = self.body_record(id)?.handle;
        self.bodies[handle].add_force(vec3(force_n), wake_up);
        Ok(())
    }

    pub fn add_force_at_point(
        &mut self,
        id: &BodyId,
        force_n: [f32; 3],
        point_world_m: [f32; 3],
        wake_up: bool,
    ) -> Result<(), PhysicsError> {
        validate_vector(force_n, "force")?;
        validate_vector(point_world_m, "force point")?;
        let handle = self.body_record(id)?.handle;
        self.bodies[handle].add_force_at_point(
            vec3(force_n),
            Point::from(vec3(point_world_m)),
            wake_up,
        );
        Ok(())
    }

    pub fn apply_impulse(
        &mut self,
        id: &BodyId,
        impulse_ns: [f32; 3],
        wake_up: bool,
    ) -> Result<(), PhysicsError> {
        validate_vector(impulse_ns, "impulse")?;
        let handle = self.body_record(id)?.handle;
        self.bodies[handle].apply_impulse(vec3(impulse_ns), wake_up);
        Ok(())
    }

    pub fn add_torque(
        &mut self,
        id: &BodyId,
        torque_nm: [f32; 3],
        wake_up: bool,
    ) -> Result<(), PhysicsError> {
        validate_vector(torque_nm, "torque")?;
        let handle = self.body_record(id)?.handle;
        self.bodies[handle].add_torque(vec3(torque_nm), wake_up);
        Ok(())
    }

    pub fn apply_torque_impulse(
        &mut self,
        id: &BodyId,
        impulse_nms: [f32; 3],
        wake_up: bool,
    ) -> Result<(), PhysicsError> {
        validate_vector(impulse_nms, "torque impulse")?;
        let handle = self.body_record(id)?.handle;
        self.bodies[handle].apply_torque_impulse(vec3(impulse_nms), wake_up);
        Ok(())
    }

    pub fn wake_up(&mut self, id: &BodyId) -> Result<(), PhysicsError> {
        let handle = self.body_record(id)?.handle;
        self.bodies[handle].wake_up(true);
        Ok(())
    }

    pub fn body_snapshot(&self, id: &BodyId) -> Result<BodySnapshot, PhysicsError> {
        let record = self.body_record(id)?;
        let body = &self.bodies[record.handle];
        Ok(snapshot_body(
            id.clone(),
            body,
            record.desc.mass,
            self.is_body_solver_active(record.handle),
        ))
    }

    /// Returns active contact and sensor pairs without materializing a full
    /// diagnostics snapshot or its debug geometry.
    pub fn contacts(&self) -> Vec<ContactSnapshot> {
        self.contact_snapshots()
    }

    pub fn snapshot(&self) -> PhysicsSnapshot {
        let bodies = self
            .body_records
            .iter()
            .map(|(id, record)| {
                snapshot_body(
                    id.clone(),
                    &self.bodies[record.handle],
                    record.desc.mass,
                    self.is_body_solver_active(record.handle),
                )
            })
            .collect();
        let colliders = self
            .collider_records
            .iter()
            .map(|(id, record)| ColliderSnapshot {
                id: id.clone(),
                body_id: record.body_id.clone(),
                world_pose: pose(self.colliders[record.handle].position()),
                desc: record.desc.clone(),
            })
            .collect();
        let joints = self
            .joint_records
            .iter()
            .filter_map(|(id, record)| {
                snapshot_joint(
                    id.clone(),
                    record,
                    &self.impulse_joints,
                    &self.multibody_joints,
                    &self.bodies,
                    self.integration_parameters.dt,
                )
                .ok()
            })
            .collect();
        PhysicsSnapshot {
            api_version: PHYSICS_API_VERSION,
            step_index: self.step_index,
            simulation_time_sec: self.simulation_time_sec,
            bodies,
            colliders,
            joints,
            contacts: self.contact_snapshots(),
            debug_geometry: self.debug_geometry(),
        }
    }

    pub fn debug_geometry(&self) -> Vec<DebugGeometryRecord> {
        let mut geometry = Vec::new();
        for (id, record) in &self.collider_records {
            let collider = &self.colliders[record.handle];
            append_debug_geometry(
                &mut geometry,
                id,
                &record.body_id,
                collider.position(),
                &record.desc.shape,
                &mut Vec::new(),
                &record.desc,
            );
        }
        geometry
    }

    pub fn diagnostics(&self) -> &PhysicsDiagnostics {
        &self.diagnostics
    }

    pub const fn capabilities() -> PhysicsCapabilities {
        PhysicsCapabilities {
            multibody_joint_chains: true,
            joint_friction: true,
            automatic_joint_break_thresholds: true,
            in_place_joint_motor_updates: true,
            joint_limit_observation: true,
            checkpoint_state_digest: true,
            backend_phase_timings: false,
            exact_resource_bytes: false,
        }
    }

    pub fn canonical_state_digest(&self) -> u64 {
        fnv1a64(format!("{:?}", self.snapshot()).as_bytes())
    }

    pub fn compare_checkpoint(&self, checkpoint: &PhysicsCheckpoint) -> CheckpointComparison {
        let provenance = checkpoint.provenance();
        let checkpoint_version_matches =
            provenance.checkpoint_version == PHYSICS_CHECKPOINT_VERSION;
        let api_version_matches = provenance.physics_api_version == PHYSICS_API_VERSION;
        let implementation_matches = checkpoint.implementation_id == PHYSICS_IMPLEMENTATION_ID;
        let config_matches = provenance.config == self.config;
        let current_state_digest = self.canonical_state_digest();
        let state_matches = current_state_digest == provenance.state_digest;
        let first_divergence = if !checkpoint_version_matches {
            Some("checkpoint_version")
        } else if !api_version_matches {
            Some("physics_api_version")
        } else if !implementation_matches {
            Some("implementation")
        } else if !config_matches {
            Some("config")
        } else if !state_matches {
            Some("state_digest")
        } else {
            None
        };
        CheckpointComparison {
            compatible: checkpoint_version_matches && api_version_matches && implementation_matches,
            checkpoint_version_matches,
            api_version_matches,
            implementation_matches,
            config_matches,
            state_matches,
            current_state_digest,
            checkpoint_state_digest: provenance.state_digest,
            first_divergence,
        }
    }

    /// Applies a deterministic command batch and advances one fixed step. If
    /// any command fails, the complete pre-batch checkpoint is restored and
    /// no step is performed.
    pub fn step_with_commands(&mut self, input: StepInput) -> Result<StepOutput, PhysicsError> {
        let checkpoint = self.checkpoint();
        for command in input.commands {
            if let Err(error) = self.apply_command(command) {
                self.restore(&checkpoint)?;
                return Err(error);
            }
        }
        Ok(self.step())
    }

    pub fn step(&mut self) -> StepOutput {
        let start = Instant::now();
        let substep_dt = self.config.fixed_dt_sec / self.config.substeps as f32;
        self.integration_parameters.dt = substep_dt;
        let mut events = std::mem::take(&mut self.pending_events);
        let mut joint_breaks = Vec::new();
        let mut previous_contacts = self.active_contacts.clone();
        let mut previous_sensors = self.active_sensors.clone();
        for substep_index in 0..self.config.substeps {
            self.advance_bounded_kinematics(substep_dt);
            self.apply_joint_friction(substep_dt);
            let joint_velocities_before = self.joint_velocity_observations();
            self.pipeline.step(
                &self.gravity,
                &self.integration_parameters,
                &mut self.island_manager,
                &mut self.broad_phase,
                &mut self.narrow_phase,
                &mut self.bodies,
                &mut self.colliders,
                &mut self.impulse_joints,
                &mut self.multibody_joints,
                &mut self.ccd_solver,
                Some(&mut self.query_pipeline),
                &(),
                &(),
            );
            self.observe_joint_impulses(&joint_velocities_before);
            let (contacts, sensors) = self.current_pairs();
            events.extend(pair_events(
                &previous_contacts,
                &contacts,
                &previous_sensors,
                &sensors,
                substep_index,
            ));
            previous_contacts = contacts;
            previous_sensors = sensors;
            joint_breaks.extend(self.break_overloaded_joints(substep_dt, substep_index));
        }
        self.step_index += 1;
        for (sequence, event) in events.iter_mut().enumerate() {
            event.step_index = self.step_index;
            event.sequence = sequence as u32;
        }
        for (sequence, event) in joint_breaks.iter_mut().enumerate() {
            event.step_index = self.step_index;
            event.sequence = sequence as u32;
        }
        self.simulation_time_sec += f64::from(self.config.fixed_dt_sec);
        self.active_contacts = previous_contacts;
        self.active_sensors = previous_sensors;
        self.refresh_diagnostics(start.elapsed().as_secs_f64());
        StepOutput {
            snapshot: self.snapshot(),
            events,
            joint_breaks,
            diagnostics: self.diagnostics.clone(),
        }
    }

    fn joint_velocity_observations(&self) -> BTreeMap<JointId, JointVelocityObservation> {
        self.joint_records
            .iter()
            .filter_map(|(id, record)| {
                let body1 = self
                    .body_records
                    .get(&record.desc.body1)
                    .and_then(|record| self.bodies.get(record.handle))?;
                let body2 = self
                    .body_records
                    .get(&record.desc.body2)
                    .and_then(|record| self.bodies.get(record.handle))?;
                let frame1 = body1.position() * isometry(record.desc.local_frame1);
                let frame2 = body2.position() * isometry(record.desc.local_frame2);
                let point1 = Point::from(frame1.translation.vector);
                let point2 = Point::from(frame2.translation.vector);
                Some((
                    id.clone(),
                    JointVelocityObservation {
                        linear: body2.velocity_at_point(&point2) - body1.velocity_at_point(&point1),
                        angular: body2.angvel() - body1.angvel(),
                    },
                ))
            })
            .collect()
    }

    fn observe_joint_impulses(&mut self, before: &BTreeMap<JointId, JointVelocityObservation>) {
        let after = self.joint_velocity_observations();
        let observations = self
            .joint_records
            .iter()
            .filter_map(|(id, record)| {
                let before = before.get(id)?;
                let after = after.get(id)?;
                let body1 = self.body_records.get(&record.desc.body1)?;
                let body2 = self.body_records.get(&record.desc.body2)?;
                let body1 = self.bodies.get(body1.handle)?;
                let body2 = self.bodies.get(body2.handle)?;
                let frame1 = body1.position() * isometry(record.desc.local_frame1);
                let frame2 = body2.position() * isometry(record.desc.local_frame2);
                let point1 = Point::from(frame1.translation.vector);
                let point2 = Point::from(frame2.translation.vector);
                let mut linear_delta = after.linear - before.linear;
                let mut angular_delta = after.angular - before.angular;
                match &record.desc.kind {
                    JointKindDesc::Fixed => {}
                    JointKindDesc::Revolute { axis, .. } => {
                        let axis = frame1.rotation * vec3(*axis).normalize();
                        angular_delta -= axis * angular_delta.dot(&axis);
                    }
                    JointKindDesc::Prismatic { axis, .. } => {
                        let axis = frame1.rotation * vec3(*axis).normalize();
                        linear_delta -= axis * linear_delta.dot(&axis);
                    }
                    JointKindDesc::Spherical { .. } => angular_delta = Vector::zeros(),
                }
                let linear = observed_linear_impulse(body1, body2, point1, point2, linear_delta);
                let angular = observed_angular_impulse(body1, body2, angular_delta);
                Some((
                    id.clone(),
                    [
                        linear.x, linear.y, linear.z, angular.x, angular.y, angular.z,
                    ],
                ))
            })
            .collect::<Vec<_>>();
        for (id, impulse) in observations {
            if let Some(record) = self.joint_records.get_mut(&id) {
                record.observed_impulse = impulse;
            }
        }
    }

    fn apply_joint_friction(&mut self, dt_sec: f32) {
        let configured = self
            .joint_records
            .iter()
            .filter_map(|(id, record)| {
                record
                    .friction_maximum_effort
                    .map(|maximum| (id.clone(), record.desc.clone(), maximum))
            })
            .collect::<Vec<_>>();
        for record in self.joint_records.values_mut() {
            record.friction_applied_impulse = [0.0; 6];
        }
        for (id, desc, maximum_effort) in configured {
            let Some(body1_handle) = self.body_records.get(&desc.body1).map(|body| body.handle)
            else {
                continue;
            };
            let Some(body2_handle) = self.body_records.get(&desc.body2).map(|body| body.handle)
            else {
                continue;
            };
            let body1 = &self.bodies[body1_handle];
            let body2 = &self.bodies[body2_handle];
            let frame1 = body1.position() * isometry(desc.local_frame1);
            let frame2 = body2.position() * isometry(desc.local_frame2);
            let maximum_impulse = maximum_effort * dt_sec;
            let applied = match desc.kind {
                JointKindDesc::Prismatic { axis, .. } => {
                    let axis = frame1.rotation * vec3(axis).normalize();
                    let point1 = Point::from(frame1.translation.vector);
                    let point2 = Point::from(frame2.translation.vector);
                    let relative_speed = (body2.velocity_at_point(&point2)
                        - body1.velocity_at_point(&point1))
                    .dot(&axis);
                    let inverse_mass = point_inverse_mass(body1, point1, axis)
                        + point_inverse_mass(body2, point2, axis);
                    let scalar = resisting_impulse(relative_speed, inverse_mass, maximum_impulse);
                    let impulse = axis * scalar;
                    self.bodies[body1_handle].apply_impulse_at_point(-impulse, point1, true);
                    self.bodies[body2_handle].apply_impulse_at_point(impulse, point2, true);
                    [impulse.x, impulse.y, impulse.z, 0.0, 0.0, 0.0]
                }
                JointKindDesc::Revolute { axis, .. } => {
                    let axis = frame1.rotation * vec3(axis).normalize();
                    let relative_speed = (body2.angvel() - body1.angvel()).dot(&axis);
                    let inverse_inertia =
                        angular_inverse_mass(body1, axis) + angular_inverse_mass(body2, axis);
                    let scalar =
                        resisting_impulse(relative_speed, inverse_inertia, maximum_impulse);
                    let impulse = axis * scalar;
                    self.bodies[body1_handle].apply_torque_impulse(-impulse, true);
                    self.bodies[body2_handle].apply_torque_impulse(impulse, true);
                    [0.0, 0.0, 0.0, impulse.x, impulse.y, impulse.z]
                }
                JointKindDesc::Spherical { .. } => {
                    let relative_velocity = body2.angvel() - body1.angvel();
                    let speed = relative_velocity.norm();
                    if speed <= f32::EPSILON {
                        [0.0; 6]
                    } else {
                        let axis = relative_velocity / speed;
                        let inverse_inertia =
                            angular_inverse_mass(body1, axis) + angular_inverse_mass(body2, axis);
                        let scalar = resisting_impulse(speed, inverse_inertia, maximum_impulse);
                        let impulse = axis * scalar;
                        self.bodies[body1_handle].apply_torque_impulse(-impulse, true);
                        self.bodies[body2_handle].apply_torque_impulse(impulse, true);
                        [0.0, 0.0, 0.0, impulse.x, impulse.y, impulse.z]
                    }
                }
                JointKindDesc::Fixed => [0.0; 6],
            };
            if let Some(record) = self.joint_records.get_mut(&id) {
                record.friction_applied_impulse = applied;
            }
        }
    }

    fn break_overloaded_joints(&mut self, dt_sec: f32, substep_index: u32) -> Vec<JointBreakEvent> {
        let mut overloaded = Vec::new();
        for (id, record) in &self.joint_records {
            let Some(thresholds) = record.break_thresholds else {
                continue;
            };
            let linear_impulse = Vector::new(
                record.observed_impulse[0],
                record.observed_impulse[1],
                record.observed_impulse[2],
            )
            .norm();
            let angular_impulse = Vector::new(
                record.observed_impulse[3],
                record.observed_impulse[4],
                record.observed_impulse[5],
            )
            .norm();
            let force = linear_impulse / dt_sec;
            let torque = angular_impulse / dt_sec;
            let exceeded = [
                (JointBreakCause::Force, force, thresholds.maximum_force_n),
                (
                    JointBreakCause::Torque,
                    torque,
                    thresholds.maximum_torque_nm,
                ),
                (
                    JointBreakCause::LinearImpulse,
                    linear_impulse,
                    thresholds.maximum_linear_impulse_ns,
                ),
                (
                    JointBreakCause::AngularImpulse,
                    angular_impulse,
                    thresholds.maximum_angular_impulse_nms,
                ),
            ]
            .into_iter()
            .find_map(|(cause, observed, threshold)| {
                threshold
                    .filter(|threshold| observed > *threshold)
                    .map(|threshold| (cause, observed, threshold))
            });
            if let Some((cause, observed, threshold)) = exceeded {
                overloaded.push((id.clone(), cause, observed, threshold));
            }
        }
        overloaded
            .into_iter()
            .map(|(joint, cause, observed, threshold)| {
                self.remove_joint(&joint)
                    .expect("overloaded joint was resolved above");
                JointBreakEvent {
                    joint,
                    cause,
                    observed,
                    threshold,
                    step_index: 0,
                    substep_index,
                    sequence: 0,
                }
            })
            .collect()
    }

    pub fn cast_ray(
        &self,
        origin_m: [f32; 3],
        direction: [f32; 3],
        maximum_distance_m: f32,
        solid: bool,
    ) -> Result<Option<RayHit>, PhysicsError> {
        self.cast_ray_filtered(
            origin_m,
            direction,
            maximum_distance_m,
            solid,
            &PhysicsQueryFilter::default(),
        )
    }

    pub fn cast_ray_filtered(
        &self,
        origin_m: [f32; 3],
        direction: [f32; 3],
        maximum_distance_m: f32,
        solid: bool,
        filter: &PhysicsQueryFilter,
    ) -> Result<Option<RayHit>, PhysicsError> {
        if maximum_distance_m < 0.0 || !maximum_distance_m.is_finite() {
            return Err(PhysicsError::InvalidValue("maximum_distance_m"));
        }
        let direction = vec3(direction);
        if direction.norm_squared() <= f32::EPSILON {
            return Err(PhysicsError::InvalidValue("ray direction"));
        }
        let normalized = direction.normalize();
        let ray = Ray::new(Point::from(vec3(origin_m)), normalized);
        let resolved = self.resolve_query_filter(filter)?;
        let predicate = |handle, collider: &Collider| resolved.allows(handle, collider);
        let mut hits = Vec::new();
        self.query_pipeline.intersections_with_ray(
            &self.bodies,
            &self.colliders,
            &ray,
            maximum_distance_m,
            solid,
            QueryFilter::default().predicate(&predicate),
            |handle, hit| {
                if let Some(collider) = self.collider_ids.get(&handle) {
                    let point = ray.point_at(hit.time_of_impact);
                    hits.push(RayHit {
                        collider: collider.clone(),
                        distance_m: hit.time_of_impact,
                        point_m: [point.x, point.y, point.z],
                        normal: [hit.normal.x, hit.normal.y, hit.normal.z],
                    });
                }
                true
            },
        );
        hits.sort_by(|left, right| {
            left.distance_m
                .total_cmp(&right.distance_m)
                .then_with(|| left.collider.cmp(&right.collider))
        });
        Ok(hits.into_iter().next())
    }

    pub fn overlap_shape(
        &self,
        pose: Pose,
        shape: &ColliderShape,
    ) -> Result<Vec<ColliderId>, PhysicsError> {
        self.overlap_shape_filtered(pose, shape, &PhysicsQueryFilter::default())
    }

    pub fn overlap_shape_filtered(
        &self,
        pose: Pose,
        shape: &ColliderShape,
        filter: &PhysicsQueryFilter,
    ) -> Result<Vec<ColliderId>, PhysicsError> {
        validate_pose(pose)?;
        let shape = shared_shape(shape)?;
        let resolved = self.resolve_query_filter(filter)?;
        let predicate = |handle, collider: &Collider| resolved.allows(handle, collider);
        let mut result = Vec::new();
        self.query_pipeline.intersections_with_shape(
            &self.bodies,
            &self.colliders,
            &isometry(pose),
            &*shape,
            QueryFilter::default().predicate(&predicate),
            |handle| {
                if let Some(id) = self.collider_ids.get(&handle) {
                    result.push(id.clone());
                }
                true
            },
        );
        result.sort();
        result.dedup();
        Ok(result)
    }

    pub fn project_point(
        &self,
        point_m: [f32; 3],
        solid: bool,
        filter: &PhysicsQueryFilter,
    ) -> Result<Option<PointProjection>, PhysicsError> {
        if !point_m.iter().all(|value| value.is_finite()) {
            return Err(PhysicsError::InvalidValue("point_m"));
        }
        let resolved = self.resolve_query_filter(filter)?;
        let point = Point::from(vec3(point_m));
        let mut projections = Vec::new();
        for (id, record) in &self.collider_records {
            let collider = &self.colliders[record.handle];
            if !resolved.allows(record.handle, collider) {
                continue;
            }
            let projection = collider
                .shape()
                .project_point(collider.position(), &point, solid);
            projections.push(PointProjection {
                collider: id.clone(),
                point_m: [projection.point.x, projection.point.y, projection.point.z],
                distance_m: (projection.point - point).norm(),
                is_inside: projection.is_inside,
            });
        }
        projections.sort_by(|left, right| {
            left.distance_m
                .total_cmp(&right.distance_m)
                .then_with(|| left.collider.cmp(&right.collider))
        });
        Ok(projections.into_iter().next())
    }

    pub fn cast_shape(
        &self,
        pose: Pose,
        velocity_mps: [f32; 3],
        shape: &ColliderShape,
        maximum_time_sec: f32,
        stop_at_penetration: bool,
        filter: &PhysicsQueryFilter,
    ) -> Result<Option<ShapeCastHit>, PhysicsError> {
        validate_pose(pose)?;
        if !velocity_mps.iter().all(|value| value.is_finite()) {
            return Err(PhysicsError::InvalidValue("velocity_mps"));
        }
        if maximum_time_sec < 0.0 || !maximum_time_sec.is_finite() {
            return Err(PhysicsError::InvalidValue("maximum_time_sec"));
        }
        let moving_shape = shared_shape(shape)?;
        let start = isometry(pose);
        let velocity = vec3(velocity_mps);
        let resolved = self.resolve_query_filter(filter)?;
        let options = ShapeCastOptions {
            max_time_of_impact: maximum_time_sec,
            stop_at_penetration,
            compute_impact_geometry_on_penetration: true,
            ..ShapeCastOptions::default()
        };
        let mut hits = Vec::new();
        for (id, record) in &self.collider_records {
            let collider = &self.colliders[record.handle];
            if !resolved.allows(record.handle, collider) {
                continue;
            }
            let Some(hit) = cast_shapes(
                &start,
                &velocity,
                &*moving_shape,
                collider.position(),
                &Vector::zeros(),
                collider.shape(),
                options,
            )
            .map_err(|_| PhysicsError::InvalidShape("unsupported shape-cast pair".into()))?
            else {
                continue;
            };
            let impact_pose = Isometry::from_parts(
                Translation::from(start.translation.vector + velocity * hit.time_of_impact),
                start.rotation,
            );
            let witness1 = impact_pose * hit.witness1;
            let witness2 = collider.position() * hit.witness2;
            let normal1 = impact_pose.rotation * hit.normal1.into_inner();
            let normal2 = collider.position().rotation * hit.normal2.into_inner();
            hits.push(ShapeCastHit {
                collider: id.clone(),
                time_of_impact_sec: hit.time_of_impact,
                witness1_m: [witness1.x, witness1.y, witness1.z],
                witness2_m: [witness2.x, witness2.y, witness2.z],
                normal1: [normal1.x, normal1.y, normal1.z],
                normal2: [normal2.x, normal2.y, normal2.z],
            });
        }
        hits.sort_by(|left, right| {
            left.time_of_impact_sec
                .total_cmp(&right.time_of_impact_sec)
                .then_with(|| left.collider.cmp(&right.collider))
        });
        Ok(hits.into_iter().next())
    }

    pub fn closest_distance(
        &self,
        pose: Pose,
        shape: &ColliderShape,
        maximum_distance_m: f32,
        filter: &PhysicsQueryFilter,
    ) -> Result<Option<ProximityHit>, PhysicsError> {
        validate_pose(pose)?;
        if maximum_distance_m < 0.0 || !maximum_distance_m.is_finite() {
            return Err(PhysicsError::InvalidValue("maximum_distance_m"));
        }
        let query_shape = shared_shape(shape)?;
        let query_pose = isometry(pose);
        let resolved = self.resolve_query_filter(filter)?;
        let mut hits = Vec::new();
        for (id, record) in &self.collider_records {
            let collider = &self.colliders[record.handle];
            if !resolved.allows(record.handle, collider) {
                continue;
            }
            let distance_m = distance(
                &query_pose,
                &*query_shape,
                collider.position(),
                collider.shape(),
            )
            .map_err(|_| PhysicsError::InvalidShape("unsupported distance-query pair".into()))?;
            if distance_m <= maximum_distance_m {
                hits.push(ProximityHit {
                    collider: id.clone(),
                    distance_m,
                });
            }
        }
        hits.sort_by(|left, right| {
            left.distance_m
                .total_cmp(&right.distance_m)
                .then_with(|| left.collider.cmp(&right.collider))
        });
        Ok(hits.into_iter().next())
    }

    fn resolve_query_filter(
        &self,
        filter: &PhysicsQueryFilter,
    ) -> Result<ResolvedQueryFilter, PhysicsError> {
        let excluded_bodies = filter
            .excluded_bodies
            .iter()
            .map(|id| {
                self.body_records
                    .get(id)
                    .map(|record| record.handle)
                    .ok_or_else(|| PhysicsError::UnknownBody(id.clone()))
            })
            .collect::<Result<HashSet<_>, _>>()?;
        let excluded_colliders = filter
            .excluded_colliders
            .iter()
            .map(|id| {
                self.collider_records
                    .get(id)
                    .map(|record| record.handle)
                    .ok_or_else(|| PhysicsError::UnknownCollider(id.clone()))
            })
            .collect::<Result<HashSet<_>, _>>()?;
        Ok(ResolvedQueryFilter {
            excluded_bodies,
            excluded_colliders,
            groups: filter.groups.map(|groups| {
                InteractionGroups::new(
                    Group::from_bits_truncate(groups.memberships),
                    Group::from_bits_truncate(groups.filter),
                )
            }),
            include_sensors: filter.include_sensors,
        })
    }

    pub fn checkpoint(&self) -> PhysicsCheckpoint {
        let provenance = CheckpointProvenance {
            checkpoint_version: PHYSICS_CHECKPOINT_VERSION,
            physics_api_version: PHYSICS_API_VERSION,
            config: self.config.clone(),
            step_index: self.step_index,
            simulation_time_sec: self.simulation_time_sec,
            state_digest_algorithm: "pge-debug-fnv1a64-v0",
            state_digest: self.canonical_state_digest(),
        };
        PhysicsCheckpoint {
            checkpoint_version: PHYSICS_CHECKPOINT_VERSION,
            implementation_id: PHYSICS_IMPLEMENTATION_ID,
            config: self.config.clone(),
            step_index: self.step_index,
            simulation_time_sec: self.simulation_time_sec,
            backend_state: BackendCheckpoint {
                gravity: self.gravity,
                integration_parameters: self.integration_parameters,
                island_manager: self.island_manager.clone(),
                broad_phase: self.broad_phase.clone(),
                narrow_phase: self.narrow_phase.clone(),
                bodies: self.bodies.clone(),
                colliders: self.colliders.clone(),
                impulse_joints: self.impulse_joints.clone(),
                multibody_joints: self.multibody_joints.clone(),
                ccd_solver: self.ccd_solver.clone(),
                query_pipeline: self.query_pipeline.clone(),
                body_records: self.body_records.clone(),
                collider_records: self.collider_records.clone(),
                joint_records: self.joint_records.clone(),
                collider_ids: self.collider_ids.clone(),
                active_contacts: self.active_contacts.clone(),
                active_sensors: self.active_sensors.clone(),
                pending_events: self.pending_events.clone(),
                retired_body_ids: self.retired_body_ids.clone(),
                retired_collider_ids: self.retired_collider_ids.clone(),
                retired_joint_ids: self.retired_joint_ids.clone(),
                kinematic_targets: self.kinematic_targets.clone(),
                diagnostics: self.diagnostics.clone(),
            },
            provenance,
        }
    }

    pub fn restore(&mut self, checkpoint: &PhysicsCheckpoint) -> Result<(), PhysicsError> {
        if checkpoint.checkpoint_version != PHYSICS_CHECKPOINT_VERSION
            || checkpoint.implementation_id != PHYSICS_IMPLEMENTATION_ID
        {
            return Err(PhysicsError::IncompatibleCheckpoint);
        }
        let state = checkpoint.backend_state.clone();
        self.config = checkpoint.config.clone();
        self.gravity = state.gravity;
        self.integration_parameters = state.integration_parameters;
        self.pipeline = PhysicsPipeline::new();
        self.island_manager = state.island_manager;
        self.broad_phase = state.broad_phase;
        self.narrow_phase = state.narrow_phase;
        self.bodies = state.bodies;
        self.colliders = state.colliders;
        self.impulse_joints = state.impulse_joints;
        self.multibody_joints = state.multibody_joints;
        self.ccd_solver = state.ccd_solver;
        self.query_pipeline = state.query_pipeline;
        self.body_records = state.body_records;
        self.collider_records = state.collider_records;
        self.joint_records = state.joint_records;
        self.collider_ids = state.collider_ids;
        self.active_contacts = state.active_contacts;
        self.active_sensors = state.active_sensors;
        self.pending_events = state.pending_events;
        self.retired_body_ids = state.retired_body_ids;
        self.retired_collider_ids = state.retired_collider_ids;
        self.retired_joint_ids = state.retired_joint_ids;
        self.kinematic_targets = state.kinematic_targets;
        self.step_index = checkpoint.step_index;
        self.simulation_time_sec = checkpoint.simulation_time_sec;
        self.diagnostics = state.diagnostics;
        Ok(())
    }

    fn body_record(&self, id: &BodyId) -> Result<&BodyRecord, PhysicsError> {
        self.body_records
            .get(id)
            .ok_or_else(|| PhysicsError::UnknownBody(id.clone()))
    }

    fn body_record_mut(&mut self, id: &BodyId) -> Result<&mut BodyRecord, PhysicsError> {
        self.body_records
            .get_mut(id)
            .ok_or_else(|| PhysicsError::UnknownBody(id.clone()))
    }

    fn apply_command(&mut self, command: PhysicsCommand) -> Result<(), PhysicsError> {
        match command {
            PhysicsCommand::CreateBody { id, desc } => self.create_body(id, desc),
            PhysicsCommand::RemoveBody { id } => self.remove_body(&id),
            PhysicsCommand::CreateCollider { id, body_id, desc } => {
                self.create_collider(id, &body_id, desc)
            }
            PhysicsCommand::RemoveCollider { id } => self.remove_collider(&id),
            PhysicsCommand::CreateJoint { id, desc } => self.create_joint(id, desc),
            PhysicsCommand::CreateMultibodyJoint { id, desc } => {
                self.create_multibody_joint(id, desc)
            }
            PhysicsCommand::UpdateJoint { id, desc } => self.update_joint(&id, desc),
            PhysicsCommand::RemoveJoint { id } => self.remove_joint(&id),
            PhysicsCommand::SetJointFriction { id, maximum_effort } => {
                self.set_joint_friction(&id, maximum_effort)
            }
            PhysicsCommand::SetJointBreakThresholds { id, thresholds } => {
                self.set_joint_break_thresholds(&id, thresholds)
            }
            PhysicsCommand::SetBodyMode { id, mode, wake_up } => {
                self.set_body_mode(&id, mode, wake_up)
            }
            PhysicsCommand::SetBodyPose { id, pose, wake_up } => {
                self.set_body_pose(&id, pose, wake_up)
            }
            PhysicsCommand::SetBodyVelocity {
                id,
                linear_mps,
                angular_rps,
                wake_up,
            } => self.set_body_velocity(&id, linear_mps, angular_rps, wake_up),
            PhysicsCommand::SetNextKinematicPose { id, pose } => {
                self.set_next_kinematic_pose(&id, pose)
            }
            PhysicsCommand::SetBoundedKinematicTarget { id, target } => {
                self.set_bounded_kinematic_target(&id, target)
            }
            PhysicsCommand::SetBoundedKinematicTargetWithMode { id, target, mode } => {
                self.set_bounded_kinematic_target_with_mode(&id, target, mode)
            }
            PhysicsCommand::ClearBoundedKinematicTarget { id } => {
                self.clear_bounded_kinematic_target(&id)
            }
            PhysicsCommand::AddForce {
                id,
                force_n,
                wake_up,
            } => self.add_force(&id, force_n, wake_up),
            PhysicsCommand::AddForceAtPoint {
                id,
                force_n,
                point_world_m,
                wake_up,
            } => self.add_force_at_point(&id, force_n, point_world_m, wake_up),
            PhysicsCommand::ApplyImpulse {
                id,
                impulse_ns,
                wake_up,
            } => self.apply_impulse(&id, impulse_ns, wake_up),
            PhysicsCommand::AddTorque {
                id,
                torque_nm,
                wake_up,
            } => self.add_torque(&id, torque_nm, wake_up),
            PhysicsCommand::ApplyTorqueImpulse {
                id,
                impulse_nms,
                wake_up,
            } => self.apply_torque_impulse(&id, impulse_nms, wake_up),
            PhysicsCommand::WakeUp { id } => self.wake_up(&id),
        }
    }

    fn current_pairs(&self) -> (BTreeSet<PairKey>, BTreeSet<PairKey>) {
        let contacts = self
            .narrow_phase
            .contact_pairs()
            .filter(|pair| pair.has_any_active_contact)
            .filter_map(|pair| self.pair_key(pair.collider1, pair.collider2))
            .collect();
        let sensors = self
            .narrow_phase
            .intersection_pairs()
            .filter(|(_, _, intersecting)| *intersecting)
            .filter_map(|(first, second, _)| self.pair_key(first, second))
            .collect();
        (contacts, sensors)
    }

    fn pair_key(&self, first: ColliderHandle, second: ColliderHandle) -> Option<PairKey> {
        Some(PairKey::new(
            self.collider_ids.get(&first)?.clone(),
            self.collider_ids.get(&second)?.clone(),
        ))
    }

    fn contact_snapshots(&self) -> Vec<ContactSnapshot> {
        let mut contacts = self
            .narrow_phase
            .contact_pairs()
            .filter(|pair| pair.has_any_active_contact)
            .filter_map(|pair| {
                let backend_first = self.collider_ids.get(&pair.collider1)?.clone();
                let backend_second = self.collider_ids.get(&pair.collider2)?.clone();
                let flipped = backend_first > backend_second;
                let mut impulse = pair.total_impulse();
                let key = if flipped {
                    impulse = -impulse;
                    PairKey(backend_second, backend_first)
                } else {
                    PairKey(backend_first, backend_second)
                };
                let collider1 = &self.colliders[pair.collider1];
                let collider2 = &self.colliders[pair.collider2];
                let mut manifolds = pair
                    .manifolds
                    .iter()
                    .map(|manifold| {
                        let mut normal = manifold.data.normal;
                        if flipped {
                            normal = -normal;
                        }
                        let mut points = manifold
                            .points
                            .iter()
                            .map(|contact| {
                                let backend_point1 = collider1.position() * contact.local_p1;
                                let backend_point2 = collider2.position() * contact.local_p2;
                                let (point1, point2, handle1, handle2) = if flipped {
                                    (
                                        backend_point2,
                                        backend_point1,
                                        pair.collider2,
                                        pair.collider1,
                                    )
                                } else {
                                    (
                                        backend_point1,
                                        backend_point2,
                                        pair.collider1,
                                        pair.collider2,
                                    )
                                };
                                let relative_velocity = self
                                    .collider_velocity_at_point(handle2, &point2)
                                    - self.collider_velocity_at_point(handle1, &point1);
                                ContactPointSnapshot {
                                    point1_m: [point1.x, point1.y, point1.z],
                                    point2_m: [point2.x, point2.y, point2.z],
                                    distance_m: contact.dist,
                                    penetration_depth_m: (-contact.dist).max(0.0),
                                    relative_velocity_mps: [
                                        relative_velocity.x,
                                        relative_velocity.y,
                                        relative_velocity.z,
                                    ],
                                    normal_impulse_ns: contact.data.impulse,
                                    tangent_impulse_magnitude_ns: finite_optional(
                                        contact.data.tangent_impulse.norm(),
                                    ),
                                }
                            })
                            .collect::<Vec<_>>();
                        points.sort_by(compare_contact_points);
                        let (subshape1, subshape2) = if flipped {
                            (manifold.subshape2, manifold.subshape1)
                        } else {
                            (manifold.subshape1, manifold.subshape2)
                        };
                        ContactManifoldSnapshot {
                            normal_on_collider1: [normal.x, normal.y, normal.z],
                            subshape1,
                            subshape2,
                            points,
                        }
                    })
                    .collect::<Vec<_>>();
                manifolds.sort_by(compare_contact_manifolds);
                let normal = manifolds
                    .first()
                    .map(|manifold| manifold.normal_on_collider1)
                    .unwrap_or([0.0; 3]);
                Some(ContactSnapshot {
                    collider1: key.0,
                    collider2: key.1,
                    sensor: false,
                    normal,
                    total_impulse_ns: [impulse.x, impulse.y, impulse.z],
                    total_impulse_magnitude_ns: pair.total_impulse_magnitude(),
                    manifolds,
                })
            })
            .collect::<Vec<_>>();
        contacts.extend(self.active_sensors.iter().map(|key| ContactSnapshot {
            collider1: key.0.clone(),
            collider2: key.1.clone(),
            sensor: true,
            normal: [0.0; 3],
            total_impulse_ns: [0.0; 3],
            total_impulse_magnitude_ns: 0.0,
            manifolds: Vec::new(),
        }));
        contacts.sort_by(|a, b| (&a.collider1, &a.collider2).cmp(&(&b.collider1, &b.collider2)));
        contacts
    }

    fn collider_velocity_at_point(
        &self,
        handle: ColliderHandle,
        point: &Point<Real>,
    ) -> Vector<Real> {
        self.colliders[handle]
            .parent()
            .and_then(|parent| self.bodies.get(parent))
            .map_or_else(Vector::zeros, |body| body.velocity_at_point(point))
    }

    fn discard_removed_pairs(&mut self) {
        let present = self
            .collider_records
            .keys()
            .cloned()
            .collect::<BTreeSet<_>>();
        self.active_contacts
            .retain(|pair| present.contains(&pair.0) && present.contains(&pair.1));
        self.active_sensors
            .retain(|pair| present.contains(&pair.0) && present.contains(&pair.1));
    }

    fn queue_pair_removal_events(&mut self, collider_id: &ColliderId) {
        let stopped_contacts = self
            .active_contacts
            .iter()
            .filter(|pair| pair.0 == *collider_id || pair.1 == *collider_id)
            .cloned()
            .collect::<Vec<_>>();
        let stopped_sensors = self
            .active_sensors
            .iter()
            .filter(|pair| pair.0 == *collider_id || pair.1 == *collider_id)
            .cloned()
            .collect::<Vec<_>>();
        append_events(
            &mut self.pending_events,
            stopped_contacts.iter(),
            PhysicsEventKind::ContactStopped,
            0,
        );
        append_events(
            &mut self.pending_events,
            stopped_sensors.iter(),
            PhysicsEventKind::SensorStopped,
            0,
        );
    }

    fn advance_bounded_kinematics(&mut self, dt_sec: f32) {
        let ids = self.kinematic_targets.keys().cloned().collect::<Vec<_>>();
        for id in ids {
            let Some(mut state) = self.kinematic_targets.get(&id).copied() else {
                continue;
            };
            let Some(record) = self.body_records.get(&id) else {
                self.kinematic_targets.remove(&id);
                continue;
            };
            let current = *self.bodies[record.handle].position();
            let target = isometry(state.target.pose);
            let translation_delta = target.translation.vector - current.translation.vector;
            let distance = translation_delta.norm();
            let angle = current.rotation.angle_to(&target.rotation);
            state.linear_speed_mps = bounded_speed(
                state.linear_speed_mps,
                distance,
                state.target.maximum_linear_speed_mps,
                state.target.maximum_linear_acceleration_mps2,
                dt_sec,
            );
            state.angular_speed_rps = bounded_speed(
                state.angular_speed_rps,
                angle,
                state.target.maximum_angular_speed_rps,
                state.target.maximum_angular_acceleration_rps2,
                dt_sec,
            );
            let mut translation_fraction = if distance <= 1.0e-7 {
                1.0
            } else {
                (state.linear_speed_mps * dt_sec / distance).min(1.0)
            };
            let mut rotation_fraction = if angle <= 1.0e-7 {
                1.0
            } else {
                (state.angular_speed_rps * dt_sec / angle).min(1.0)
            };
            if state.mode == KinematicTargetMode::CoupledPose {
                let coupled_fraction = translation_fraction.min(rotation_fraction);
                translation_fraction = coupled_fraction;
                rotation_fraction = coupled_fraction;
                state.linear_speed_mps = if distance <= 1.0e-7 {
                    0.0
                } else {
                    distance * coupled_fraction / dt_sec
                };
                state.angular_speed_rps = if angle <= 1.0e-7 {
                    0.0
                } else {
                    angle * coupled_fraction / dt_sec
                };
            }
            let next = Isometry::from_parts(
                Translation::from(
                    current.translation.vector + translation_delta * translation_fraction,
                ),
                current.rotation.slerp(&target.rotation, rotation_fraction),
            );
            self.bodies[record.handle].set_next_kinematic_position(next);
            if translation_fraction >= 1.0 && rotation_fraction >= 1.0 {
                self.kinematic_targets.remove(&id);
            } else {
                self.kinematic_targets.insert(id, state);
            }
        }
    }

    fn refresh_diagnostics(&mut self, last_step_seconds: f64) {
        let contact_manifold_count = self
            .narrow_phase
            .contact_pairs()
            .filter(|pair| pair.has_any_active_contact)
            .map(|pair| pair.manifolds.len())
            .sum();
        let contact_point_count = self
            .narrow_phase
            .contact_pairs()
            .filter(|pair| pair.has_any_active_contact)
            .flat_map(|pair| &pair.manifolds)
            .map(|manifold| manifold.points.len())
            .sum();
        self.diagnostics = PhysicsDiagnostics {
            body_count: self.body_records.len(),
            collider_count: self.collider_records.len(),
            joint_count: self.joint_records.len(),
            active_contact_count: self.active_contacts.len(),
            active_sensor_count: self.active_sensors.len(),
            sleeping_body_count: self
                .body_records
                .values()
                .filter(|record| self.bodies[record.handle].is_sleeping())
                .count(),
            active_dynamic_body_count: self.island_manager.active_dynamic_bodies().len(),
            active_kinematic_body_count: self.island_manager.active_kinematic_bodies().len(),
            ccd_enabled_body_count: self
                .body_records
                .values()
                .filter(|record| self.bodies[record.handle].is_ccd_enabled())
                .count(),
            ccd_active_body_count: self
                .body_records
                .values()
                .filter(|record| self.bodies[record.handle].is_ccd_active())
                .count(),
            contact_manifold_count,
            contact_point_count,
            sleeping_island_count: None,
            estimated_resource_bytes: None,
            backend_phase_seconds: None,
            last_step_seconds,
        };
    }

    fn is_body_solver_active(&self, handle: RigidBodyHandle) -> bool {
        self.island_manager
            .active_dynamic_bodies()
            .contains(&handle)
            || self
                .island_manager
                .active_kinematic_bodies()
                .contains(&handle)
    }
}

impl Default for PhysicsWorld {
    fn default() -> Self {
        Self::new(PhysicsConfig::default()).expect("default physics configuration is valid")
    }
}

fn validate_config(config: &PhysicsConfig) -> Result<(), PhysicsError> {
    if !config.gravity_mps2.iter().all(|value| value.is_finite()) {
        return Err(PhysicsError::InvalidValue("gravity_mps2"));
    }
    if !config.fixed_dt_sec.is_finite() || config.fixed_dt_sec <= 0.0 {
        return Err(PhysicsError::InvalidValue("fixed_dt_sec"));
    }
    if config.substeps == 0 {
        return Err(PhysicsError::InvalidValue("substeps"));
    }
    Ok(())
}

fn validate_body(desc: &BodyDesc) -> Result<(), PhysicsError> {
    validate_pose(desc.pose)?;
    if !desc
        .linear_velocity_mps
        .iter()
        .chain(desc.angular_velocity_rps.iter())
        .all(|value| value.is_finite())
    {
        return Err(PhysicsError::InvalidValue("body velocity"));
    }
    if !desc.gravity_scale.is_finite()
        || !desc.linear_damping.is_finite()
        || desc.linear_damping < 0.0
        || !desc.angular_damping.is_finite()
        || desc.angular_damping < 0.0
    {
        return Err(PhysicsError::InvalidValue("body scalar"));
    }
    if let Some(mass) = desc.mass {
        if !mass.mass_kg.is_finite()
            || mass.mass_kg <= 0.0
            || !mass.center_of_mass_m.iter().all(|value| value.is_finite())
            || !mass
                .principal_inertia_kg_m2
                .iter()
                .all(|value| value.is_finite() && *value > 0.0)
        {
            return Err(PhysicsError::InvalidValue("mass properties"));
        }
        validate_quaternion(mass.principal_inertia_frame_xyzw)?;
        if let Some(tensor) = mass.inertia_tensor_kg_m2 {
            let symmetric = (tensor[0][1] - tensor[1][0]).abs() <= 1.0e-5
                && (tensor[0][2] - tensor[2][0]).abs() <= 1.0e-5
                && (tensor[1][2] - tensor[2][1]).abs() <= 1.0e-5;
            let leading_minor_2 = tensor[0][0] * tensor[1][1] - tensor[0][1] * tensor[1][0];
            let determinant = tensor[0][0]
                * (tensor[1][1] * tensor[2][2] - tensor[1][2] * tensor[2][1])
                - tensor[0][1] * (tensor[1][0] * tensor[2][2] - tensor[1][2] * tensor[2][0])
                + tensor[0][2] * (tensor[1][0] * tensor[2][1] - tensor[1][1] * tensor[2][0]);
            if !tensor.iter().flatten().all(|value| value.is_finite())
                || !symmetric
                || tensor[0][0] <= 0.0
                || leading_minor_2 <= 0.0
                || determinant <= 0.0
            {
                return Err(PhysicsError::InvalidValue("inertia tensor"));
            }
        }
    }
    Ok(())
}

fn validate_kinematic_target(target: BoundedKinematicTarget) -> Result<(), PhysicsError> {
    validate_pose(target.pose)?;
    if [
        target.maximum_linear_speed_mps,
        target.maximum_angular_speed_rps,
        target.maximum_linear_acceleration_mps2,
        target.maximum_angular_acceleration_rps2,
    ]
    .iter()
    .all(|value| value.is_finite() && *value > 0.0)
    {
        Ok(())
    } else {
        Err(PhysicsError::InvalidValue("bounded kinematic limits"))
    }
}

fn validate_pose(pose: Pose) -> Result<(), PhysicsError> {
    if pose.translation.iter().all(|value| value.is_finite())
        && validate_quaternion(pose.rotation_xyzw).is_ok()
    {
        Ok(())
    } else {
        Err(PhysicsError::InvalidValue("pose"))
    }
}

fn validate_quaternion(rotation_xyzw: [f32; 4]) -> Result<(), PhysicsError> {
    let norm_squared = rotation_xyzw.iter().map(|value| value * value).sum::<f32>();
    if rotation_xyzw.iter().all(|value| value.is_finite()) && norm_squared > f32::EPSILON {
        Ok(())
    } else {
        Err(PhysicsError::InvalidValue("rotation quaternion"))
    }
}

fn validate_vector(value: [f32; 3], name: &'static str) -> Result<(), PhysicsError> {
    if value.iter().all(|component| component.is_finite()) {
        Ok(())
    } else {
        Err(PhysicsError::InvalidValue(name))
    }
}

fn validate_collider(desc: &ColliderDesc) -> Result<(), PhysicsError> {
    validate_pose(desc.pose)?;
    let material = desc.material;
    if !material.friction.is_finite()
        || material.friction < 0.0
        || !material.restitution.is_finite()
        || !(0.0..=1.0).contains(&material.restitution)
        || !material.density_kg_m3.is_finite()
        || material.density_kg_m3 < 0.0
        || !material.contact_skin_m.is_finite()
        || material.contact_skin_m < 0.0
    {
        return Err(PhysicsError::InvalidValue("collider material"));
    }
    validate_shape(&desc.shape)
}

fn validate_joint(desc: &JointDesc) -> Result<(), PhysicsError> {
    validate_pose(desc.local_frame1)?;
    validate_pose(desc.local_frame2)?;
    if desc.body1 == desc.body2 {
        return Err(PhysicsError::InvalidValue("joint bodies"));
    }
    let validate_axis = |axis: [f32; 3]| {
        if axis.iter().all(|value| value.is_finite()) && vec3(axis).norm_squared() > 1.0e-8 {
            Ok(())
        } else {
            Err(PhysicsError::InvalidValue("joint axis"))
        }
    };
    match &desc.kind {
        JointKindDesc::Fixed => Ok(()),
        JointKindDesc::Revolute {
            axis,
            limits,
            motor,
        }
        | JointKindDesc::Prismatic {
            axis,
            limits,
            motor,
        } => {
            validate_axis(*axis)?;
            validate_joint_limit(*limits)?;
            validate_joint_motor(*motor)
        }
        JointKindDesc::Spherical { limits, motors } => {
            for limit in limits {
                validate_joint_limit(*limit)?;
            }
            for motor in motors {
                validate_joint_motor(*motor)?;
            }
            Ok(())
        }
    }
}

fn validate_joint_limit(limit: Option<JointLimitDesc>) -> Result<(), PhysicsError> {
    if let Some(limit) = limit {
        if !limit.minimum.is_finite() || !limit.maximum.is_finite() || limit.minimum > limit.maximum
        {
            return Err(PhysicsError::InvalidValue("joint limits"));
        }
    }
    Ok(())
}

fn validate_joint_motor(motor: Option<JointMotorDesc>) -> Result<(), PhysicsError> {
    if let Some(motor) = motor {
        if ![
            motor.target_position,
            motor.target_velocity,
            motor.stiffness,
            motor.damping,
            motor.maximum_force,
        ]
        .iter()
        .all(|value| value.is_finite())
            || motor.stiffness < 0.0
            || motor.damping < 0.0
            || motor.maximum_force < 0.0
        {
            return Err(PhysicsError::InvalidValue("joint motor"));
        }
    }
    Ok(())
}

fn validate_shape(shape: &ColliderShape) -> Result<(), PhysicsError> {
    let positive = |value: f32| value.is_finite() && value > 0.0;
    match shape {
        ColliderShape::Box { size } => size
            .iter()
            .all(|value| positive(*value))
            .then_some(())
            .ok_or_else(|| PhysicsError::InvalidShape("box size must be positive".into())),
        ColliderShape::Sphere { radius } => positive(*radius)
            .then_some(())
            .ok_or_else(|| PhysicsError::InvalidShape("sphere radius must be positive".into())),
        ColliderShape::CapsuleY {
            half_height,
            radius,
        }
        | ColliderShape::CylinderY {
            half_height,
            radius,
        }
        | ColliderShape::ConeY {
            half_height,
            radius,
        } => (positive(*half_height) && positive(*radius))
            .then_some(())
            .ok_or_else(|| PhysicsError::InvalidShape("shape dimensions must be positive".into())),
        ColliderShape::ConvexHull { points } => {
            if points.len() >= 4 && points.iter().flatten().all(|value| value.is_finite()) {
                Ok(())
            } else {
                Err(PhysicsError::InvalidShape(
                    "convex hull needs at least four finite points".into(),
                ))
            }
        }
        ColliderShape::TriangleMesh { vertices, indices } => {
            if vertices.len() >= 3
                && !indices.is_empty()
                && vertices.iter().flatten().all(|value| value.is_finite())
                && indices
                    .iter()
                    .flatten()
                    .all(|index| (*index as usize) < vertices.len())
            {
                Ok(())
            } else {
                Err(PhysicsError::InvalidShape("invalid triangle mesh".into()))
            }
        }
        ColliderShape::HeightField {
            rows,
            columns,
            heights,
            scale,
        } => {
            let sample_count = rows.checked_mul(*columns);
            if *rows >= 2
                && *columns >= 2
                && sample_count == Some(heights.len())
                && heights.iter().all(|height| height.is_finite())
                && scale.iter().all(|value| positive(*value))
            {
                Ok(())
            } else {
                Err(PhysicsError::InvalidShape(
                    "heightfield needs a finite grid of at least 2x2 and positive scale".into(),
                ))
            }
        }
        ColliderShape::Compound { children } => {
            if children.is_empty() {
                return Err(PhysicsError::InvalidShape("empty compound".into()));
            }
            for child in children {
                validate_pose(child.pose)?;
                validate_shape(&child.shape)?;
            }
            Ok(())
        }
    }
}

fn body_builder(desc: &BodyDesc) -> RigidBodyBuilder {
    let mut builder = RigidBodyBuilder::new(body_type(desc.mode))
        .position(isometry(desc.pose))
        .linvel(vec3(desc.linear_velocity_mps))
        .angvel(vec3(desc.angular_velocity_rps))
        .linear_damping(desc.linear_damping)
        .angular_damping(desc.angular_damping)
        .gravity_scale(desc.gravity_scale)
        .ccd_enabled(desc.ccd_enabled)
        .enabled_translations(
            !desc.lock_translation[0],
            !desc.lock_translation[1],
            !desc.lock_translation[2],
        )
        .enabled_rotations(
            !desc.lock_rotation[0],
            !desc.lock_rotation[1],
            !desc.lock_rotation[2],
        )
        .sleeping(desc.sleeping);
    if let Some(mass) = desc.mass {
        let local_com = Point::from(vec3(mass.center_of_mass_m));
        let mass_kg = mass.mass_kg;
        let properties = if let Some(tensor) = mass.inertia_tensor_kg_m2 {
            MassProperties::with_inertia_matrix(
                local_com,
                mass_kg,
                Matrix3::from_row_slice(&[
                    tensor[0][0],
                    tensor[0][1],
                    tensor[0][2],
                    tensor[1][0],
                    tensor[1][1],
                    tensor[1][2],
                    tensor[2][0],
                    tensor[2][1],
                    tensor[2][2],
                ]),
            )
        } else {
            let [x, y, z, w] = mass.principal_inertia_frame_xyzw;
            MassProperties::with_principal_inertia_frame(
                local_com,
                mass_kg,
                vec3(mass.principal_inertia_kg_m2),
                UnitQuaternion::new_normalize(Quaternion::new(w, x, y, z)),
            )
        };
        builder = builder.additional_mass_properties(properties);
    }
    builder
}

fn joint_data(desc: &JointDesc) -> Result<GenericJoint, PhysicsError> {
    validate_joint(desc)?;
    let locked_axes = match desc.kind {
        JointKindDesc::Fixed => JointAxesMask::LOCKED_FIXED_AXES,
        JointKindDesc::Revolute { .. } => JointAxesMask::LOCKED_REVOLUTE_AXES,
        JointKindDesc::Prismatic { .. } => JointAxesMask::LOCKED_PRISMATIC_AXES,
        JointKindDesc::Spherical { .. } => JointAxesMask::LOCKED_SPHERICAL_AXES,
    };
    let mut data = GenericJointBuilder::new(locked_axes)
        .contacts_enabled(desc.contacts_enabled)
        .local_frame1(isometry(desc.local_frame1))
        .local_frame2(isometry(desc.local_frame2))
        .build();
    match &desc.kind {
        JointKindDesc::Fixed => {}
        JointKindDesc::Revolute {
            axis,
            limits,
            motor,
        } => {
            set_joint_axes(&mut data, desc, *axis);
            apply_joint_axis_config(&mut data, JointAxis::AngX, *limits, *motor);
        }
        JointKindDesc::Prismatic {
            axis,
            limits,
            motor,
        } => {
            set_joint_axes(&mut data, desc, *axis);
            apply_joint_axis_config(&mut data, JointAxis::LinX, *limits, *motor);
        }
        JointKindDesc::Spherical { limits, motors } => {
            for (index, axis) in [JointAxis::AngX, JointAxis::AngY, JointAxis::AngZ]
                .into_iter()
                .enumerate()
            {
                apply_joint_axis_config(&mut data, axis, limits[index], motors[index]);
            }
        }
    }
    Ok(data)
}

fn set_joint_axes(data: &mut GenericJoint, desc: &JointDesc, axis: [f32; 3]) {
    let axis = vec3(axis).normalize();
    let axis1 = isometry(desc.local_frame1).rotation * axis;
    let axis2 = isometry(desc.local_frame2).rotation * axis;
    data.set_local_axis1(UnitVector::new_normalize(axis1));
    data.set_local_axis2(UnitVector::new_normalize(axis2));
}

fn apply_joint_axis_config(
    data: &mut GenericJoint,
    axis: JointAxis,
    limit: Option<JointLimitDesc>,
    motor: Option<JointMotorDesc>,
) {
    if let Some(limit) = limit {
        data.set_limits(axis, [limit.minimum, limit.maximum]);
    }
    if let Some(motor) = motor {
        data.set_motor(
            axis,
            motor.target_position,
            motor.target_velocity,
            motor.stiffness,
            motor.damping,
        );
        data.set_motor_max_force(axis, motor.maximum_force);
    }
}

fn collider_builder(
    desc: &ColliderDesc,
    parent_has_explicit_mass: bool,
) -> Result<ColliderBuilder, PhysicsError> {
    validate_collider(desc)?;
    let groups = InteractionGroups::new(
        Group::from_bits_truncate(desc.collision_memberships),
        Group::from_bits_truncate(desc.collision_filter),
    );
    Ok(ColliderBuilder::new(shared_shape(&desc.shape)?)
        .position(isometry(desc.pose))
        .friction(desc.material.friction)
        .restitution(desc.material.restitution)
        .density(if parent_has_explicit_mass {
            0.0
        } else {
            desc.material.density_kg_m3
        })
        .contact_skin(desc.material.contact_skin_m)
        .collision_groups(groups)
        .sensor(desc.sensor))
}

fn shared_shape(shape: &ColliderShape) -> Result<SharedShape, PhysicsError> {
    match shape {
        ColliderShape::Box { size } => Ok(SharedShape::cuboid(
            size[0] * 0.5,
            size[1] * 0.5,
            size[2] * 0.5,
        )),
        ColliderShape::Sphere { radius } => Ok(SharedShape::ball(*radius)),
        ColliderShape::CapsuleY {
            half_height,
            radius,
        } => Ok(SharedShape::capsule_y(*half_height, *radius)),
        ColliderShape::CylinderY {
            half_height,
            radius,
        } => Ok(SharedShape::cylinder(*half_height, *radius)),
        ColliderShape::ConeY {
            half_height,
            radius,
        } => Ok(SharedShape::cone(*half_height, *radius)),
        ColliderShape::ConvexHull { points } => SharedShape::convex_hull(
            &points
                .iter()
                .map(|point| Point::from(vec3(*point)))
                .collect::<Vec<_>>(),
        )
        .ok_or_else(|| PhysicsError::InvalidShape("convex hull has no volume".into())),
        ColliderShape::TriangleMesh { vertices, indices } => SharedShape::trimesh(
            vertices
                .iter()
                .map(|point| Point::from(vec3(*point)))
                .collect(),
            indices.clone(),
        )
        .map_err(|error| PhysicsError::InvalidShape(error.to_string())),
        ColliderShape::HeightField {
            rows,
            columns,
            heights,
            scale,
        } => Ok(SharedShape::heightfield(
            DMatrix::from_row_slice(*rows, *columns, heights),
            vec3(*scale),
        )),
        ColliderShape::Compound { children } => {
            if children.is_empty() {
                return Err(PhysicsError::InvalidShape("empty compound".into()));
            }
            children
                .iter()
                .map(|child| Ok((isometry(child.pose), shared_shape(&child.shape)?)))
                .collect::<Result<Vec<_>, PhysicsError>>()
                .map(SharedShape::compound)
        }
    }
}

fn bounded_speed(
    previous_speed: f32,
    remaining_distance: f32,
    maximum_speed: f32,
    maximum_acceleration: f32,
    dt_sec: f32,
) -> f32 {
    let acceleration_step = maximum_acceleration * dt_sec;
    let stopping_limited = ((acceleration_step * acceleration_step
        + 2.0 * maximum_acceleration * remaining_distance)
        .sqrt()
        - acceleration_step)
        .max(0.0);
    let desired = maximum_speed.min(stopping_limited);
    if desired >= previous_speed {
        desired.min(previous_speed + acceleration_step)
    } else {
        desired.max((previous_speed - acceleration_step).max(0.0))
    }
}

fn body_type(mode: BodyMode) -> RigidBodyType {
    match mode {
        BodyMode::Static => RigidBodyType::Fixed,
        BodyMode::Dynamic => RigidBodyType::Dynamic,
        BodyMode::KinematicPosition => RigidBodyType::KinematicPositionBased,
        BodyMode::KinematicVelocity => RigidBodyType::KinematicVelocityBased,
    }
}

fn body_mode(body_type: RigidBodyType) -> BodyMode {
    match body_type {
        RigidBodyType::Fixed => BodyMode::Static,
        RigidBodyType::Dynamic => BodyMode::Dynamic,
        RigidBodyType::KinematicPositionBased => BodyMode::KinematicPosition,
        RigidBodyType::KinematicVelocityBased => BodyMode::KinematicVelocity,
    }
}

fn isometry(pose: Pose) -> Isometry<Real> {
    let [x, y, z, w] = pose.rotation_xyzw;
    Isometry::from_parts(
        Translation::from(vec3(pose.translation)),
        UnitQuaternion::new_normalize(Quaternion::new(w, x, y, z)),
    )
}

fn pose(isometry: &Isometry<Real>) -> Pose {
    let quaternion = isometry.rotation.quaternion();
    Pose {
        translation: [
            isometry.translation.x,
            isometry.translation.y,
            isometry.translation.z,
        ],
        rotation_xyzw: [quaternion.i, quaternion.j, quaternion.k, quaternion.w],
    }
}

fn append_debug_geometry(
    output: &mut Vec<DebugGeometryRecord>,
    collider_id: &ColliderId,
    body_id: &BodyId,
    world_pose: &Isometry<Real>,
    shape: &ColliderShape,
    child_path: &mut Vec<u32>,
    desc: &ColliderDesc,
) {
    if let ColliderShape::Compound { children } = shape {
        for (index, child) in children.iter().enumerate() {
            child_path.push(index as u32);
            let child_pose = world_pose * isometry(child.pose);
            append_debug_geometry(
                output,
                collider_id,
                body_id,
                &child_pose,
                &child.shape,
                child_path,
                desc,
            );
            child_path.pop();
        }
    } else {
        output.push(DebugGeometryRecord {
            collider: collider_id.clone(),
            body_id: body_id.clone(),
            child_path: child_path.clone(),
            world_pose: pose(world_pose),
            shape: shape.clone(),
            sensor: desc.sensor,
            collision_memberships: desc.collision_memberships,
            collision_filter: desc.collision_filter,
        });
    }
}

fn vec3(value: [f32; 3]) -> Vector<Real> {
    Vector::new(value[0], value[1], value[2])
}

fn snapshot_body(
    id: BodyId,
    body: &RigidBody,
    authored_mass: Option<MassPropertiesDesc>,
    solver_active: bool,
) -> BodySnapshot {
    let mass_properties = body.mass_properties();
    let local = &mass_properties.local_mprops;
    let inertia = local.reconstruct_inertia_matrix();
    let effective_mass = mass_properties.effective_mass();
    BodySnapshot {
        id,
        mode: body_mode(body.body_type()),
        pose: pose(body.position()),
        linear_velocity_mps: [body.linvel().x, body.linvel().y, body.linvel().z],
        angular_velocity_rps: [body.angvel().x, body.angvel().y, body.angvel().z],
        sleeping: body.is_sleeping(),
        solver_active,
        ccd_enabled: body.is_ccd_enabled(),
        ccd_active: body.is_ccd_active(),
        authored_mass,
        effective_mass: EffectiveMassProperties {
            mass_kg: mass_properties.mass(),
            center_of_mass_m: [local.local_com.x, local.local_com.y, local.local_com.z],
            inertia_tensor_kg_m2: [
                [inertia[(0, 0)], inertia[(0, 1)], inertia[(0, 2)]],
                [inertia[(1, 0)], inertia[(1, 1)], inertia[(1, 2)]],
                [inertia[(2, 0)], inertia[(2, 1)], inertia[(2, 2)]],
            ],
            effective_translation_mass_kg: [effective_mass.x, effective_mass.y, effective_mass.z],
        },
    }
}

fn angular_inverse_mass(body: &RigidBody, axis: Vector<Real>) -> f32 {
    let inverse_inertia_sqrt = body.mass_properties().effective_world_inv_inertia_sqrt;
    axis.dot(&(inverse_inertia_sqrt * (inverse_inertia_sqrt * axis)))
}

fn point_inverse_mass(body: &RigidBody, point: Point<Real>, axis: Vector<Real>) -> f32 {
    let mass_properties = body.mass_properties();
    let offset = point - mass_properties.world_com;
    let angular_axis = offset.cross(&axis);
    axis.dot(&mass_properties.effective_inv_mass.component_mul(&axis))
        + angular_inverse_mass(body, angular_axis)
}

fn resisting_impulse(relative_speed: f32, inverse_mass: f32, maximum_impulse: f32) -> f32 {
    if relative_speed.abs() <= f32::EPSILON || inverse_mass <= f32::EPSILON {
        0.0
    } else {
        (-relative_speed / inverse_mass).clamp(-maximum_impulse, maximum_impulse)
    }
}

fn observed_linear_impulse(
    body1: &RigidBody,
    body2: &RigidBody,
    point1: Point<Real>,
    point2: Point<Real>,
    velocity_delta: Vector<Real>,
) -> Vector<Real> {
    let magnitude = velocity_delta.norm();
    if magnitude <= f32::EPSILON {
        return Vector::zeros();
    }
    let axis = velocity_delta / magnitude;
    let inverse_mass =
        point_inverse_mass(body1, point1, axis) + point_inverse_mass(body2, point2, axis);
    if inverse_mass <= f32::EPSILON {
        Vector::zeros()
    } else {
        axis * (magnitude / inverse_mass)
    }
}

fn observed_angular_impulse(
    body1: &RigidBody,
    body2: &RigidBody,
    velocity_delta: Vector<Real>,
) -> Vector<Real> {
    let magnitude = velocity_delta.norm();
    if magnitude <= f32::EPSILON {
        return Vector::zeros();
    }
    let axis = velocity_delta / magnitude;
    let inverse_inertia = angular_inverse_mass(body1, axis) + angular_inverse_mass(body2, axis);
    if inverse_inertia <= f32::EPSILON {
        Vector::zeros()
    } else {
        axis * (magnitude / inverse_inertia)
    }
}

fn snapshot_joint(
    id: JointId,
    record: &JointRecord,
    joints: &ImpulseJointSet,
    multibody_joints: &MultibodyJointSet,
    bodies: &RigidBodySet,
    dt_sec: f32,
) -> Result<JointSnapshot, PhysicsError> {
    let (impulses, body1_handle, body2_handle) = match record.handle {
        JointBackendHandle::Impulse(handle) => {
            let joint = joints
                .get(handle)
                .ok_or_else(|| PhysicsError::UnknownJoint(id.clone()))?;
            (record.observed_impulse, joint.body1, joint.body2)
        }
        JointBackendHandle::Multibody(handle) => {
            let (multibody, link_id) = multibody_joints
                .get(handle)
                .ok_or_else(|| PhysicsError::UnknownJoint(id.clone()))?;
            let link = multibody
                .link(link_id)
                .ok_or_else(|| PhysicsError::UnknownJoint(id.clone()))?;
            let parent_id = link
                .parent_id()
                .ok_or_else(|| PhysicsError::UnknownJoint(id.clone()))?;
            let parent = multibody
                .link(parent_id)
                .ok_or_else(|| PhysicsError::UnknownJoint(id.clone()))?;
            (
                record.observed_impulse,
                parent.rigid_body_handle(),
                link.rigid_body_handle(),
            )
        }
    };
    let body1 = bodies
        .get(body1_handle)
        .ok_or_else(|| PhysicsError::UnknownBody(record.desc.body1.clone()))?;
    let body2 = bodies
        .get(body2_handle)
        .ok_or_else(|| PhysicsError::UnknownBody(record.desc.body2.clone()))?;
    let frame1 = body1.position() * isometry(record.desc.local_frame1);
    let frame2 = body2.position() * isometry(record.desc.local_frame2);
    let relative = frame1.inverse() * frame2;
    let (position, velocity) = match &record.desc.kind {
        JointKindDesc::Revolute { axis, .. } => {
            let axis_local = vec3(*axis).normalize();
            let axis_world = frame1.rotation * axis_local;
            (
                Some(relative.rotation.scaled_axis().dot(&axis_local)),
                Some((body2.angvel() - body1.angvel()).dot(&axis_world)),
            )
        }
        JointKindDesc::Prismatic { axis, .. } => {
            let axis_world = frame1.rotation * vec3(*axis).normalize();
            (
                Some((frame2.translation.vector - frame1.translation.vector).dot(&axis_world)),
                Some((body2.linvel() - body1.linvel()).dot(&axis_world)),
            )
        }
        JointKindDesc::Fixed | JointKindDesc::Spherical { .. } => (None, None),
    };
    let quaternion = relative.rotation.quaternion();
    let (limit, motor, effort_impulse) = match &record.desc.kind {
        JointKindDesc::Revolute { limits, motor, .. } => {
            (*limits, *motor, Some(impulses[JointAxis::AngX as usize]))
        }
        JointKindDesc::Prismatic { limits, motor, .. } => {
            (*limits, *motor, Some(impulses[JointAxis::LinX as usize]))
        }
        JointKindDesc::Fixed | JointKindDesc::Spherical { .. } => (None, None, None),
    };
    let (limit_state, limit_error) = match (position, limit) {
        (Some(position), Some(limit)) if position < limit.minimum => (
            Some(JointLimitState::BelowMinimum),
            Some(position - limit.minimum),
        ),
        (Some(position), Some(limit)) if position > limit.maximum => (
            Some(JointLimitState::AboveMaximum),
            Some(position - limit.maximum),
        ),
        (Some(_), Some(_)) => (Some(JointLimitState::WithinLimits), Some(0.0)),
        _ => (None, None),
    };
    let mut linear_error = relative.translation.vector;
    let mut angular_error = relative.rotation.scaled_axis();
    match &record.desc.kind {
        JointKindDesc::Fixed => {}
        JointKindDesc::Revolute { axis, .. } => {
            let axis = vec3(*axis).normalize();
            angular_error -= axis * angular_error.dot(&axis);
        }
        JointKindDesc::Prismatic { axis, .. } => {
            let axis = vec3(*axis).normalize();
            linear_error -= axis * linear_error.dot(&axis);
        }
        JointKindDesc::Spherical { .. } => angular_error = Vector::zeros(),
    }
    Ok(JointSnapshot {
        id,
        desc: record.desc.clone(),
        position,
        velocity,
        relative_rotation_xyzw: [quaternion.i, quaternion.j, quaternion.k, quaternion.w],
        applied_impulse: impulses,
        limit_state,
        limit_error,
        motor_position_error: position
            .zip(motor)
            .map(|(position, motor)| motor.target_position - position),
        applied_effort: effort_impulse
            .and_then(|impulse| (dt_sec.is_finite() && dt_sec > 0.0).then_some(impulse / dt_sec)),
        friction_maximum_effort: record.friction_maximum_effort,
        friction_applied_impulse: record.friction_applied_impulse,
        break_thresholds: record.break_thresholds,
        constraint_error: [
            linear_error.x,
            linear_error.y,
            linear_error.z,
            angular_error.x,
            angular_error.y,
            angular_error.z,
        ],
    })
}

fn compare_contact_points(left: &ContactPointSnapshot, right: &ContactPointSnapshot) -> Ordering {
    compare_float_slices(&left.point1_m, &right.point1_m)
        .then_with(|| compare_float_slices(&left.point2_m, &right.point2_m))
        .then_with(|| left.distance_m.total_cmp(&right.distance_m))
        .then_with(|| left.normal_impulse_ns.total_cmp(&right.normal_impulse_ns))
}

fn compare_contact_manifolds(
    left: &ContactManifoldSnapshot,
    right: &ContactManifoldSnapshot,
) -> Ordering {
    left.subshape1
        .cmp(&right.subshape1)
        .then_with(|| left.subshape2.cmp(&right.subshape2))
        .then_with(|| compare_float_slices(&left.normal_on_collider1, &right.normal_on_collider1))
        .then_with(|| match (left.points.first(), right.points.first()) {
            (Some(left), Some(right)) => compare_contact_points(left, right),
            (None, Some(_)) => Ordering::Less,
            (Some(_), None) => Ordering::Greater,
            (None, None) => Ordering::Equal,
        })
        .then_with(|| left.points.len().cmp(&right.points.len()))
}

fn compare_float_slices(left: &[f32], right: &[f32]) -> Ordering {
    left.iter()
        .zip(right)
        .find_map(|(left, right)| {
            let ordering = left.total_cmp(right);
            (ordering != Ordering::Equal).then_some(ordering)
        })
        .unwrap_or_else(|| left.len().cmp(&right.len()))
}

fn finite_optional(value: f32) -> Option<f32> {
    value.is_finite().then_some(value)
}

fn fnv1a64(bytes: &[u8]) -> u64 {
    bytes.iter().fold(0xcbf2_9ce4_8422_2325, |hash, byte| {
        (hash ^ u64::from(*byte)).wrapping_mul(0x0000_0100_0000_01b3)
    })
}

fn pair_events(
    previous_contacts: &BTreeSet<PairKey>,
    contacts: &BTreeSet<PairKey>,
    previous_sensors: &BTreeSet<PairKey>,
    sensors: &BTreeSet<PairKey>,
    substep_index: u32,
) -> Vec<PhysicsEvent> {
    let mut events = Vec::new();
    append_events(
        &mut events,
        contacts.difference(previous_contacts),
        PhysicsEventKind::ContactStarted,
        substep_index,
    );
    append_events(
        &mut events,
        previous_contacts.difference(contacts),
        PhysicsEventKind::ContactStopped,
        substep_index,
    );
    append_events(
        &mut events,
        sensors.difference(previous_sensors),
        PhysicsEventKind::SensorStarted,
        substep_index,
    );
    append_events(
        &mut events,
        previous_sensors.difference(sensors),
        PhysicsEventKind::SensorStopped,
        substep_index,
    );
    events
}

fn append_events<'a>(
    events: &mut Vec<PhysicsEvent>,
    pairs: impl Iterator<Item = &'a PairKey>,
    kind: PhysicsEventKind,
    substep_index: u32,
) {
    for pair in pairs {
        events.push(PhysicsEvent {
            kind: kind.clone(),
            collider1: pair.0.clone(),
            collider2: pair.1.clone(),
            step_index: 0,
            substep_index,
            sequence: 0,
        });
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn dynamic_ball(id: &str, z: f32) -> (BodyId, BodyDesc, ColliderId, ColliderDesc) {
        let body = BodyId::new(id);
        let body_desc = BodyDesc {
            pose: Pose {
                translation: [0.0, 0.0, z],
                ..Pose::default()
            },
            ..BodyDesc::default()
        };
        let collider = ColliderId::new(format!("{id}:collider"));
        let collider_desc = ColliderDesc::new(ColliderShape::Sphere { radius: 0.1 });
        (body, body_desc, collider, collider_desc)
    }

    fn add_floor(world: &mut PhysicsWorld) {
        let floor = BodyId::new("floor");
        world
            .create_body(
                floor.clone(),
                BodyDesc {
                    mode: BodyMode::Static,
                    pose: Pose {
                        translation: [0.0, 0.0, -0.05],
                        ..Pose::default()
                    },
                    ..BodyDesc::default()
                },
            )
            .unwrap();
        world
            .create_collider(
                ColliderId::new("floor:collider"),
                &floor,
                ColliderDesc::new(ColliderShape::Box {
                    size: [4.0, 4.0, 0.1],
                }),
            )
            .unwrap();
    }

    #[test]
    fn persistent_body_falls_contacts_and_keeps_stable_identity() {
        let mut world = PhysicsWorld::default();
        add_floor(&mut world);
        let (body, body_desc, collider, collider_desc) = dynamic_ball("ball", 0.5);
        world.create_body(body.clone(), body_desc).unwrap();
        world
            .create_collider(collider, &body, collider_desc)
            .unwrap();

        let mut saw_contact = false;
        for _ in 0..120 {
            let output = world.step();
            saw_contact |= output
                .events
                .iter()
                .any(|event| event.kind == PhysicsEventKind::ContactStarted);
        }

        let snapshot = world.body_snapshot(&body).unwrap();
        assert!(saw_contact);
        assert!(snapshot.pose.translation[2] >= 0.09);
        assert_eq!(snapshot.id, body);
        assert_eq!(world.diagnostics().body_count, 2);
    }

    #[test]
    fn compound_query_lifecycle_and_ray_cast_use_public_ids() {
        let mut world = PhysicsWorld::default();
        let body = BodyId::new("compound");
        world
            .create_body(
                body.clone(),
                BodyDesc {
                    mode: BodyMode::Static,
                    ..BodyDesc::default()
                },
            )
            .unwrap();
        let collider = ColliderId::new("compound:collider");
        world
            .create_collider(
                collider.clone(),
                &body,
                ColliderDesc::new(ColliderShape::Compound {
                    children: vec![
                        ColliderChildDesc {
                            pose: Pose {
                                translation: [-0.2, 0.0, 0.0],
                                ..Pose::default()
                            },
                            shape: ColliderShape::Box {
                                size: [0.2, 0.2, 0.2],
                            },
                        },
                        ColliderChildDesc {
                            pose: Pose {
                                translation: [0.2, 0.0, 0.0],
                                ..Pose::default()
                            },
                            shape: ColliderShape::Sphere { radius: 0.1 },
                        },
                    ],
                }),
            )
            .unwrap();

        let hit = world
            .cast_ray([-1.0, 0.0, 0.0], [1.0, 0.0, 0.0], 2.0, true)
            .unwrap()
            .unwrap();
        assert_eq!(hit.collider, collider);
        let overlaps = world
            .overlap_shape(
                Pose::default(),
                &ColliderShape::Box {
                    size: [1.0, 1.0, 1.0],
                },
            )
            .unwrap();
        assert_eq!(overlaps, vec![collider.clone()]);
        world.remove_collider(&collider).unwrap();
        world.remove_body(&body).unwrap();
        assert_eq!(world.diagnostics().body_count, 0);
        assert_eq!(world.diagnostics().collider_count, 0);
    }

    #[test]
    fn set_body_pose_refreshes_attached_colliders_before_queries() {
        let mut world = PhysicsWorld::default();
        let body = BodyId::new("query-body");
        world
            .create_body(
                body.clone(),
                BodyDesc {
                    mode: BodyMode::Static,
                    ..BodyDesc::default()
                },
            )
            .unwrap();
        world
            .create_collider(
                ColliderId::new("query-collider"),
                &body,
                ColliderDesc::new(ColliderShape::Box {
                    size: [1.0, 1.0, 1.0],
                }),
            )
            .unwrap();
        assert!(world
            .cast_ray([-2.0, 0.0, 0.0], [1.0, 0.0, 0.0], 4.0, true)
            .unwrap()
            .is_some());

        world
            .set_body_pose(
                &body,
                Pose {
                    translation: [10.0, 0.0, 0.0],
                    ..Pose::default()
                },
                false,
            )
            .unwrap();

        assert!(world
            .cast_ray([-2.0, 0.0, 0.0], [1.0, 0.0, 0.0], 4.0, true)
            .unwrap()
            .is_none());
        assert!(world
            .cast_ray([8.0, 0.0, 0.0], [1.0, 0.0, 0.0], 4.0, true)
            .unwrap()
            .is_some());
    }

    #[test]
    fn full_inertia_tensor_is_complete_symmetric_and_positive_definite() {
        let mut world = PhysicsWorld::default();
        let body = BodyId::new("tensor-body");
        world
            .create_body(
                body.clone(),
                BodyDesc {
                    mass: Some(MassPropertiesDesc {
                        mass_kg: 2.0,
                        inertia_tensor_kg_m2: Some([
                            [2.0, 0.2, 0.1],
                            [0.2, 3.0, 0.3],
                            [0.1, 0.3, 4.0],
                        ]),
                        ..MassPropertiesDesc::default()
                    }),
                    ..BodyDesc::default()
                },
            )
            .unwrap();
        let effective = world.body_snapshot(&body).unwrap().effective_mass;
        assert!((effective.mass_kg - 2.0).abs() < 1.0e-5);
        assert_eq!(effective.center_of_mass_m, [0.0; 3]);
        let expected = [[2.0, 0.2, 0.1], [0.2, 3.0, 0.3], [0.1, 0.3, 4.0]];
        for (row, expected_row) in expected.iter().enumerate() {
            for (column, expected_value) in expected_row.iter().enumerate() {
                assert!(
                    (effective.inertia_tensor_kg_m2[row][column] - expected_value).abs() < 1.0e-4
                );
            }
        }

        let invalid = world.create_body(
            BodyId::new("invalid-tensor"),
            BodyDesc {
                mass: Some(MassPropertiesDesc {
                    inertia_tensor_kg_m2: Some([[1.0, 2.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]),
                    ..MassPropertiesDesc::default()
                }),
                ..BodyDesc::default()
            },
        );
        assert_eq!(invalid, Err(PhysicsError::InvalidValue("inertia tensor")));
    }

    #[test]
    fn checkpoint_restore_replays_fixed_step_state() {
        let mut world = PhysicsWorld::default();
        let (body, body_desc, collider, collider_desc) = dynamic_ball("ball", 1.0);
        world.create_body(body.clone(), body_desc).unwrap();
        world
            .create_collider(collider, &body, collider_desc)
            .unwrap();
        for _ in 0..10 {
            world.step();
        }
        let checkpoint = world.checkpoint();
        let provenance = checkpoint.provenance();
        assert_eq!(provenance.checkpoint_version, PHYSICS_CHECKPOINT_VERSION);
        assert!(world.compare_checkpoint(&checkpoint).implementation_matches);
        assert!(world.compare_checkpoint(&checkpoint).state_matches);
        for _ in 0..5 {
            world.step();
        }
        assert_eq!(
            world.compare_checkpoint(&checkpoint).first_divergence,
            Some("state_digest")
        );
        let expected = world.body_snapshot(&body).unwrap();
        world.restore(&checkpoint).unwrap();
        assert!(world.compare_checkpoint(&checkpoint).state_matches);
        for _ in 0..5 {
            world.step();
        }
        let replayed = world.body_snapshot(&body).unwrap();
        for index in 0..3 {
            assert!(
                (expected.pose.translation[index] - replayed.pose.translation[index]).abs()
                    < 1.0e-5
            );
            assert!(
                (expected.linear_velocity_mps[index] - replayed.linear_velocity_mps[index]).abs()
                    < 1.0e-5
            );
        }
        assert_eq!(world.snapshot().step_index, 15);
    }

    #[test]
    fn ordered_step_commands_repeat_exactly_and_roll_back_atomically() {
        let mut world = PhysicsWorld::new(PhysicsConfig {
            gravity_mps2: [0.0; 3],
            ..PhysicsConfig::default()
        })
        .unwrap();
        let body = BodyId::new("commanded");
        world
            .create_body(body.clone(), BodyDesc::default())
            .unwrap();
        world
            .create_collider(
                ColliderId::new("commanded:collider"),
                &body,
                ColliderDesc::new(ColliderShape::Box {
                    size: [0.2, 0.1, 0.1],
                }),
            )
            .unwrap();
        let checkpoint = world.checkpoint();
        let input = StepInput {
            commands: vec![
                PhysicsCommand::AddForceAtPoint {
                    id: body.clone(),
                    force_n: [1.0, 0.0, 0.0],
                    point_world_m: [0.0, 0.1, 0.0],
                    wake_up: true,
                },
                PhysicsCommand::AddTorque {
                    id: body.clone(),
                    torque_nm: [0.0, 0.0, 0.25],
                    wake_up: true,
                },
            ],
        };
        let first = (0..5)
            .map(|_| {
                let output = world.step_with_commands(input.clone()).unwrap();
                (output.snapshot, output.events)
            })
            .collect::<Vec<_>>();
        world.restore(&checkpoint).unwrap();
        let second = (0..5)
            .map(|_| {
                let output = world.step_with_commands(input.clone()).unwrap();
                (output.snapshot, output.events)
            })
            .collect::<Vec<_>>();
        assert_eq!(first, second);

        world.restore(&checkpoint).unwrap();
        let before = world.snapshot();
        let failure = world.step_with_commands(StepInput {
            commands: vec![
                PhysicsCommand::AddForce {
                    id: body,
                    force_n: [1.0, 0.0, 0.0],
                    wake_up: true,
                },
                PhysicsCommand::WakeUp {
                    id: BodyId::new("missing"),
                },
            ],
        });
        assert!(matches!(failure, Err(PhysicsError::UnknownBody(_))));
        assert_eq!(world.snapshot(), before);
    }

    #[test]
    fn force_at_point_and_runtime_body_mode_are_supported() {
        let mut world = PhysicsWorld::new(PhysicsConfig {
            gravity_mps2: [0.0; 3],
            ..PhysicsConfig::default()
        })
        .unwrap();
        let body = BodyId::new("vehicle");
        world
            .create_body(body.clone(), BodyDesc::default())
            .unwrap();
        world
            .create_collider(
                ColliderId::new("vehicle:collider"),
                &body,
                ColliderDesc::new(ColliderShape::Box {
                    size: [0.4, 0.2, 0.1],
                }),
            )
            .unwrap();
        world
            .add_force_at_point(&body, [2.0, 0.0, 0.0], [0.0, 0.1, 0.0], true)
            .unwrap();
        world.step();
        let moved = world.body_snapshot(&body).unwrap();
        assert!(moved.linear_velocity_mps[0] > 0.0);
        assert!(moved.angular_velocity_rps[2].abs() > 0.0);
        world
            .set_body_mode(&body, BodyMode::KinematicPosition, true)
            .unwrap();
        world
            .set_next_kinematic_pose(
                &body,
                Pose {
                    translation: [1.0, 0.0, 0.0],
                    ..Pose::default()
                },
            )
            .unwrap();
        world.step();
        assert!(world.body_snapshot(&body).unwrap().pose.translation[0] > 0.9);
    }

    #[test]
    fn bounded_kinematic_target_respects_motion_limits_and_pushes_with_stable_contacts() {
        let mut world = PhysicsWorld::new(PhysicsConfig {
            gravity_mps2: [0.0; 3],
            fixed_dt_sec: 0.05,
            substeps: 4,
        })
        .unwrap();
        let pusher = BodyId::new("pusher");
        world
            .create_body(
                pusher.clone(),
                BodyDesc {
                    mode: BodyMode::KinematicPosition,
                    ccd_enabled: true,
                    ..BodyDesc::default()
                },
            )
            .unwrap();
        let pusher_collider = ColliderId::new("pusher:collider");
        world
            .create_collider(
                pusher_collider.clone(),
                &pusher,
                ColliderDesc::new(ColliderShape::Box {
                    size: [0.2, 0.2, 0.2],
                }),
            )
            .unwrap();
        let ball = BodyId::new("pushed-ball");
        world
            .create_body(
                ball.clone(),
                BodyDesc {
                    pose: Pose {
                        translation: [0.35, 0.0, 0.0],
                        ..Pose::default()
                    },
                    ccd_enabled: true,
                    ..BodyDesc::default()
                },
            )
            .unwrap();
        let ball_collider = ColliderId::new("pushed-ball:collider");
        world
            .create_collider(
                ball_collider.clone(),
                &ball,
                ColliderDesc::new(ColliderShape::Sphere { radius: 0.1 }),
            )
            .unwrap();
        let target = BoundedKinematicTarget {
            pose: Pose {
                translation: [1.0, 0.0, 0.0],
                rotation_xyzw: [
                    0.0,
                    0.0,
                    std::f32::consts::FRAC_1_SQRT_2,
                    std::f32::consts::FRAC_1_SQRT_2,
                ],
            },
            maximum_linear_speed_mps: 0.5,
            maximum_angular_speed_rps: 1.0,
            maximum_linear_acceleration_mps2: 1.0,
            maximum_angular_acceleration_rps2: 2.0,
        };
        world.set_bounded_kinematic_target(&pusher, target).unwrap();

        let mut previous = world.body_snapshot(&pusher).unwrap().pose;
        let mut previous_linear_speed = 0.0;
        let mut previous_angular_speed = 0.0;
        let mut saw_contact = false;
        for _ in 0..40 {
            // Robot-link targets may be refreshed every frame. This must not
            // restart the acceleration ramp.
            world.set_bounded_kinematic_target(&pusher, target).unwrap();
            let output = world.step();
            let current_state = world.body_snapshot(&pusher).unwrap();
            let current = current_state.pose;
            let linear_speed = (current.translation[0] - previous.translation[0]) / 0.05;
            assert!(linear_speed <= 0.5 + 1.0e-4, "linear speed {linear_speed}");
            assert!(
                (linear_speed - previous_linear_speed).abs() <= 1.0 * 0.05 + 1.0e-3,
                "linear acceleration step {}",
                linear_speed - previous_linear_speed
            );
            let quaternion_dot = current
                .rotation_xyzw
                .iter()
                .zip(previous.rotation_xyzw)
                .map(|(left, right)| left * right)
                .sum::<f32>()
                .abs()
                .min(1.0);
            let angular_step = 2.0 * quaternion_dot.acos();
            assert!(
                angular_step <= 1.0 * 0.05 + 5.0e-4,
                "angular step {angular_step}"
            );
            let angular_speed = current_state.angular_velocity_rps[2].abs();
            assert!(
                (angular_speed - previous_angular_speed).abs() <= 2.0 * 0.05 + 1.0e-2,
                "angular acceleration step {}",
                angular_speed - previous_angular_speed
            );
            saw_contact |= output.events.iter().any(|event| {
                event.kind == PhysicsEventKind::ContactStarted
                    && event.collider1 == ball_collider
                    && event.collider2 == pusher_collider
            });
            previous = current;
            previous_linear_speed = linear_speed;
            previous_angular_speed = angular_speed;
        }
        let pusher_state = world.body_snapshot(&pusher).unwrap();
        let ball_state = world.body_snapshot(&ball).unwrap();
        assert!(pusher_state.pose.translation[0] > 0.5);
        assert!(ball_state.pose.translation[0] > 0.35);
        assert!(ball_state
            .linear_velocity_mps
            .iter()
            .all(|component| component.is_finite() && component.abs() < 10.0));
        assert!(saw_contact);
    }

    #[test]
    fn coupled_kinematic_target_applies_acceleration_caps_to_one_pose_fraction() {
        let mut world = PhysicsWorld::new(PhysicsConfig {
            gravity_mps2: [0.0; 3],
            fixed_dt_sec: 0.1,
            substeps: 1,
        })
        .unwrap();
        let body = BodyId::new("coupled-acceleration");
        world
            .create_body(
                body.clone(),
                BodyDesc {
                    mode: BodyMode::KinematicPosition,
                    ..BodyDesc::default()
                },
            )
            .unwrap();
        world
            .set_bounded_kinematic_target_with_mode(
                &body,
                BoundedKinematicTarget {
                    pose: Pose {
                        translation: [1.0, 0.0, 0.0],
                        rotation_xyzw: [
                            0.0,
                            0.0,
                            std::f32::consts::FRAC_1_SQRT_2,
                            std::f32::consts::FRAC_1_SQRT_2,
                        ],
                    },
                    maximum_linear_speed_mps: 100.0,
                    maximum_angular_speed_rps: 100.0,
                    maximum_linear_acceleration_mps2: 1.0,
                    maximum_angular_acceleration_rps2: 2.0,
                },
                KinematicTargetMode::CoupledPose,
            )
            .unwrap();

        world.step();
        let snapshot = world.body_snapshot(&body).unwrap();
        let translation_fraction = snapshot.pose.translation[0];
        let rotation_angle = 2.0 * snapshot.pose.rotation_xyzw[2].asin();
        let rotation_fraction = rotation_angle / std::f32::consts::FRAC_PI_2;
        assert!((translation_fraction - rotation_fraction).abs() < 1.0e-5);
        assert!((translation_fraction - 0.01).abs() < 1.0e-5);
        assert!(snapshot.linear_velocity_mps[0] <= 1.0 * 0.1 + 1.0e-5);
        assert!(snapshot.angular_velocity_rps[2] <= 2.0 * 0.1 + 1.0e-5);
    }

    #[test]
    fn retired_ids_cannot_be_reused_by_stale_callers() {
        let mut world = PhysicsWorld::default();
        let body = BodyId::new("once");
        world
            .create_body(body.clone(), BodyDesc::default())
            .unwrap();
        world.remove_body(&body).unwrap();
        assert_eq!(
            world.create_body(body.clone(), BodyDesc::default()),
            Err(PhysicsError::RetiredBody(body))
        );
    }

    #[test]
    fn removing_active_contact_queues_a_stopped_event() {
        let mut world = PhysicsWorld::default();
        add_floor(&mut world);
        let (body, body_desc, collider, collider_desc) = dynamic_ball("ball", 0.2);
        world.create_body(body.clone(), body_desc).unwrap();
        world
            .create_collider(collider.clone(), &body, collider_desc)
            .unwrap();
        for _ in 0..60 {
            world.step();
        }
        assert!(world.diagnostics().active_contact_count > 0);
        world.remove_collider(&collider).unwrap();
        assert!(world
            .step()
            .events
            .iter()
            .any(|event| event.kind == PhysicsEventKind::ContactStopped));
    }

    #[test]
    fn checkpoint_preserves_contact_history_without_false_restart() {
        let mut world = PhysicsWorld::default();
        add_floor(&mut world);
        let (body, body_desc, collider, collider_desc) = dynamic_ball("ball", 0.2);
        world.create_body(body.clone(), body_desc).unwrap();
        world
            .create_collider(collider, &body, collider_desc)
            .unwrap();
        for _ in 0..60 {
            world.step();
        }
        let checkpoint = world.checkpoint();
        world.step();
        world.restore(&checkpoint).unwrap();
        let output = world.step();
        assert!(!output
            .events
            .iter()
            .any(|event| event.kind == PhysicsEventKind::ContactStarted));
    }

    #[test]
    fn sensor_pairs_are_ordered_events_and_snapshot_truth() {
        let mut world = PhysicsWorld::new(PhysicsConfig {
            gravity_mps2: [0.0; 3],
            ..PhysicsConfig::default()
        })
        .unwrap();
        for (name, sensor) in [("a", true), ("b", false)] {
            let body = BodyId::new(name);
            world
                .create_body(
                    body.clone(),
                    BodyDesc {
                        mode: if sensor {
                            BodyMode::Static
                        } else {
                            BodyMode::Dynamic
                        },
                        ..BodyDesc::default()
                    },
                )
                .unwrap();
            let mut collider = ColliderDesc::new(ColliderShape::Sphere { radius: 0.2 });
            collider.sensor = sensor;
            world
                .create_collider(ColliderId::new(format!("{name}:collider")), &body, collider)
                .unwrap();
        }
        let output = world.step();
        assert!(output
            .events
            .iter()
            .any(|event| event.kind == PhysicsEventKind::SensorStarted));
        assert!(output
            .snapshot
            .contacts
            .iter()
            .any(|contact| contact.sensor));
    }

    #[test]
    fn stack_contacts_expose_manifolds_analytics_and_live_debug_geometry() {
        let mut world = PhysicsWorld::default();
        add_floor(&mut world);
        for (name, z) in [("lower", 0.15), ("upper", 0.36)] {
            let body = BodyId::new(name);
            world
                .create_body(
                    body.clone(),
                    BodyDesc {
                        pose: Pose {
                            translation: [0.0, 0.0, z],
                            ..Pose::default()
                        },
                        ccd_enabled: name == "upper",
                        ..BodyDesc::default()
                    },
                )
                .unwrap();
            world
                .create_collider(
                    ColliderId::new(format!("{name}:collider")),
                    &body,
                    ColliderDesc::new(ColliderShape::Box {
                        size: [0.2, 0.2, 0.2],
                    }),
                )
                .unwrap();
        }
        let sleeper = BodyId::new("sleeper");
        world
            .create_body(
                sleeper.clone(),
                BodyDesc {
                    sleeping: true,
                    pose: Pose {
                        translation: [10.0, 0.0, 10.0],
                        ..Pose::default()
                    },
                    ..BodyDesc::default()
                },
            )
            .unwrap();
        for _ in 0..180 {
            world.step();
        }
        let snapshot = world.snapshot();
        assert!(snapshot.contacts.len() >= 2);
        for contact in snapshot.contacts.iter().filter(|contact| !contact.sensor) {
            assert!(contact.collider1 < contact.collider2);
            assert!(!contact.manifolds.is_empty());
            for manifold in &contact.manifolds {
                let normal_length = vec3(manifold.normal_on_collider1).norm();
                assert!((normal_length - 1.0).abs() < 1.0e-4);
                assert!(!manifold.points.is_empty());
                for point in &manifold.points {
                    assert!(point.point1_m.iter().all(|value| value.is_finite()));
                    assert!(point.point2_m.iter().all(|value| value.is_finite()));
                    assert!(point
                        .relative_velocity_mps
                        .iter()
                        .all(|value| value.is_finite()));
                    assert!(point.penetration_depth_m >= 0.0);
                    assert!(point.normal_impulse_ns >= 0.0);
                    assert!(point
                        .tangent_impulse_magnitude_ns
                        .is_none_or(|impulse| impulse >= 0.0));
                }
            }
        }
        assert_eq!(snapshot.debug_geometry.len(), 3);
        assert!(snapshot
            .debug_geometry
            .windows(2)
            .all(|pair| pair[0].collider <= pair[1].collider));
        assert_eq!(world.diagnostics().ccd_enabled_body_count, 1);
        assert!(world.diagnostics().contact_manifold_count >= 2);
        assert!(world.diagnostics().contact_point_count >= 2);
        assert_eq!(world.diagnostics().sleeping_island_count, None);
        assert_eq!(world.diagnostics().backend_phase_seconds, None);
        assert_eq!(world.diagnostics().estimated_resource_bytes, None);
        assert!(
            world
                .body_snapshot(&BodyId::new("upper"))
                .unwrap()
                .ccd_enabled
        );
        let sleeping = world.body_snapshot(&sleeper).unwrap();
        assert!(sleeping.sleeping);
        assert!(!sleeping.solver_active);
        world.wake_up(&sleeper).unwrap();
        assert!(!world.body_snapshot(&sleeper).unwrap().sleeping);
        world.step();
        assert!(world.diagnostics().active_dynamic_body_count > 0);
    }

    #[test]
    fn sensor_crossing_preserves_substep_event_order() {
        let mut world = PhysicsWorld::new(PhysicsConfig {
            gravity_mps2: [0.0; 3],
            fixed_dt_sec: 0.4,
            substeps: 8,
        })
        .unwrap();
        let sensor_body = BodyId::new("sensor");
        world
            .create_body(
                sensor_body.clone(),
                BodyDesc {
                    mode: BodyMode::Static,
                    ..BodyDesc::default()
                },
            )
            .unwrap();
        let mut sensor = ColliderDesc::new(ColliderShape::Sphere { radius: 0.2 });
        sensor.sensor = true;
        world
            .create_collider(ColliderId::new("sensor"), &sensor_body, sensor)
            .unwrap();
        let mover = BodyId::new("mover");
        world
            .create_body(
                mover.clone(),
                BodyDesc {
                    mode: BodyMode::Dynamic,
                    pose: Pose {
                        translation: [-0.8, 0.0, 0.0],
                        ..Pose::default()
                    },
                    linear_velocity_mps: [4.0, 0.0, 0.0],
                    ..BodyDesc::default()
                },
            )
            .unwrap();
        world
            .create_collider(
                ColliderId::new("mover"),
                &mover,
                ColliderDesc::new(ColliderShape::Sphere { radius: 0.1 }),
            )
            .unwrap();

        let events = world.step().events;
        assert_eq!(
            events.len(),
            2,
            "mover {:?}, contacts {:?}",
            world.body_snapshot(&mover).unwrap(),
            world.snapshot().contacts
        );
        assert_eq!(events[0].kind, PhysicsEventKind::SensorStarted);
        assert_eq!(events[1].kind, PhysicsEventKind::SensorStopped);
        assert!(events[0].substep_index < events[1].substep_index);
        assert_eq!(events[0].step_index, 1);
        assert_eq!(events[0].sequence, 0);
        assert_eq!(events[1].sequence, 1);
        assert_eq!(world.diagnostics().active_sensor_count, 0);
    }

    fn add_joint_body(world: &mut PhysicsWorld, id: &str, x: f32, mode: BodyMode) -> BodyId {
        let body = BodyId::new(id);
        world
            .create_body(
                body.clone(),
                BodyDesc {
                    mode,
                    pose: Pose {
                        translation: [x, 0.0, 0.0],
                        ..Pose::default()
                    },
                    mass: (mode == BodyMode::Dynamic).then_some(MassPropertiesDesc {
                        mass_kg: 1.0,
                        principal_inertia_kg_m2: [0.1; 3],
                        ..MassPropertiesDesc::default()
                    }),
                    ..BodyDesc::default()
                },
            )
            .unwrap();
        body
    }

    #[test]
    fn fixed_joint_lifecycle_snapshot_diagnostics_and_checkpoint_are_stable() {
        let mut world = PhysicsWorld::new(PhysicsConfig {
            gravity_mps2: [0.0; 3],
            ..PhysicsConfig::default()
        })
        .unwrap();
        let base = add_joint_body(&mut world, "fixed-base", 0.0, BodyMode::Static);
        let child = add_joint_body(&mut world, "fixed-child", 1.0, BodyMode::Dynamic);
        let joint_id = JointId::new("fixed-joint");
        world
            .create_joint(
                joint_id.clone(),
                JointDesc {
                    body1: base.clone(),
                    body2: child.clone(),
                    local_frame1: Pose {
                        translation: [0.5, 0.0, 0.0],
                        ..Pose::default()
                    },
                    local_frame2: Pose {
                        translation: [-0.5, 0.0, 0.0],
                        ..Pose::default()
                    },
                    kind: JointKindDesc::Fixed,
                    contacts_enabled: false,
                },
            )
            .unwrap();
        world.add_force(&child, [10.0, 0.0, 0.0], true).unwrap();
        for _ in 0..60 {
            world.step();
        }
        assert!((world.body_snapshot(&child).unwrap().pose.translation[0] - 1.0).abs() < 0.02);
        assert_eq!(world.joint_snapshot(&joint_id).unwrap().position, None);
        assert_eq!(world.diagnostics().joint_count, 1);

        let checkpoint = world.checkpoint();
        world.remove_joint(&joint_id).unwrap();
        assert_eq!(world.diagnostics().joint_count, 0);
        world.restore(&checkpoint).unwrap();
        assert_eq!(world.snapshot().joints[0].id, joint_id);
        world.remove_body(&base).unwrap();
        assert_eq!(world.diagnostics().joint_count, 0);
    }

    #[test]
    fn prismatic_motor_limit_update_and_all_joint_kinds_are_backend_neutral() {
        let mut world = PhysicsWorld::new(PhysicsConfig {
            gravity_mps2: [0.0; 3],
            substeps: 2,
            ..PhysicsConfig::default()
        })
        .unwrap();
        let base = add_joint_body(&mut world, "base", 0.0, BodyMode::Static);
        let slider = add_joint_body(&mut world, "slider", 0.0, BodyMode::Dynamic);
        let joint_id = JointId::new("b-slider");
        let slider_desc = JointDesc {
            body1: base.clone(),
            body2: slider,
            local_frame1: Pose::default(),
            local_frame2: Pose::default(),
            kind: JointKindDesc::Prismatic {
                axis: [1.0, 0.0, 0.0],
                limits: Some(JointLimitDesc {
                    minimum: -0.1,
                    maximum: 0.5,
                }),
                motor: Some(JointMotorDesc {
                    target_position: 0.35,
                    target_velocity: 0.0,
                    stiffness: 40.0,
                    damping: 8.0,
                    maximum_force: 20.0,
                }),
            },
            contacts_enabled: false,
        };
        world
            .create_joint(joint_id.clone(), slider_desc.clone())
            .unwrap();
        for _ in 0..180 {
            world.step();
        }
        let driven = world.joint_snapshot(&joint_id).unwrap();
        assert!(driven.position.unwrap() > 0.2, "{driven:?}");
        assert!(driven.position.unwrap() <= 0.51, "{driven:?}");
        world
            .update_joint_motor(
                &joint_id,
                JointMotorAxis::Primary,
                Some(JointMotorDesc {
                    target_position: 0.1,
                    target_velocity: -0.05,
                    stiffness: 40.0,
                    damping: 8.0,
                    maximum_force: 20.0,
                }),
            )
            .unwrap();
        for _ in 0..180 {
            world.step();
        }
        let returned = world.joint_snapshot(&joint_id).unwrap();
        assert!(returned.position.unwrap() < 0.2);
        assert_eq!(returned.limit_state, Some(JointLimitState::WithinLimits));
        assert_eq!(returned.limit_error, Some(0.0));
        assert!(returned.motor_position_error.unwrap().abs() < 0.1);
        assert!(returned.applied_effort.unwrap().is_finite());
        assert!(returned
            .constraint_error
            .iter()
            .all(|error| error.is_finite()));
        world.set_joint_friction(&joint_id, 0.1).unwrap();
        world
            .set_joint_break_thresholds(
                &joint_id,
                JointBreakThresholds {
                    maximum_force_n: Some(10.0),
                    maximum_torque_nm: None,
                    maximum_linear_impulse_ns: None,
                    maximum_angular_impulse_nms: None,
                },
            )
            .unwrap();
        assert!(PhysicsWorld::capabilities().multibody_joint_chains);
        assert!(PhysicsWorld::capabilities().joint_friction);
        assert!(PhysicsWorld::capabilities().automatic_joint_break_thresholds);

        let revolute_child = add_joint_body(&mut world, "revolute", 1.0, BodyMode::Dynamic);
        world
            .create_joint(
                JointId::new("c-revolute"),
                JointDesc {
                    body1: base.clone(),
                    body2: revolute_child.clone(),
                    local_frame1: Pose::default(),
                    local_frame2: Pose::default(),
                    kind: JointKindDesc::Revolute {
                        axis: [0.0, 0.0, 1.0],
                        limits: Some(JointLimitDesc {
                            minimum: -0.5,
                            maximum: 0.5,
                        }),
                        motor: None,
                    },
                    contacts_enabled: false,
                },
            )
            .unwrap();
        let spherical_child = add_joint_body(&mut world, "spherical", 2.0, BodyMode::Dynamic);
        world
            .create_joint(
                JointId::new("a-spherical"),
                JointDesc {
                    body1: revolute_child,
                    body2: spherical_child,
                    local_frame1: Pose::default(),
                    local_frame2: Pose::default(),
                    kind: JointKindDesc::Spherical {
                        limits: [None, None, None],
                        motors: [None, None, None],
                    },
                    contacts_enabled: false,
                },
            )
            .unwrap();
        world.step();
        let ids = world
            .snapshot()
            .joints
            .into_iter()
            .map(|joint| joint.id)
            .collect::<Vec<_>>();
        assert_eq!(
            ids,
            vec![
                JointId::new("a-spherical"),
                JointId::new("b-slider"),
                JointId::new("c-revolute")
            ]
        );
        assert_eq!(world.diagnostics().joint_count, 3);
    }

    #[test]
    fn removing_middle_of_joint_chain_cleans_owned_constraints_safely() {
        let mut world = PhysicsWorld::new(PhysicsConfig {
            gravity_mps2: [0.0; 3],
            ..PhysicsConfig::default()
        })
        .unwrap();
        let first = add_joint_body(&mut world, "chain-a", 0.0, BodyMode::Static);
        let middle = add_joint_body(&mut world, "chain-b", 1.0, BodyMode::Dynamic);
        let last = add_joint_body(&mut world, "chain-c", 2.0, BodyMode::Dynamic);
        for (id, body1, body2) in [
            ("chain-ab", first.clone(), middle.clone()),
            ("chain-bc", middle.clone(), last.clone()),
        ] {
            world
                .create_joint(
                    JointId::new(id),
                    JointDesc {
                        body1,
                        body2,
                        local_frame1: Pose::default(),
                        local_frame2: Pose::default(),
                        kind: JointKindDesc::Fixed,
                        contacts_enabled: false,
                    },
                )
                .unwrap();
        }
        world.step();
        world.remove_body(&middle).unwrap();
        assert_eq!(world.diagnostics().joint_count, 0);
        assert!(world.body_snapshot(&first).is_ok());
        assert!(world.body_snapshot(&last).is_ok());
        assert!(matches!(
            world.joint_snapshot(&JointId::new("chain-ab")),
            Err(PhysicsError::UnknownJoint(_))
        ));
        world.step();
    }

    #[test]
    fn query_matrix_filters_and_ties_are_backend_neutral() {
        let mut world = PhysicsWorld::new(PhysicsConfig {
            gravity_mps2: [0.0; 3],
            ..PhysicsConfig::default()
        })
        .unwrap();
        for (name, memberships, sensor, x) in [
            ("a", 0b01, false, 0.0),
            ("b", 0b10, false, 0.0),
            ("sensor", 0b01, true, 2.0),
        ] {
            let body = BodyId::new(name);
            world
                .create_body(
                    body.clone(),
                    BodyDesc {
                        mode: BodyMode::Static,
                        pose: Pose {
                            translation: [x, 0.0, 0.0],
                            ..Pose::default()
                        },
                        ..BodyDesc::default()
                    },
                )
                .unwrap();
            let mut collider = ColliderDesc::new(ColliderShape::Sphere { radius: 0.5 });
            collider.sensor = sensor;
            collider.collision_memberships = memberships;
            world
                .create_collider(ColliderId::new(name), &body, collider)
                .unwrap();
        }

        let ray = world
            .cast_ray([-2.0, 0.0, 0.0], [1.0, 0.0, 0.0], 5.0, true)
            .unwrap()
            .unwrap();
        assert_eq!(ray.collider, ColliderId::new("a"));
        let select_b = PhysicsQueryFilter {
            groups: Some(PhysicsQueryGroups {
                memberships: u32::MAX,
                filter: 0b10,
            }),
            include_sensors: false,
            ..PhysicsQueryFilter::default()
        };
        assert_eq!(
            world
                .cast_ray_filtered([-2.0, 0.0, 0.0], [1.0, 0.0, 0.0], 5.0, true, &select_b)
                .unwrap()
                .unwrap()
                .collider,
            ColliderId::new("b")
        );
        let exclude_a = PhysicsQueryFilter {
            excluded_bodies: vec![BodyId::new("a")],
            include_sensors: false,
            ..PhysicsQueryFilter::default()
        };
        assert_eq!(
            world
                .project_point([0.0, 2.0, 0.0], true, &exclude_a)
                .unwrap()
                .unwrap()
                .collider,
            ColliderId::new("b")
        );
        assert_eq!(
            world
                .cast_shape(
                    Pose {
                        translation: [-2.0, 0.0, 0.0],
                        ..Pose::default()
                    },
                    [1.0, 0.0, 0.0],
                    &ColliderShape::Sphere { radius: 0.25 },
                    5.0,
                    true,
                    &PhysicsQueryFilter::default(),
                )
                .unwrap()
                .unwrap()
                .collider,
            ColliderId::new("a")
        );
        assert_eq!(
            world
                .closest_distance(
                    Pose::default(),
                    &ColliderShape::Sphere { radius: 0.1 },
                    1.0,
                    &PhysicsQueryFilter::default(),
                )
                .unwrap()
                .unwrap()
                .collider,
            ColliderId::new("a")
        );
        let sensor_probe = ColliderShape::Sphere { radius: 0.2 };
        let sensor_pose = Pose {
            translation: [2.0, 0.0, 0.0],
            ..Pose::default()
        };
        assert_eq!(
            world
                .overlap_shape_filtered(
                    sensor_pose,
                    &sensor_probe,
                    &PhysicsQueryFilter {
                        include_sensors: false,
                        ..PhysicsQueryFilter::default()
                    },
                )
                .unwrap(),
            Vec::<ColliderId>::new()
        );
        assert_eq!(
            world.overlap_shape(sensor_pose, &sensor_probe).unwrap(),
            vec![ColliderId::new("sensor")]
        );

        let terrain = BodyId::new("terrain");
        world
            .create_body(
                terrain.clone(),
                BodyDesc {
                    mode: BodyMode::Static,
                    pose: Pose {
                        translation: [10.0, 0.0, 0.0],
                        ..Pose::default()
                    },
                    ..BodyDesc::default()
                },
            )
            .unwrap();
        world
            .create_collider(
                ColliderId::new("terrain"),
                &terrain,
                ColliderDesc::new(ColliderShape::HeightField {
                    rows: 2,
                    columns: 2,
                    heights: vec![0.0; 4],
                    scale: [2.0, 1.0, 2.0],
                }),
            )
            .unwrap();
        assert_eq!(
            world
                .cast_ray([10.0, 2.0, 0.0], [0.0, -1.0, 0.0], 4.0, true)
                .unwrap()
                .unwrap()
                .collider,
            ColliderId::new("terrain")
        );
    }

    #[test]
    fn invalid_physics_inputs_fail_instead_of_clamping() {
        assert!(PhysicsWorld::new(PhysicsConfig {
            gravity_mps2: [f32::NAN, 0.0, 0.0],
            ..PhysicsConfig::default()
        })
        .is_err());
        let mut world = PhysicsWorld::default();
        let body = BodyId::new("invalid");
        assert!(world
            .create_body(
                body,
                BodyDesc {
                    linear_damping: -1.0,
                    ..BodyDesc::default()
                },
            )
            .is_err());
        let body = BodyId::new("valid");
        world
            .create_body(body.clone(), BodyDesc::default())
            .unwrap();
        assert!(world
            .create_collider(
                ColliderId::new("bad-shape"),
                &body,
                ColliderDesc::new(ColliderShape::Sphere { radius: 0.0 }),
            )
            .is_err());
    }
}
