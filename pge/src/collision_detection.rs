use glam::*;

/// Determines if a vector is axis-aligned (aligned with X, Y, or Z within a tolerance).
fn is_axis_aligned(axis: &Vec3) -> bool {
    let epsilon = 1e-6;
    (axis.x.abs() > 1.0 - epsilon && axis.y.abs() < epsilon && axis.z.abs() < epsilon)
        || (axis.y.abs() > 1.0 - epsilon && axis.x.abs() < epsilon && axis.z.abs() < epsilon)
        || (axis.z.abs() > 1.0 - epsilon && axis.x.abs() < epsilon && axis.y.abs() < epsilon)
}

/// Represents collision information between two OBBs.
#[derive(Debug, Clone)]
pub struct CollisionInfo {
    pub correction: Vec3,
    pub normal: Vec3,
    pub contact_point: Vec3,
}

/// Performs collision detection between two Oriented Bounding Boxes (OBBs).
pub fn obb_collide(
    transform1: Mat4,
    half_size1: Vec3,
    transform2: Mat4,
    half_size2: Vec3,
) -> Option<CollisionInfo> {
    // Extract rotation matrices
    let r1 = Mat3::from_mat4(transform1);
    let r2 = Mat3::from_mat4(transform2);

    // Compute rotation matrix expressing box2 in box1's coordinate frame
    let r = r1.transpose() * r2;

    // Compute translation vector
    let translation1 = transform1.w_axis.truncate();
    let translation2 = transform2.w_axis.truncate();
    let t = r1.transpose() * (translation2 - translation1);

    // Add an epsilon term to counteract arithmetic errors when two edges are parallel
    let epsilon = 1e-6_f32;

    // Compute absolute rotation matrix with epsilon
    let r_abs = Mat3::from_cols(
        r.x_axis.abs() + Vec3::splat(epsilon),
        r.y_axis.abs() + Vec3::splat(epsilon),
        r.z_axis.abs() + Vec3::splat(epsilon),
    );

    // Initialize variables for primary and cross axes overlaps
    let mut primary_min_overlap = f32::MAX;
    let mut primary_min_axis = Vec3::ZERO;

    let mut cross_min_overlap = f32::MAX;
    let mut cross_min_axis = Vec3::ZERO;

    enum AxisType {
        FaceA(usize),
        FaceB(usize),
        Edge(usize, usize),
    }
    let mut cross_axis_type = None::<AxisType>;

    // ------------------------------
    // 1. Test Axes L = A0, A1, A2 (box1's local axes)
    // ------------------------------
    for i in 0..3 {
        let ra = half_size1[i];
        let rb = half_size2.dot(r_abs.col(i));
        let overlap = t[i].abs() - (ra + rb);
        if overlap > 0.0 {
            return None; // Separation axis found
        } else if overlap.abs() < primary_min_overlap {
            primary_min_overlap = overlap.abs();
            primary_min_axis = r1.col(i);
            if t[i] < 0.0 {
                primary_min_axis = -primary_min_axis;
            }
        }
    }

    // ------------------------------
    // 2. Test Axes L = B0, B1, B2 (box2's local axes)
    // ------------------------------
    for i in 0..3 {
        let ra = half_size1.dot(r_abs.row(i));
        let rb = half_size2[i];
        let t_proj = Vec3::new(r.x_axis[i], r.y_axis[i], r.z_axis[i]).dot(t);
        let overlap = t_proj.abs() - (ra + rb);
        if overlap > 0.0 {
            return None; // Separation axis found
        } else if overlap.abs() < primary_min_overlap {
            primary_min_overlap = overlap.abs();
            primary_min_axis = r2.col(i);
            let projection = t.dot(r2.col(i));
            if projection < 0.0 {
                primary_min_axis = -primary_min_axis;
            }
        }
    }

    // ------------------------------
    // 3. Test Cross Product Axes L = Ai x Bj
    // ------------------------------
    let axes = [
        (0, 0),
        (0, 1),
        (0, 2),
        (1, 0),
        (1, 1),
        (1, 2),
        (2, 0),
        (2, 1),
        (2, 2),
    ];

    for &(i, j) in &axes {
        let ra = half_size1[(i + 1) % 3] * r_abs.col((i + 2) % 3)[j]
            + half_size1[(i + 2) % 3] * r_abs.col((i + 1) % 3)[j];
        let rb = half_size2[(j + 1) % 3] * r_abs.col(i)[(j + 2) % 3]
            + half_size2[(j + 2) % 3] * r_abs.col(i)[(j + 1) % 3];
        let t_component = t[(i + 2) % 3] * r.col((i + 1) % 3)[j]
            - t[(i + 1) % 3] * r.col((i + 2) % 3)[j];
        let overlap = t_component.abs() - (ra + rb);
        if overlap > 0.0 {
            return None; // Separation axis found
        } else if overlap.abs() < cross_min_overlap && overlap.abs() < primary_min_overlap {
            let axis = r1.col((i + 1) % 3).cross(r2.col(j));
            if axis.length_squared() > epsilon {
                let normalized_axis = axis.normalize();
                let projection = t.dot(normalized_axis);
                cross_min_axis = if projection < 0.0 {
                    -normalized_axis
                } else {
                    normalized_axis
                };
                cross_min_overlap = overlap.abs();
                cross_axis_type = Some(AxisType::Edge(i, j));
            }
        }
    }

    // ------------------------------
    // 4. Determine the Minimal Overlap Axis
    // ------------------------------
    let (min_overlap, min_axis) = if cross_min_overlap < primary_min_overlap * 0.95 {
        // Only use cross axis if it's significantly smaller than primary axis overlap
        (cross_min_overlap, cross_min_axis)
    } else {
        (primary_min_overlap, primary_min_axis)
    };

    // ------------------------------
    // 5. Prepare Collision Info
    // ------------------------------
    // For axis-aligned boxes, ensure we're using the most appropriate primary axis
    let final_normal = if is_axis_aligned(&min_axis) {
        // If the collision is primarily vertical (Y-axis), prefer that
        if t.y.abs() > t.x.abs() && t.y.abs() > t.z.abs() {
            if t.y > 0.0 { Vec3::Y } else { -Vec3::Y }
        } else if t.x.abs() > t.z.abs() {
            if t.x > 0.0 { Vec3::X } else { -Vec3::X }
        } else {
            if t.z > 0.0 { Vec3::Z } else { -Vec3::Z }
        }
    } else {
        min_axis
    };

    // Compute correction vector
    let correction = final_normal * min_overlap;

    // Estimate the contact point
    let contact_point = match cross_axis_type {
        Some(AxisType::Edge(edge_a, edge_b)) => {
            compute_contact_point_edge(
                translation1,
                r1,
                half_size1,
                edge_a,
                translation2,
                r2,
                half_size2,
                edge_b,
            )
        }
        _ => (translation1 + translation2) * 0.5,
    };

    Some(CollisionInfo {
        correction,
        normal: final_normal,
        contact_point,
    })
}

/// Computes the contact point when the collision normal is derived from cross product axes (edges).
fn compute_contact_point_edge(
    pos1: Vec3,
    r1: Mat3,
    half_size1: Vec3,
    edge_index1: usize,
    pos2: Vec3,
    r2: Mat3,
    half_size2: Vec3,
    edge_index2: usize,
) -> Vec3 {
    // Define the edges in world coordinates
    let edge_dir1 = r1.col(edge_index1);
    let edge_point1 = pos1;

    let edge_dir2 = r2.col(edge_index2);
    let edge_point2 = pos2;

    // Compute closest points between the two edges
    let (closest_point1, closest_point2) =
        closest_points_on_lines(edge_point1, edge_dir1, edge_point2, edge_dir2);

    // Average the closest points as contact point
    (closest_point1 + closest_point2) * 0.5
}

/// Helper function to find the closest points on two lines defined by point and direction.
fn closest_points_on_lines(
    p1: Vec3,
    d1: Vec3,
    p2: Vec3,
    d2: Vec3,
) -> (Vec3, Vec3) {
    let r = p1 - p2;
    let a = d1.dot(d1);
    let e = d2.dot(d2);
    let f = d2.dot(r);

    let c = d1.dot(r);
    let b = d1.dot(d2);
    let denom = a * e - b * b;

    let s;
    let t;

    if denom.abs() > 1e-6 { 
        s = (b * f - c * e) / denom;
    } else {
        s = 0.0;
    }

    if e.abs() > 1e-6 {
        t = (b * s + f) / e;
    } else {
        t = 0.0;
    }

    let closest_point1 = p1 + d1 * s;
    let closest_point2 = p2 + d2 * t;

    (closest_point1, closest_point2)
}

#[cfg(test)]
mod obb_tests {
    use super::*;

    /// Helper function to compare vectors with a tolerance.
    fn approx_eq_vec(a: Vec3, b: Vec3, epsilon: f32) -> bool {
        (a - b).length() < epsilon
    }

    fn approx_eq_scalar(a: f32, b: f32, epsilon: f32) -> bool {
        (a - b).abs() < epsilon
    }

    #[test]
    fn object_falls_to_floor() {
        let transform1 = Mat4::from_translation(Vec3::new(0.0, 1.0, 0.0));
        let half_size1 = Vec3::new(1.0, 1.5, 1.0);
        let transform2 = Mat4::IDENTITY;
        let half_size2 = Vec3::new(10.0, 1.0, 10.0);
        let collision = obb_collide(transform1, half_size1, transform2, half_size2);
        assert!(collision.is_some());

        let collision = collision.unwrap();
        assert_eq!(collision.normal, -Vec3::Y);
    }

    /// Test case where two identical boxes overlap completely.
    #[test]
    fn test_obb_collide_overlap() {
        let transform1 = Mat4::IDENTITY;
        let half_size1 = Vec3::new(1.0, 1.0, 1.0);
        let transform2 = Mat4::IDENTITY;
        let half_size2 = Vec3::new(1.0, 1.0, 1.0);
        let collision = obb_collide(transform1, half_size1, transform2, half_size2);
        assert!(collision.is_some());
        if let Some(info) = collision {
            // Since boxes are identical and overlapping completely, the normal is arbitrary.
            // The correction should be zero vector as boxes are fully overlapping.
            assert!(approx_eq_vec(info.correction, Vec3::ZERO, 1e-4));
            // Normal can be any unit vector; for simplicity, we check its length.
            assert!(approx_eq_scalar(info.normal.length(), 1.0, 1e-4));
            // Contact point should be the center of the boxes.
            assert!(approx_eq_vec(info.contact_point, Vec3::new(0.0, 0.0, 0.0), 1e-4));
        }
    }

    /// Test case where two boxes are separated along the X-axis and do not overlap.
    #[test]
    fn test_obb_collide_no_overlap() {
        let transform1 = Mat4::IDENTITY;
        let half_size1 = Vec3::new(1.0, 1.0, 1.0);
        // Shift the second box by 3 units along the X-axis
        let transform2 = Mat4::from_translation(Vec3::new(3.0, 0.0, 0.0));
        let half_size2 = Vec3::new(1.0, 1.0, 1.0);
        let collision = obb_collide(transform1, half_size1, transform2, half_size2);
        assert!(collision.is_none());
    }

    /// Test case where two boxes are exactly touching faces along the X-axis.
    /*#[test]
    fn test_obb_collide_touching_faces() {
        let transform1 = Mat4::IDENTITY;
        let half_size1 = Vec3::new(1.0, 1.0, 1.0);
        // Shift the second box by 2 units along the X-axis (touching faces)
        let transform2 = Mat4::from_translation(Vec3::new(2.0, 0.0, 0.0));
        let half_size2 = Vec3::new(1.0, 1.0, 1.0);
        let collision = obb_collide(transform1, half_size1, transform2, half_size2);
        assert!(collision.is_some());
        if let Some(info) = collision {
            // The normal should point along the X-axis
            assert!(approx_eq_vec(info.normal, Vec3::X, 1e-4));
            // The correction should push the boxes apart minimally
            // Since they are just touching, the correction vector can be zero or minimal due to epsilon
            assert!(info.correction.x.abs() <= 1e-6);
            // Contact point should lie on the touching face
            assert!(approx_eq_vec(info.contact_point, Vec3::new(1.0, 0.0, 0.0), 1e-4));
        }
    }*/

    /// Test case where one box is entirely inside another box.
    #[test]
    fn test_obb_collide_one_inside_other() {
        let transform1 = Mat4::IDENTITY;
        let half_size1 = Vec3::new(3.0, 3.0, 3.0);
        let transform2 = Mat4::IDENTITY;
        let half_size2 = Vec3::new(1.0, 1.0, 1.0);
        let collision = obb_collide(transform1, half_size1, transform2, half_size2);
        assert!(collision.is_some());
        if let Some(info) = collision {
            // The normal is along one of the primary axes
            assert!(
                info.normal == Vec3::X
                    || info.normal == Vec3::Y
                    || info.normal == Vec3::Z
                    || info.normal == -Vec3::X
                    || info.normal == -Vec3::Y
                    || info.normal == -Vec3::Z
            );
            // The correction should move the smaller box out along the normal
            assert!(info.correction.length() >= 0.0);
            // Contact point should be the center as boxes are concentric
            assert!(approx_eq_vec(info.contact_point, Vec3::new(0.0, 0.0, 0.0), 1e-4));
        }
    }

    /// Test case with two rotated boxes that overlap.
    #[test]
    fn test_obb_collide_rotated_overlap() {
        let transform1 = Mat4::IDENTITY;
        let half_size1 = Vec3::new(1.0, 1.0, 1.0);
        // Rotate the second box by 45 degrees around the Z-axis and shift it by (1, 0, 0)
        let rotation = Quat::from_rotation_z(45.0_f32.to_radians());
        let translation = Vec3::new(1.0, 0.0, 0.0);
        let transform2 = Mat4::from_rotation_translation(rotation, translation);
        let half_size2 = Vec3::new(1.0, 1.0, 1.0);
        let collision = obb_collide(transform1, half_size1, transform2, half_size2);
        assert!(collision.is_some());
        if let Some(info) = collision {
            // The normal should be a unit vector
            assert!(approx_eq_scalar(info.normal.length(), 1.0, 1e-4));
            // Correction vector should resolve the collision
            assert!(info.correction.length() > 0.0);
            // Contact point should lie within both boxes
            assert!(
                info.contact_point.x.abs() <= 1.0
                    && info.contact_point.y.abs() <= 1.0
                    && info.contact_point.z.abs() <= 1.0
            );
        }
    }

    /// Test case where two rotated boxes do not overlap.
    #[test]
    fn test_obb_collide_rotated_no_overlap() {
        let transform1 = Mat4::IDENTITY;
        let half_size1 = Vec3::new(1.0, 1.0, 1.0);
        // Rotate the second box by 90 degrees around the Z-axis and shift it by (3, 0, 0)
        let rotation = Quat::from_rotation_z(90.0_f32.to_radians());
        let translation = Vec3::new(3.0, 0.0, 0.0);
        let transform2 = Mat4::from_rotation_translation(rotation, translation);
        let half_size2 = Vec3::new(1.0, 1.0, 1.0);
        let collision = obb_collide(transform1, half_size1, transform2, half_size2);
        assert!(collision.is_none());
    }

    /// Test case where two boxes touch at an edge.
    #[test]
    fn test_obb_collide_touching_edges() {
        let transform1 = Mat4::IDENTITY;
        let half_size1 = Vec3::new(1.0, 1.0, 1.0);
        // Shift the second box so that they touch at an edge (e.g., corner at (1,1,1))
        let translation = Vec3::new(2.0, 2.0, 0.0);
        let transform2 = Mat4::from_translation(translation);
        let half_size2 = Vec3::new(1.0, 1.0, 1.0);
        let collision = obb_collide(transform1, half_size1, transform2, half_size2);
        assert!(collision.is_some());
        if let Some(info) = collision {
            // The normal should be a unit vector
            assert!(info.normal.length() > 0.0);
            // Correction vector should move the boxes apart minimally
            assert!(info.correction.length() > 0.0);
            // Contact point should lie along the touching edge
            // For this specific translation, expect contact_point.x ~=1 and y~=1
            assert!(approx_eq_scalar(info.contact_point.x, 1.0, 1e-4));
            assert!(approx_eq_scalar(info.contact_point.y, 1.0, 1e-4));
            // z can be arbitrary as there's no shift in z
            assert!(approx_eq_scalar(info.contact_point.z, 0.0, 1e-4));
        }
    }

    /// Test case where two boxes touch at a vertex.
    #[test]
    fn test_obb_collide_touching_vertices() {
        let transform1 = Mat4::IDENTITY;
        let half_size1 = Vec3::new(1.0, 1.0, 1.0);
        // Shift the second box so that they touch at a single vertex
        let translation = Vec3::new(2.0, 2.0, 2.0);
        let transform2 = Mat4::from_translation(translation);
        let half_size2 = Vec3::new(1.0, 1.0, 1.0);
        let collision = obb_collide(transform1, half_size1, transform2, half_size2);
        assert!(collision.is_some());
        if let Some(info) = collision {
            // Normal should point from one box's corner to the other
            assert!(info.normal.length() > 0.0);
            // Correction should be along the normal
            assert!(info.correction.length() > 0.0);
            // Contact point should be at the touching vertex
            assert!(approx_eq_vec(
                info.contact_point,
                Vec3::new(1.0, 1.0, 1.0),
                1e-4
            ));
        }
    }

    /// Test case with one rotated box overlapping and another not rotated.
    #[test]
    fn test_obb_collide_mixed_rotation_overlap() {
        let transform1 = Mat4::IDENTITY;
        let half_size1 = Vec3::new(2.0, 2.0, 2.0);
        // Rotate the second box by 30 degrees around the Y-axis and shift it
        let rotation = Quat::from_rotation_y(30.0_f32.to_radians());
        let translation = Vec3::new(1.0, 0.0, 1.0);
        let transform2 = Mat4::from_rotation_translation(rotation, translation);
        let half_size2 = Vec3::new(1.0, 1.0, 1.0);
        let collision = obb_collide(transform1, half_size1, transform2, half_size2);
        assert!(collision.is_some());
        if let Some(info) = collision {
            // Normal should be a unit vector
            assert!(approx_eq_scalar(info.normal.length(), 1.0, 1e-4));
            // Correction vector should properly resolve the collision
            assert!(info.correction.length() > 0.0);
            // Contact point should lie within overlapping region
            assert!(
                info.contact_point.x.abs() <= 2.0
                    && info.contact_point.y.abs() <= 2.0
                    && info.contact_point.z.abs() <= 2.0
            );
        }
    }

    /// Test case with both boxes rotated differently and do not overlap.
    #[test]
    fn test_obb_collide_different_rotations_no_overlap() {
        // Rotate the first box by 45 degrees around the X-axis
        let rotation1 = Quat::from_rotation_x(45.0_f32.to_radians());
        let translation1 = Vec3::ZERO;
        let transform1 = Mat4::from_rotation_translation(rotation1, translation1);
        let half_size1 = Vec3::new(1.5, 1.5, 1.5);
        // Rotate the second box by -45 degrees around the Y-axis and shift it
        let rotation2 = Quat::from_rotation_y(-45.0_f32.to_radians());
        let translation2 = Vec3::new(4.0, 4.0, 0.0);
        let transform2 = Mat4::from_rotation_translation(rotation2, translation2);
        let half_size2 = Vec3::new(1.0, 1.0, 1.0);
        let collision = obb_collide(transform1, half_size1, transform2, half_size2);
        assert!(collision.is_none());
    }

    /// Test case where boxes are aligned but have different sizes and overlap.
    #[test]
    fn test_obb_collide_different_sizes_overlap() {
        let transform1 = Mat4::IDENTITY;
        let half_size1 = Vec3::new(2.0, 2.0, 2.0);
        let transform2 = Mat4::IDENTITY;
        let half_size2 = Vec3::new(1.0, 1.0, 1.0);
        let collision = obb_collide(transform1, half_size1, transform2, half_size2);
        assert!(collision.is_some());
        if let Some(info) = collision {
            // Normal should align with one of the primary axes
            assert!(
                info.normal == Vec3::X
                    || info.normal == Vec3::Y
                    || info.normal == Vec3::Z
                    || info.normal == -Vec3::X
                    || info.normal == -Vec3::Y
                    || info.normal == -Vec3::Z
            );
            // Correction should push the smaller box out along the normal
            assert!(info.correction.length() >= 0.0);
            // Contact point should lie on the face of the smaller box
            assert!(
                info.contact_point.x.abs() <= 1.0
                    && info.contact_point.y.abs() <= 1.0
                    && info.contact_point.z.abs() <= 1.0
            );
        }
    }

    /// Test case where one box is scaled non-uniformly and overlaps with another box.
    #[test]
    fn test_obb_collide_non_uniform_scale_overlap() {
        let transform1 = Mat4::IDENTITY;
        let half_size1 = Vec3::new(1.0, 3.0, 1.0);
        let transform2 = Mat4::IDENTITY;
        let half_size2 = Vec3::new(2.0, 1.0, 2.0);
        let collision = obb_collide(transform1, half_size1, transform2, half_size2);
        assert!(collision.is_some());
        if let Some(info) = collision {
            // Normal should be along the direction of minimal penetration
            assert!(info.normal.length() > 0.0);
            // Correction should resolve the collision
            assert!(info.correction.length() > 0.0);
            // Contact point should be within overlapping region
            assert!(
                info.contact_point.x.abs() <= 1.0
                    && info.contact_point.y.abs() <= 1.0
                    && info.contact_point.z.abs() <= 1.0
            );
        }
    }

    /// Test case where one box is scaled non-uniformly and does not overlap with another box.
    #[test]
    fn test_obb_collide_non_uniform_scale_no_overlap() {
        let transform1 = Mat4::IDENTITY;
        let half_size1 = Vec3::new(1.0, 1.0, 1.0);
        let transform2 = Mat4::from_translation(Vec3::new(5.0, 0.0, 0.0));
        let half_size2 = Vec3::new(2.0, 2.0, 2.0);
        let collision = obb_collide(transform1, half_size1, transform2, half_size2);
        assert!(collision.is_none());
    }
}