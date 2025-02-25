#include "physics.h"
#include "test.h"

TEST(vec3_operations){
    PGEVec3 v1 = { 1.0f, 2.0f, 3.0f }, v2 = { 4.0f, 5.0f, 6.0f };
    PGEVec3 vadd = Vec3_add(v1, v2);
    EXPECT_APPROX_EQ(vadd.x, 5.0f, 0.001f);
    EXPECT_APPROX_EQ(vadd.y, 7.0f, 0.001f);
    EXPECT_APPROX_EQ(vadd.z, 9.0f, 0.001f);
    PGEVec3 vsub = Vec3_sub(v2, v1);
    EXPECT_APPROX_EQ(vsub.x, 3.0f, 0.001f);
    EXPECT_APPROX_EQ(vsub.y, 3.0f, 0.001f);
    EXPECT_APPROX_EQ(vsub.z, 3.0f, 0.001f);
    PGEVec3 vscale = Vec3_scale(v1, 2.0f);
    EXPECT_APPROX_EQ(vscale.x, 2.0f, 0.001f);
    EXPECT_APPROX_EQ(vscale.y, 4.0f, 0.001f);
    EXPECT_APPROX_EQ(vscale.z, 6.0f, 0.001f);
    float dot = Vec3_dot(v1, v2);
    EXPECT_APPROX_EQ(dot, 32.0f, 0.001f);
    PGEVec3 vcross = Vec3_cross(v1, v2);
    EXPECT_APPROX_EQ(vcross.x, -3.0f, 0.001f);
    EXPECT_APPROX_EQ(vcross.y, 6.0f, 0.001f);
    EXPECT_APPROX_EQ(vcross.z, -3.0f, 0.001f);
    PGEVec3 norm = Vec3_normalized(v1);
    EXPECT_APPROX_EQ(Vec3_length(norm), 1.0f, 0.001f);
}

TEST(quaternion_integration){
    PGEQuat q = Quaternion_identity();
    PGEVec3 av = { 0.0f, 1.0f, 0.0f };
    float dt = 0.016f;
    PGEQuat qNew = pge_quat_integrate(q, av, dt);
    assert(qNew.w != q.w || qNew.x != q.x || qNew.y != q.y || qNew.z != q.z);
}

TEST(rigidbody_integration){
    RigidBody body = { { 0.0f, 0.0f, 0.0f }, { 1.0f, 2.0f, 3.0f }, 1.0f, 0.5f, Quaternion_identity(), { 0.0f, 0.0f, 0.0f }, 1.0f };
    float dt = 1.0f;
    RigidBody_integrate(&body, dt);
    EXPECT_APPROX_EQ(body.position.x, 1.0f, 0.001f);
    EXPECT_APPROX_EQ(body.position.y, 2.0f, 0.001f);
    EXPECT_APPROX_EQ(body.position.z, 3.0f, 0.001f);
}

TEST(joint_solver){
    RigidBody bodyA = { { 0.0f, 0.0f, 0.0f }, { 0.0f, 0.0f, 0.0f }, 1.0f, 0.5f, Quaternion_identity(), { 0.0f, 0.0f, 0.0f }, 1.0f };
    RigidBody bodyB = { { 2.0f, 0.0f, 0.0f }, { 0.0f, 0.0f, 0.0f }, 1.0f, 0.5f, Quaternion_identity(), { 0.0f, 0.0f, 0.0f }, 1.0f };
    Joint joint = { &bodyA, &bodyB, { 0.0f, 0.0f, 0.0f }, { 0.0f, 0.0f, 0.0f }, 1.0f };
    Joint_solve(&joint);
    EXPECT_APPROX_EQ(bodyA.velocity.x, 0.5f, 0.001f);
    EXPECT_APPROX_EQ(bodyA.velocity.y, 0.0f, 0.001f);
    EXPECT_APPROX_EQ(bodyA.velocity.z, 0.0f, 0.001f);
    EXPECT_APPROX_EQ(bodyB.velocity.x, -0.5f, 0.001f);
    EXPECT_APPROX_EQ(bodyB.velocity.y, 0.0f, 0.001f);
    EXPECT_APPROX_EQ(bodyB.velocity.z, 0.0f, 0.001f);
}

TEST(scene_update){
    RigidBody body = { { 0.0f, 0.0f, 0.0f }, { 1.0f, 0.0f, 0.0f }, 1.0f, 0.5f, Quaternion_identity(), { 0.0f, 1.0f, 0.0f }, 1.0f };
    RigidBody bodies[1];
    bodies[0] = body;
    Scene scene = { bodies, 1, { 0.0f, -9.81f, 0.0f } };
    float dt = 1.0f;
    scene_update(&scene, dt);
    EXPECT_APPROX_EQ(bodies[0].velocity.x, 1.0f, 0.001f);
    EXPECT_APPROX_EQ(bodies[0].velocity.y, -9.81f, 0.001f);
    EXPECT_APPROX_EQ(bodies[0].velocity.z, 0.0f, 0.001f);
    EXPECT_APPROX_EQ(bodies[0].position.x, 1.0f, 0.001f);
    EXPECT_APPROX_EQ(bodies[0].position.y, -9.81f, 0.001f);
    EXPECT_APPROX_EQ(bodies[0].position.z, 0.0f, 0.001f);
    assert(bodies[0].rot.w != 1.0f || bodies[0].rot.x != 0.0f || bodies[0].rot.y != 0.0f || bodies[0].rot.z != 0.0f);
}