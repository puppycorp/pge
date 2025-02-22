#ifndef PHYSICS_H
#define PHYSICS_H

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <assert.h>

typedef struct {
    float x, y, z;
} Vec3;
static inline Vec3 Vec3_add(Vec3 a, Vec3 b) { return (Vec3){ a.x + b.x, a.y + b.y, a.z + b.z }; }
static inline Vec3 Vec3_sub(Vec3 a, Vec3 b) { return (Vec3){ a.x - b.x, a.y - b.y, a.z - b.z }; }
static inline Vec3 Vec3_scale(Vec3 v, float s) { return (Vec3){ v.x * s, v.y * s, v.z * s }; }
static inline float Vec3_dot(Vec3 a, Vec3 b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
static inline Vec3 Vec3_cross(Vec3 a, Vec3 b) { return (Vec3){ a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x }; }
static inline float Vec3_length(Vec3 v) {
    return sqrtf(Vec3_dot(v, v));
}
static inline Vec3 Vec3_normalized(Vec3 v) {
    float len = Vec3_length(v);
    if (len == 0) return v;
    return Vec3_scale(v, 1.0f / len);
}

typedef struct {
    float w, x, y, z;
} Quaternion;
static inline Quaternion Quaternion_identity(void) {
    Quaternion q = { 1.0f, 0.0f, 0.0f, 0.0f };
    return q;
}
static inline Quaternion Quaternion_scale(Quaternion q, float s) {
    Quaternion r = { q.w * s, q.x * s, q.y * s, q.z * s };
    return r;
}
static inline float Quaternion_mag(Quaternion q) {
    return sqrtf(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
}
static inline Quaternion Quaternion_normalized(Quaternion q) {
    float m = Quaternion_mag(q);
    Quaternion r = { q.w / m, q.x / m, q.y / m, q.z / m };
    return r;
}
static inline Quaternion Quaternion_mul(Quaternion a, Quaternion b) {
    Quaternion r = {
        a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
        a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
        a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
        a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w
    };
    return r;
}
static inline Quaternion Quaternion_integrate(Quaternion q, Vec3 av, float dt) {
    Quaternion omega = { 0.0f, av.x, av.y, av.z };
    Quaternion qdot = Quaternion_scale(Quaternion_mul(omega, q), 0.5f);
    Quaternion r = { q.w + qdot.w * dt, q.x + qdot.x * dt, q.y + qdot.y * dt, q.z + qdot.z * dt };
    return Quaternion_normalized(r);
}
typedef struct {
    int type;
    union {
        struct { float width, height; } plane;
        struct { float radius; } sphere;
        struct { float width, height, depth; } box;
    };
} CollisionShape;
struct RigidBody {
    Vec3 position;
    Vec3 velocity;
    float mass;
    float restitution;
    Quaternion rot;
    Vec3 avel;
    float inertia;
    CollisionShape shape;
};
typedef struct RigidBody RigidBody;
typedef struct {
    RigidBody *bodyA;
    RigidBody *bodyB;
    Vec3 anchorA;
    Vec3 anchorB;
    float distance;
} Joint;
void Joint_solve(Joint *joint) {
    Vec3 rA = joint->anchorA, rB = joint->anchorB;
    Vec3 pA = Vec3_add(joint->bodyA->position, rA);
    Vec3 pB = Vec3_add(joint->bodyB->position, rB);
    Vec3 diff = Vec3_sub(pB, pA);
    float len = Vec3_length(diff);
    Vec3 n = (len == 0 ? diff : Vec3_scale(diff, 1.0f / len));
    float c = len - joint->distance;
    float invMass = 1.0f / joint->bodyA->mass + 1.0f / joint->bodyB->mass;
    Vec3 impulse = Vec3_scale(n, -c / invMass);
    joint->bodyA->velocity = Vec3_sub(joint->bodyA->velocity, Vec3_scale(impulse, 1.0f / joint->bodyA->mass));
    joint->bodyB->velocity = Vec3_add(joint->bodyB->velocity, Vec3_scale(impulse, 1.0f / joint->bodyB->mass));
}
void RigidBody_integrate(RigidBody *body, float dt) {
    body->position = Vec3_add(body->position, Vec3_scale(body->velocity, dt));
}

void resolveJoint(Joint *joint, Vec3 rA, Vec3 rB) {
    Vec3 pA = Vec3_add(joint->bodyA->position, rA);
    Vec3 pB = Vec3_add(joint->bodyB->position, rB);
    Vec3 diff = Vec3_sub(pB, pA);
    float len = Vec3_length(diff);
    Vec3 n = (len == 0 ? diff : Vec3_scale(diff, 1.0f / len));
    float c = len - joint->distance;
    float invMass = 1.0f / joint->bodyA->mass + 1.0f / joint->bodyB->mass;
    Vec3 impulse = Vec3_scale(n, -c / invMass);
    joint->bodyA->velocity = Vec3_sub(joint->bodyA->velocity, Vec3_scale(impulse, 1.0f / joint->bodyA->mass));
    joint->bodyB->velocity = Vec3_add(joint->bodyB->velocity, Vec3_scale(impulse, 1.0f / joint->bodyB->mass));
}
typedef struct {
    size_t key;
    RigidBody **bodies;
    size_t count;
    size_t capacity;
} GridEntry;
typedef struct {
    float cellSize;
    GridEntry *entries;
    size_t count;
    size_t capacity;
} SpatialGrid;
int SpatialGrid_init(SpatialGrid *grid, float cellSize) {
    grid->cellSize = cellSize;
    grid->count = 0;
    grid->capacity = 16;
    grid->entries = malloc(grid->capacity * sizeof(GridEntry));
    return grid->entries ? 0 : -1;
}
void SpatialGrid_clear(SpatialGrid *grid) {
    for (size_t i = 0; i < grid->count; i++)
        free(grid->entries[i].bodies);
    free(grid->entries);
    grid->entries = NULL;
    grid->count = 0;
    grid->capacity = 0;
}
size_t hashCoords(int x, int y, int z) {
    return ((size_t)x * 73856093u) ^ ((size_t)y * 19349663u) ^ ((size_t)z * 83492791u);
}
void cellCoords(SpatialGrid *grid, Vec3 pos, int *cx, int *cy, int *cz) {
    *cx = (int)floorf(pos.x / grid->cellSize);
    *cy = (int)floorf(pos.y / grid->cellSize);
    *cz = (int)floorf(pos.z / grid->cellSize);
}
int SpatialGrid_insert(SpatialGrid *grid, RigidBody *body) {
    int cx, cy, cz;
    cellCoords(grid, body->position, &cx, &cy, &cz);
    size_t key = hashCoords(cx, cy, cz);
    for (size_t i = 0; i < grid->count; i++) {
        if (grid->entries[i].key == key) {
            if (grid->entries[i].count == grid->entries[i].capacity) {
                size_t cap = grid->entries[i].capacity * 2;
                RigidBody **arr = realloc(grid->entries[i].bodies, cap * sizeof(RigidBody *));
                if (!arr)
                    return -1;
                grid->entries[i].bodies = arr;
                grid->entries[i].capacity = cap;
            }
            grid->entries[i].bodies[grid->entries[i].count++] = body;
            return 0;
        }
    }
    if (grid->count == grid->capacity) {
        size_t cap = grid->capacity * 2;
        GridEntry *arr = realloc(grid->entries, cap * sizeof(GridEntry));
        if (!arr)
            return -1;
        grid->entries = arr;
        grid->capacity = cap;
    }
    GridEntry *entry = &grid->entries[grid->count++];
    entry->key = key;
    entry->capacity = 4;
    entry->count = 0;
    entry->bodies = malloc(entry->capacity * sizeof(RigidBody *));
    if (!entry->bodies)
        return -1;
    entry->bodies[entry->count++] = body;
    return 0;
}
GridEntry *SpatialGrid_queryCell(SpatialGrid *grid, int cx, int cy, int cz) {
    size_t key = hashCoords(cx, cy, cz);
    for (size_t i = 0; i < grid->count; i++) {
        if (grid->entries[i].key == key)
            return &grid->entries[i];
    }
    return NULL;
}
typedef struct {
    RigidBody **bodies;
    size_t count;
    size_t capacity;
} QueryResult;
void QueryResult_init(QueryResult *qr) {
    qr->count = 0;
    qr->capacity = 8;
    qr->bodies = malloc(qr->capacity * sizeof(RigidBody *));
}
int QueryResult_append(QueryResult *qr, RigidBody *body) {
    if (qr->count == qr->capacity) {
        size_t cap = qr->capacity * 2;
        RigidBody **arr = realloc(qr->bodies, cap * sizeof(RigidBody *));
        if (!arr)
            return -1;
        qr->bodies = arr;
        qr->capacity = cap;
    }
    qr->bodies[qr->count++] = body;
    return 0;
}
QueryResult SpatialGrid_queryNearby(SpatialGrid *grid, RigidBody *body) {
    QueryResult res;
    QueryResult_init(&res);
    int cx, cy, cz;
    cellCoords(grid, body->position, &cx, &cy, &cz);
    for (int x = cx - 1; x <= cx + 1; x++) {
        for (int y = cy - 1; y <= cy + 1; y++) {
            for (int z = cz - 1; z <= cz + 1; z++) {
                GridEntry *entry = SpatialGrid_queryCell(grid, x, y, z);
                if (entry) {
                    for (size_t i = 0; i < entry->count; i++) {
                        if (entry->bodies[i] != body)
                            QueryResult_append(&res, entry->bodies[i]);
                    }
                }
            }
        }
    }
    return res;
}
typedef void (*ForEachCellFunc)(size_t key, RigidBody **bodies, size_t count);
void SpatialGrid_forEachCell(SpatialGrid *grid, ForEachCellFunc f) {
    for (size_t i = 0; i < grid->count; i++)
        f(grid->entries[i].key, grid->entries[i].bodies, grid->entries[i].count);
}

typedef struct {
    RigidBody *bodies;
    size_t bodyCount;
    Vec3 gravity;
    SpatialGrid grid;
} Scene;
void Scene_update(Scene *scene, float dt) {
    for (size_t i = 0; i < scene->bodyCount; i++) {
        scene->bodies[i].velocity = Vec3_add(scene->bodies[i].velocity, Vec3_scale(scene->gravity, dt));
        RigidBody_integrate(&scene->bodies[i], dt);
        scene->bodies[i].rot = Quaternion_integrate(scene->bodies[i].rot, scene->bodies[i].avel, dt);
    }
}
void Scene_detectCollisions(Scene *scene) {}

void expectApproxEq(float got, float expected, float tol) {
    assert(fabsf(got - expected) <= tol);
}

int main(void) {
    {
        Vec3 v1 = { 1.0f, 2.0f, 3.0f }, v2 = { 4.0f, 5.0f, 6.0f };
        Vec3 vadd = Vec3_add(v1, v2);
        expectApproxEq(vadd.x, 5.0f, 0.001f);
        expectApproxEq(vadd.y, 7.0f, 0.001f);
        expectApproxEq(vadd.z, 9.0f, 0.001f);
        Vec3 vsub = Vec3_sub(v2, v1);
        expectApproxEq(vsub.x, 3.0f, 0.001f);
        expectApproxEq(vsub.y, 3.0f, 0.001f);
        expectApproxEq(vsub.z, 3.0f, 0.001f);
        Vec3 vscale = Vec3_scale(v1, 2.0f);
        expectApproxEq(vscale.x, 2.0f, 0.001f);
        expectApproxEq(vscale.y, 4.0f, 0.001f);
        expectApproxEq(vscale.z, 6.0f, 0.001f);
        float dot = Vec3_dot(v1, v2);
        expectApproxEq(dot, 32.0f, 0.001f);
        Vec3 vcross = Vec3_cross(v1, v2);
        expectApproxEq(vcross.x, -3.0f, 0.001f);
        expectApproxEq(vcross.y, 6.0f, 0.001f);
        expectApproxEq(vcross.z, -3.0f, 0.001f);
        Vec3 norm = Vec3_normalized(v1);
        expectApproxEq(Vec3_length(norm), 1.0f, 0.001f);
    }
    {
        Quaternion q = Quaternion_identity();
        Vec3 av = { 0.0f, 1.0f, 0.0f };
        float dt = 0.016f;
        Quaternion qNew = Quaternion_integrate(q, av, dt);
        assert(qNew.w != q.w || qNew.x != q.x || qNew.y != q.y || qNew.z != q.z);
    }
    {
        RigidBody body = { { 0.0f, 0.0f, 0.0f }, { 1.0f, 2.0f, 3.0f }, 1.0f, 0.5f, Quaternion_identity(), { 0.0f, 0.0f, 0.0f }, 1.0f };
        float dt = 1.0f;
        RigidBody_integrate(&body, dt);
        expectApproxEq(body.position.x, 1.0f, 0.001f);
        expectApproxEq(body.position.y, 2.0f, 0.001f);
        expectApproxEq(body.position.z, 3.0f, 0.001f);
    }
    {
        RigidBody bodyA = { { 0.0f, 0.0f, 0.0f }, { 0.0f, 0.0f, 0.0f }, 1.0f, 0.5f, Quaternion_identity(), { 0.0f, 0.0f, 0.0f }, 1.0f };
        RigidBody bodyB = { { 2.0f, 0.0f, 0.0f }, { 0.0f, 0.0f, 0.0f }, 1.0f, 0.5f, Quaternion_identity(), { 0.0f, 0.0f, 0.0f }, 1.0f };
        Joint joint = { &bodyA, &bodyB, { 0.0f, 0.0f, 0.0f }, { 0.0f, 0.0f, 0.0f }, 1.0f };
        Joint_solve(&joint);
        expectApproxEq(bodyA.velocity.x, 0.5f, 0.001f);
        expectApproxEq(bodyA.velocity.y, 0.0f, 0.001f);
        expectApproxEq(bodyA.velocity.z, 0.0f, 0.001f);
        expectApproxEq(bodyB.velocity.x, -0.5f, 0.001f);
        expectApproxEq(bodyB.velocity.y, 0.0f, 0.001f);
        expectApproxEq(bodyB.velocity.z, 0.0f, 0.001f);
    }
    {
        RigidBody body = { { 0.0f, 0.0f, 0.0f }, { 1.0f, 0.0f, 0.0f }, 1.0f, 0.5f, Quaternion_identity(), { 0.0f, 1.0f, 0.0f }, 1.0f };
        RigidBody bodies[1];
        bodies[0] = body;
        Scene scene = { bodies, 1, { 0.0f, -9.81f, 0.0f } };
        float dt = 1.0f;
        Scene_update(&scene, dt);
        expectApproxEq(bodies[0].velocity.x, 1.0f, 0.001f);
        expectApproxEq(bodies[0].velocity.y, -9.81f, 0.001f);
        expectApproxEq(bodies[0].velocity.z, 0.0f, 0.001f);
        expectApproxEq(bodies[0].position.x, 1.0f, 0.001f);
        expectApproxEq(bodies[0].position.y, -9.81f, 0.001f);
        expectApproxEq(bodies[0].position.z, 0.0f, 0.001f);
        assert(bodies[0].rot.w != 1.0f || bodies[0].rot.x != 0.0f || bodies[0].rot.y != 0.0f || bodies[0].rot.z != 0.0f);
    }
    return 0;
}

#endif // PHYSICS_H