#ifndef PHYSICS_H
#define PHYSICS_H

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <assert.h>

typedef struct { float x, y, z; } PGEVec3;
PGEVec3 pge_vec3_new(float x, float y, float z) { return (PGEVec3){ x, y, z }; }
static inline PGEVec3 Vec3_add(PGEVec3 a, PGEVec3 b) { return (PGEVec3){ a.x + b.x, a.y + b.y, a.z + b.z }; }
static inline PGEVec3 Vec3_sub(PGEVec3 a, PGEVec3 b) { return (PGEVec3){ a.x - b.x, a.y - b.y, a.z - b.z }; }
static inline PGEVec3 Vec3_scale(PGEVec3 v, float s) { return (PGEVec3){ v.x * s, v.y * s, v.z * s }; }
static inline float Vec3_dot(PGEVec3 a, PGEVec3 b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
static inline PGEVec3 Vec3_cross(PGEVec3 a, PGEVec3 b) { return (PGEVec3){ a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x }; }
static inline float Vec3_length(PGEVec3 v) { return sqrtf(Vec3_dot(v, v)); }
static inline PGEVec3 Vec3_normalized(PGEVec3 v) {
    float len = Vec3_length(v);
    if (len == 0) return v;
    return Vec3_scale(v, 1.0f / len);
}
typedef struct { float w, x, y, z; } PGEQuat;
static inline PGEQuat Quaternion_identity(void) { return (PGEQuat){ 1.0f, 0.0f, 0.0f, 0.0f }; }
static inline PGEQuat Quaternion_scale(PGEQuat q, float s) { return (PGEQuat){ q.w * s, q.x * s, q.y * s, q.z * s }; }
static inline float Quaternion_mag(PGEQuat q) { return sqrtf(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z); }
static inline PGEQuat Quaternion_normalized(PGEQuat q) { float m = Quaternion_mag(q); return (PGEQuat){ q.w / m, q.x / m, q.y / m, q.z / m }; }
static inline PGEQuat Quaternion_mul(PGEQuat a, PGEQuat b) {
    PGEQuat r = {
        a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
        a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
        a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
        a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w
    };
    return r;
}
static inline PGEQuat pge_quat_integrate(PGEQuat q, PGEVec3 av, float dt) {
    PGEQuat omega = { 0.0f, av.x, av.y, av.z };
    PGEQuat qdot = Quaternion_scale(Quaternion_mul(omega, q), 0.5f);
    PGEQuat r = { q.w + qdot.w * dt, q.x + qdot.x * dt, q.y + qdot.y * dt, q.z + qdot.z * dt };
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
    PGEVec3 position;
    PGEVec3 velocity;
    float mass;
    float restitution;
    PGEQuat rot;
    PGEVec3 avel;
    float inertia;
    CollisionShape shape;
};
typedef struct RigidBody RigidBody;
typedef struct Cell {
    int x, y, z, count, capacity;
    RigidBody **bodies;
    struct Cell *next;
} Cell;
typedef struct {
    int size;
    Cell **table;
} Grid;
unsigned int hash(int x, int y, int z, int size){ return ((unsigned int)x * 73856093u ^ (unsigned int)y * 19349663u ^ (unsigned int)z * 83492791u) % size; }
Grid *pge_create_grid(int size){
    Grid *grid = (Grid*)malloc(sizeof(Grid));
    grid->size = size;
    grid->table = (Cell**)calloc(size, sizeof(Cell *));
    return grid;
}
Cell *pge_create_cell(int x, int y, int z){
    Cell *cell = (Cell*)malloc(sizeof(Cell));
    cell->x = x, cell->y = y, cell->count = 0, cell->capacity = 4;
    cell->bodies = (RigidBody**)(sizeof(RigidBody *) * 4);
    cell->next = NULL;
    return cell;
}
Cell *pge_get_cell(Grid *grid, int x, int y, int z){
    unsigned int index = hash(x, y, z, grid->size);
    Cell *cell = grid->table[index];
    while(cell){
        if(cell->x == x && cell->y == y)return cell;
        cell = cell->next;
    }
    Cell *newCell = pge_create_cell(x, y, z);
    newCell->next = grid->table[index];
    grid->table[index] = newCell;
    return newCell;
}
void pge_grid_insert(Grid *grid, RigidBody *body){
    int x = (int)body->position.x, y = (int)body->position.y, z = (int)body->position.z;
    Cell *cell = pge_get_cell(grid, x, y, z);
    if(cell->count == cell->capacity){
        cell->capacity *= 2;
        cell->bodies = (RigidBody**)(cell->bodies, sizeof(RigidBody *) * cell->capacity);
    }
    cell->bodies[cell->count++] = body;
}
typedef struct {
    RigidBody *bodyA;
    RigidBody *bodyB;
    PGEVec3 anchorA;
    PGEVec3 anchorB;
    float distance;
} Joint;
void Joint_solve(Joint *joint) {
    PGEVec3 rA = joint->anchorA, rB = joint->anchorB;
    PGEVec3 pA = Vec3_add(joint->bodyA->position, rA);
    PGEVec3 pB = Vec3_add(joint->bodyB->position, rB);
    PGEVec3 diff = Vec3_sub(pB, pA);
    float len = Vec3_length(diff);
    PGEVec3 n = (len == 0 ? diff : Vec3_scale(diff, 1.0f / len));
    float c = len - joint->distance;
    float invMass = 1.0f / joint->bodyA->mass + 1.0f / joint->bodyB->mass;
    PGEVec3 impulse = Vec3_scale(n, -c / invMass);
    joint->bodyA->velocity = Vec3_sub(joint->bodyA->velocity, Vec3_scale(impulse, 1.0f / joint->bodyA->mass));
    joint->bodyB->velocity = Vec3_add(joint->bodyB->velocity, Vec3_scale(impulse, 1.0f / joint->bodyB->mass));
}
void RigidBody_integrate(RigidBody *body, float dt) { body->position = Vec3_add(body->position, Vec3_scale(body->velocity, dt)); }
void resolveJoint(Joint *joint, PGEVec3 rA, PGEVec3 rB) {
    PGEVec3 pA = Vec3_add(joint->bodyA->position, rA);
    PGEVec3 pB = Vec3_add(joint->bodyB->position, rB);
    PGEVec3 diff = Vec3_sub(pB, pA);
    float len = Vec3_length(diff);
    PGEVec3 n = (len == 0 ? diff : Vec3_scale(diff, 1.0f / len));
    float c = len - joint->distance;
    float invMass = 1.0f / joint->bodyA->mass + 1.0f / joint->bodyB->mass;
    PGEVec3 impulse = Vec3_scale(n, -c / invMass);
    joint->bodyA->velocity = Vec3_sub(joint->bodyA->velocity, Vec3_scale(impulse, 1.0f / joint->bodyA->mass));
    joint->bodyB->velocity = Vec3_add(joint->bodyB->velocity, Vec3_scale(impulse, 1.0f / joint->bodyB->mass));
}
typedef struct {
    RigidBody *bodies;
    size_t bodyCount;
    PGEVec3 gravity;
    Grid grid;
} Scene;
void scene_update(Scene *scene, float dt) {
    for (size_t i = 0; i < scene->bodyCount; i++) {
        scene->bodies[i].velocity = Vec3_add(scene->bodies[i].velocity, Vec3_scale(scene->gravity, dt));
        RigidBody_integrate(&scene->bodies[i], dt);
        scene->bodies[i].rot = pge_quat_integrate(scene->bodies[i].rot, scene->bodies[i].avel, dt);
    }
}
void scene_detect_collisions(Scene *scene) {
    for(int i = 0; i < scene->grid.size; i++){
        Cell *cell = scene->grid.table[i];
        if (cell->count < 2) continue;
        for (int i = 0; i < cell->count; i++) {
            for (int j = i + 1; j < cell->count; j++) {
                RigidBody *bodyA = cell->bodies[i];
                RigidBody *bodyB = cell->bodies[j];
                if (bodyA->mass == 0 && bodyB->mass == 0) continue;
                PGEVec3 diff = Vec3_sub(bodyB->position, bodyA->position);
                float len = Vec3_length(diff);
                if (len == 0) continue;
                PGEVec3 n = Vec3_scale(diff, 1.0f / len);
                float c = len - 1.0f;
                float invMass = 1.0f / bodyA->mass + 1.0f / bodyB->mass;
                PGEVec3 impulse = Vec3_scale(n, -c / invMass);
                bodyA->velocity = Vec3_sub(bodyA->velocity, Vec3_scale(impulse, 1.0f / bodyA->mass));
                bodyB->velocity = Vec3_add(bodyB->velocity, Vec3_scale(impulse, 1.0f / bodyB->mass));
            }
        }
    }
}

#endif // PHYSICS_H