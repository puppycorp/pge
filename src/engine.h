#ifndef ENGINE_H
#define ENGINE_H

#include "physics.h"
#include "interface.h"


typedef struct PGERayCast {
	float          len;
	struct PGENode **intersects;
} PGERayCast;
typedef struct PGENode {
	char                  *name;
	PGEVec3               translation;
	PGEVec3               rotation;
	PGEVec3               scale;
	struct PGENode*       parent;
	struct PGE_Scene*     scene;
	struct PGEContactInfo **contacts;
	PGERayCast	          *raycast;
} PGENode;

typedef struct PGECamera {
	float   aspect;
	float   fov;
	float   znear;
	float   zfar;
	PGENode *node;
} PGECamera;
typedef struct PGEMaterial {
	char *name;
	PGETexture *base_color_texture;
	float (*base_color_tex_coords)[2];
	size_t base_color_tex_coords_count;
	float base_color_factor[4];
	PGETexture *metallic_roughness_texture;
	float (*metallic_roughness_tex_coords)[2];
	size_t metallic_roughness_tex_coords_count;
	float metallic_factor;
	float roughness_factor;
	PGETexture *normal_texture;
	float (*normal_tex_coords)[2];
	size_t normal_tex_coords_count;
	float normal_texture_scale;
	PGETexture *occlusion_texture;
	float (*occlusion_tex_coords)[2];
	size_t occlusion_tex_coords_count;
	float occlusion_strength;
	PGETexture *emissive_texture;
	float (*emissive_tex_coords)[2];
	size_t emissive_tex_coords_count;
	float emissive_factor[3];
} PGEMaterial;
typedef struct PGEPrimitive {
	float *vertices[3];
	int   *indices;
	float *normals[3];
	float *uvs[2];
} PGEPrimitive;
typedef struct PGEMesh {
	char         *name;
	PGEPrimitive **primitives;
} PGEMesh;
typedef struct PGE_Scene {

} PGE_Scene;
typedef struct PGEContactInfo {
	PGEVec3 normal;
	PGEVec3 point;
	PGENode *node;
} PGEContactInfo;
typedef struct PGE_Engine {
	Scene **scenes;
} PGE_Engine;

PGE_Engine *pge_create_engine() {
	PGE_Engine *engine = (PGE_Engine*)malloc(sizeof(PGE_Engine));
	// engine->scenes = malloc(sizeof(Scene*));
	return engine;
}

PGENode *pge_create_node() {
	PGENode *node = (PGENode*)malloc(sizeof(PGENode));
	node->translation = pge_vec3_new(0, 0, 0);
	node->rotation = pge_vec3_new(0, 0, 0);
	node->scale = pge_vec3_new(1, 1, 1);
	node->parent = NULL;
	node->scene = NULL;
	node->contacts = NULL;
	node->raycast = NULL;
	return node;
}

PGE_Scene *pge_create_scene(PGE_Engine *engine) {
	PGE_Scene *scene = (PGE_Scene*)malloc(sizeof(PGE_Scene));
	return scene;
}

void pge_process_meshes(PGE_Engine *engine, float dt) {

}

void pge_process_cameras(PGE_Engine *engine, float dt) {

}

void pge_process_lights(PGE_Engine *engine, float dt) {

}

void pge_process_materials(PGE_Engine *engine, float dt) {

}

void pge_process_nodes(PGE_Engine *engine, float dt) {

}

void pge_process_textures(PGE_Engine *engine, float dt) {

}

void pge_process(PGE_Engine *engine, float dt) {
	pge_process_meshes(engine, dt);
	pge_process_cameras(engine, dt);
	pge_process_lights(engine, dt);
	pge_process_materials(engine, dt);
	pge_process_nodes(engine, dt);
	pge_process_textures(engine, dt);
}

void pge_add_mesh(PGE_Scene *scene, PGEMesh *mesh) {

}

PGEMesh *pge_cube(float s) {

}

#endif // ENGINE_H