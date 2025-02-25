#ifndef ENGINE_H
#define ENGINE_H

#include "physics.h"
#include "interface.h"

typedef struct PGERayCast {
	float len;
	PGENode **intersects;
} PGERayCast;
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
typedef struct PGEScene {

} PGEScene;
typedef struct PGEContactInfo {
	PGEVec3 normal;
	PGEVec3 point;
	PGENode *node;
} PGEContactInfo;
typedef struct PGENode {
	char           *name;
	PGEVec3        translation;
	PGEVec3        rotation;
	PGEVec3        scale;
	PGENode*       parent;
	PGEScene*      scene;
	PGEContactInfo **contacts;
	PGERayCast	   *raycast;
} PGENode;
typedef struct PGEngine {
	Scene **scenes;
} PGEngine;

void pge_process_meshes(PGEngine *engine, float dt) {

}

void pge_process_cameras(PGEngine *engine, float dt) {

}

void pge_process_lights(PGEngine *engine, float dt) {

}

void pge_process_materials(PGEngine *engine, float dt) {

}

void pge_process_nodes(PGEngine *engine, float dt) {

}

void pge_process_textures(PGEngine *engine, float dt) {

}

void pge_process(PGEngine *engine, float dt) {
	pge_process_meshes(engine, dt);
	pge_process_cameras(engine, dt);
	pge_process_lights(engine, dt);
	pge_process_materials(engine, dt);
	pge_process_nodes(engine, dt);
	pge_process_textures(engine, dt);
}

#endif // ENGINE_H