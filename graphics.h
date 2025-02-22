#ifndef GRAPHICS_H
#define GRAPHICS_H

struct Buffer;
struct Texture;
struct Texture;
struct Pipeline;
Buffer* pge_create_buffer(const char* name, int size);
void pge_destroy_buffer(Buffer* buffer);
Texture* pge_create_texture(const char* name, void *data, int width, int height);
void pge_destroy_texture(Texture* texture);
Pipeline* pge_create_pipeline(const char* name);
void pge_write_buffer(Buffer* buffer, void* data, int size);

#endif // GRAPHICS_H