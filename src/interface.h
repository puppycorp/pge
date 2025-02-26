#ifndef INTERFACE_H
#define INTERFACE_H

#ifdef __cplusplus
extern "C" {
#endif

typedef struct PGEBuffer      PGEBuffer;
typedef struct PGETexture     PGETexture;
typedef struct PGEPipeline    PGEPipeline;

typedef struct PGEBufferSlice {
    PGEBuffer* buffer;
    int offset;
    int size;
} PGEBufferSlice;

typedef struct PGERange {
    int start;
    int end;
} PGERange;
typedef struct PGESubpass {
    PGEBufferSlice** vertex_buffers;
    PGEBufferSlice*  index_buffer;
    PGEPipeline*     Pipeline;
    PGEBuffer**      buffers;
    PGERange**       indices;
    PGERange**       instances;
    PGETexture**     textures;
} PGESubpass;
typedef struct PGERenderPass {
    PGEBufferSlice index_buffer;
    PGEPipeline*   pipeline;
    PGEBuffer**    buffers;
    PGETexture**   textures;
    PGERange**     indices;
    PGERange**     instances;
} PGERenderPass;

// Input event types.
typedef enum {
    // Keyboard events
    PGE_KEY_DOWN,
    PGE_KEY_UP,
    // Mouse events
    PGE_MOUSE_DOWN,
    PGE_MOUSE_UP,
    PGE_MOUSE_MOVE,
    PGE_MOUSE_SCROLL,
    // Controller events
    PGE_CONTROLLER_BUTTON_DOWN,
    PGE_CONTROLLER_BUTTON_UP,
    PGE_CONTROLLER_AXIS,
    // OpenXR / VR events
    PGE_OPENXR_EVENT,
    // No event
    PGE_INPUT_NONE
} PGEInputEventType;

#define PGE_MOD_SHIFT   0x01
#define PGE_MOD_CTRL    0x02
#define PGE_MOD_ALT     0x04

#define PGE_KEYBOARD_A  0x41
#define PGE_KEYBOARD_B  0x42

// Input event structure.
typedef struct PGEInputEvent {
    PGEInputEventType type;
    union {
        struct {
            int keyCode;   // Virtual key code.
            int modifiers; // Bitmask of modifiers (shift, ctrl, etc.).
        } keyboard;
        struct {
            int x;       // X position.
            int y;       // Y position.
            int button;  // Which button (0 = left, 1 = right, etc.).
        } mouse;
        struct {
            int scrollX; // Horizontal scroll offset.
            int scrollY; // Vertical scroll offset.
        } scroll;
        struct {
            int controllerId; // Controller identifier.
            int button;       // Which button.
        } controllerButton;
        // Controller axis event data.
        struct {
            int controllerId; // Controller identifier.
            int axis;         // Axis identifier.
            float value;      // Axis value (typically from -1.0 to 1.0).
        } controllerAxis;
        // OpenXR/VR event data.
        struct {
            int eventCode; // Custom event code for OpenXR events.
        } openxr;
    } data;
} PGEInputEvent;

PGEBuffer*    pge_create_buffer(const char* name, int size);
void          pge_destroy_buffer(PGEBuffer* buffer);
PGETexture*   pge_create_texture(const char* name, void* data, int width, int height);
void          pge_destroy_texture(PGETexture* texture);
PGEPipeline*  pge_create_pipeline(const char* name);
void          pge_write_buffer(PGEBuffer* buffer, void* data, int size);
void          pge_bind_buffer(PGEBuffer* buffer, int index);
void          pge_create_renderpass(PGERenderPass* renderpass);
int           pge_poll_event(PGEInputEvent* event);

typedef struct PGESound PGESound;
PGESound*     pge_load_sound(const char* filename);
void          pge_destroy_sound(PGESound* sound);
void          pge_play_sound(PGESound* sound, float volume);
void          pge_stop_sound(PGESound* sound);
void          pge_set_sound_loop(PGESound* sound, int loop);

#ifdef __cplusplus
}
#endif

#endif // INTERFACE_H