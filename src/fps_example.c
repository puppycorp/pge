#include "engine.h"
#include "interface.h"

int main() {
	PGE_Engine *engine = pge_create_engine();
	PGE_Scene *scene = pge_create_scene(engine);
	pge_add_mesh(scene, pge_cube(0.5));

	PGEInputEvent event;
	while (1) {
		if (pge_poll_event(&event)) {
			switch (event.type) {
				case PGE_KEY_DOWN:
					printf("Key down: %d\n", event.data.keyboard.keyCode);
					break;
				case PGE_KEY_UP:
					printf("Key up: %d\n", event.data.keyboard.keyCode);
					break;
			}
		}

		pge_process(engine, 0.016);
	}
}