use criterion::{criterion_group, criterion_main, Criterion};
use pge::*;
use rand::{thread_rng, Rng};

/// Creates a big state with randomly generated nodes and hierarchies.
fn create_big_state() -> State {
    let mut state = State::default();
    let mut rng = thread_rng();
    let num_scenes = 10; // Number of scenes to create

    for _ in 0..num_scenes {
        let mut scene = Scene::default();
        let scene_id = state.scenes.insert(scene);
        let num_nodes = rng.gen_range(5_000..10_000); // Number of nodes per scene
        let mut node_ids = Vec::new();

        for _ in 0..num_nodes {
            // Decide whether to assign a Scene or Node as parent
            let parent = if rng.gen_bool(0.7) && !node_ids.is_empty() {
                // 70% chance to assign an existing node as parent
                let parent_id = node_ids[rng.gen_range(0..node_ids.len())];
                NodeParent::Node(parent_id)
            } else {
                // 30% chance to assign the scene as parent
                NodeParent::Scene(scene_id)
            };

            let node = Node {
                parent,
                // Initialize other fields as needed
                ..Default::default()
            };

            let node_id = state.nodes.insert(node);
            node_ids.push(node_id);
        }

        // Optionally, you can associate the nodes Arena with the scene or state
        // For example:
        // state.scenes.get_mut(scene_id).unwrap().nodes = nodes;

        // If State manages multiple node arenas, ensure to integrate them appropriately
    }

    state
}

fn bench_topo_sort_nodes(c: &mut Criterion) {
	let state = create_big_state();
	println!("state nodes count: {}", state.nodes.len());
	let mut sorted_nodes = Vec::new();
	c.bench_function("topo_sort_nodes", |b| {
		b.iter(|| {
			utility::topo_sort_nodes(&state.nodes, &mut sorted_nodes);
		});
	});
}

criterion_group!(process_nodes, bench_topo_sort_nodes);