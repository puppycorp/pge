#[macro_use]
extern crate criterion;

mod process_nodes;

criterion_main! {
	process_nodes::process_nodes
}