#include <metal_stdlib>
using namespace metal;

struct VertexOut {
    float4 position [[position]];
};

vertex VertexOut vertex_main(const device float4 *vertexArray [[buffer(0)]],
                             uint vertexId [[vertex_id]]) {
    VertexOut out;
    out.position = vertexArray[vertexId];
    return out;
}

fragment float4 fragment_main() {
    // Output a solid red color.
    return float4(1.0, 0.0, 0.0, 1.0);
}