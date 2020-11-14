#version 450
#extension GL_ARB_shader_storage_buffer_object : require

struct ForceFieldObject {
    vec3 pos;
    float radius;
    vec3 vel;
    float mass;
    vec3 force;
    float _1;
    vec3 torque;
    float _2;
};

uniform float dt;

layout (std430, binding = 0) volatile buffer objects {
    ForceFieldObject objects[];
};

void main() {
    uint index = gl_GlobalInvocationID.x;
}