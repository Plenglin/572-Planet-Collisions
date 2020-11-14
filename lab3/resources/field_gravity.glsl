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

struct Force {
    vec4 force;
    vec4 torque;
};

layout (std430, binding = 0) volatile buffer in_data {
    ForceFieldObject objects[1];
};

layout (std430, binding = 1) volatile buffer out_data {
    Force interactions[1];
};

void main() {
}