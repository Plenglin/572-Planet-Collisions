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

layout (std430, binding = 1) volatile buffer in_data {
    Force interactions[];
};

layout (std430, binding = 0) volatile buffer out_data {
    ForceFieldObject objects[];
};

void main() {
    uint index = gl_GlobalInvocationID.x;
    for (uint i = 0; i < index; i++) {
        Force source = interactions[index * objects.length() + i];
        objects[index].force += source.force;
        objects[index].torque += source.torque;
    }
    for (uint i = index + 1; i < objects.length(); i++) {
        Force source = interactions[i * objects.length() + index];
        objects[index].force += source.force;
        objects[index].torque += source.torque;
    }
}