#version 450 
#extension GL_ARB_shader_storage_buffer_object : require

#define G 4
#define WORKERS 256
#define MAX_CONTACTS_PER_PARTICLE 50

struct GPUContact {
    vec3 normal;
    uint other;
    vec3 pos;
    float _1;
};

struct GPUParticle {
    vec3 pos;
    float radius;
    vec3 gravity_acc;
    float mass;
    uint contact_count;
    uint _1, _2, _3;
    GPUContact contacts[MAX_CONTACTS_PER_PARTICLE];
};

layout(local_size_x = WORKERS, local_size_y = 1) in;
layout (binding = 0, offset = 0) uniform atomic_uint ac;
layout (std430, binding=0) volatile buffer shader_data {
    uint particles_count;
    uint _1, _2, _3;
    GPUParticle particles[];
};

void main() {
	uint index = gl_GlobalInvocationID.x;

	uint start = index * particles_count / WORKERS;
	uint end = (index + 1) * particles_count / WORKERS;

	for (uint i = start; i < end; i++) {
	    particles[i].gravity_acc = vec3(0, 0, 0);
        particles[i]._1 = (i);
        particles[i].contact_count = 0;
        for (uint j = 0; j < particles_count; j++) {
            if (i == j) continue;

            vec3 r = particles[j].pos - particles[i].pos;
            float dist2 = dot(r, r);
            float dist = sqrt(dist2);
            vec3 unit_r = r / dist;

            float radius_sum = particles[i].radius + particles[j].radius;
            float depth = radius_sum - dist;
            if (depth > 0 && i > j) {
                uint write_index = atomicAdd(particles[i].contact_count, 1);
                if (write_index >= MAX_CONTACTS_PER_PARTICLE) {
                    continue;
                }
                particles[i].contacts[write_index].normal = normalize(r) * depth;
                particles[i].contacts[write_index].other = j;
                float center_dist = particles[i].radius - depth / 2;
                particles[i].contacts[write_index].pos = particles[i].pos + unit_r * center_dist;
            }
        }
	}
}