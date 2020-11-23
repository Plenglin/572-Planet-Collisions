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
	    for (uint j = 0; j < particles_count; j++) {
            vec3 r = particles[j].pos - particles[i].pos;
            float dist2 = dot(r, r);
            if (dist2 < 1e-5)
                continue;

            vec3 unit_r = normalize(r);
            float specific_acc = G / dist2;

            particles[i].gravity_acc += unit_r * (specific_acc * particles[j].mass);
	    }
	}
}