#version 450 
#extension GL_ARB_shader_storage_buffer_object : require

#define G 4
#define numberOfSpheres 552
#define WORKERS 256
#define MAX_CONTACTS_PER_PARTICLE 102

struct GPUParticle {
    vec3 pos;
    float radius;
    vec3 gravity_acc;
    float mass;
    uint contact_count;
    uint contacts[MAX_CONTACTS_PER_PARTICLE];
};

layout(local_size_x = WORKERS, local_size_y = 1) in;


layout (std430, binding=0) volatile buffer shader_data {
    uint particles_count;
    uint _1, _2, _3;
    GPUParticle particles[];
};

vec4 minus( vec4 v1, vec4 v2){
	vec4 r;
	r.x = v1.x - v2.x;
    r.y = v1.y - v2.y;
    r.z = v1.z - v2.z;
	r.w = v1.w;
    return r;

}
float dotProduct(vec4 v1, vec4 v2){
	return v1.x * v2.x + v1.y * v2.y + v1.z * v2.z;
}
float distanceSquared(vec4 v1, vec4 v2){
	vec4 delta = minus(v2,v1);
	return dotProduct(delta,delta);
}
bool doTheyCollide( vec4 pos1, vec4 pos2){
	float rSquared = pos1.w +pos2.w;
	rSquared *= rSquared;
	return distanceSquared(pos1,pos2) < rSquared;
}
vec4 scale( vec4 v, float a){
	vec4 r;
	r.x = v.x * a;
    r.y = v.y * a;
    r.z = v.z * a;
    return r;
}

vec4 projectUonV( vec4 u, vec4 v) {
    return scale(v, dotProduct(u, v) / dotProduct(v, v));
}

vec4 kaboom(vec4 vel1,vec4 vel2, vec4 pos1, vec4 pos2){
	vec4 newvel1 = vel1;
	newvel1 += projectUonV(vel2, minus(pos2, pos1));
    newvel1 -= projectUonV(vel1, minus(pos1, pos2));
	return newvel1;

}



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