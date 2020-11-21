#version 450 
#extension GL_ARB_shader_storage_buffer_object : require

#define ACC vec3(0,-9.81,0)
#define gravity .00007667
#define numberOfSpheres 552
layout(local_size_x = 256, local_size_y = 1) in;	


layout (std430, binding=0) volatile buffer shader_data
{ 
  vec4 pos[numberOfSpheres];
  vec4 vel[numberOfSpheres];
  vec4 testbit;
 
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



void main() 
	{
	uint index = uint(256) * gl_WorkGroupID.x + gl_LocalInvocationID.x;
	if(index >= numberOfSpheres) return;

		vec3 velocity = vec3(vel[index].xyz);
		vec3 position = vec3(pos[index].xyz);
		int mass = int(pos[index].w);
		vec3 force = vec3(0,0,0);
		barrier();
		for(int i = 0; i < numberOfSpheres; i++){
			if( index != i){
				vec3 forces = (pos[i].xyz - position);
				if( length(forces) > 1e-5){
					//current working strategy
					vec3 acting = (pos[i].w/2.0) * (1.0/pow(length(forces),2)) *normalize(forces);
					force += acting;
				}
			}
		}
		barrier();
		
		force += velocity;

		
		force = force * vel[index].w * gravity;
		vel[index]= vec4(force,vel[index].w);


}