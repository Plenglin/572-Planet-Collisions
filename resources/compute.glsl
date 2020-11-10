#version 450 
#extension GL_ARB_shader_storage_buffer_object : require

#define ACC vec3(0,-9.81,0)

layout(local_size_x = 256, local_size_y = 1) in;	


layout (std430, binding=0) volatile buffer shader_data
{ 
  vec4 pos[100];
  vec4 vel[100];
  vec4 trackers[100];
  vec4 groupdata;
 
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
	vec3 velocity = vec3(vel[index].x,vel[index].y,vel[index].z);
	vec3 position = vec3(pos[index].x,pos[index].y,pos[index].z);
	velocity = velocity + vec3(0,-9.81,0) * 0.001f;
	position = position + velocity * groupdata.x;
	//setting the updated velocity, odd change
	vel[index] = vec4(velocity, vel[index].w);
	pos[index] = vec4(position,pos[index].w);
	barrier();
	//original method to keep track of changes
	vec4 newVelocity = vec4(velocity, vel[index].w);
	if(index < 100){




		for(int i = 0; i < 100; i++){
			if(index != i &&(trackers[index].x == 0 || vel[index].w != vel[i].w)){
				if(doTheyCollide(pos[index],pos[i])){
					newVelocity = kaboom(vec4(velocity,vel[index].w), vel[i], pos[index], pos[i]);
					trackers[index].x=0;
					trackers[i].x=0;
				}
			}
		}
		barrier();
		//set the new velocity
		vel[index] = newVelocity;

		if((pos[index].y - pos[index].w) < -5){
			vel[index].y = abs(vel[index].y);
		}
		if((pos[index].y + pos[index].w) > 6){
			vel[index].y = -abs(vel[index].y);
		}

		if((pos[index].x - pos[index].w) < -5){
			vel[index].x = abs(vel[index].x);
		}

		if((pos[index].x + pos[index].w) > 5){
			vel[index].x = -abs(vel[index].x);
		}

		if((pos[index].z - pos[index].w) < -25){
			vel[index].z = abs(vel[index].z);
		}
		
		if((pos[index].z - pos[index].w) > -15){
			vel[index].z = -abs(vel[index].z);
		}
	}
}