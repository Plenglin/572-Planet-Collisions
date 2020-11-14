//
// Created by astrid on 11/3/20.
//

#include <glm/gtc/matrix_transform.hpp>
#include <algorithm>
#include "World.h"

#define G 30
#define V_EPSILON 0.1f
#define TANGENT_ALPHA 0.99f
#define RESTITUTION 0.3f

using namespace glm;

World::World() {
}

void World::step(float dt) {
}

void World::refresh_objects() {
    object_buffer.set_objects(objects);
}
