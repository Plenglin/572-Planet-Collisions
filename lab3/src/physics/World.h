//
// Created by astrid on 11/3/20.
//

#ifndef LAB3_WORLD_H
#define LAB3_WORLD_H


#include <vector>
#include <glm/glm.hpp>
#include <memory>
#include <set>
#include <unordered_set>
#include <unordered_map>
#include "ForceFieldProgram.h"


struct World {
    std::vector<PhysicsObject> objects;
    std::vector<ForceFieldProgram> fields;
    PhysicsObjectBuffer object_buffer;
    ForceFieldProgram gravity = ForceFieldProgram("../resources/field_gravity.glsl");
    //ForceFieldProgram spring = ForceFieldProgram("../resources/field_spring.glsl");

    World();
    void refresh_objects();
    void step(float dt);
};


#endif //LAB3_WORLD_H
