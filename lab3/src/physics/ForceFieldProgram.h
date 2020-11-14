//
// Created by astrid on 11/12/20.
//

#ifndef LAB3_FORCEFIELDPROGRAM_H
#define LAB3_FORCEFIELDPROGRAM_H

#include <glm/vec3.hpp>
#include <glm/vec4.hpp>
#include <vector>
#include <glad/glad.h>

struct PhysicsObject {
    glm::vec3 pos;
    float radius;
    glm::vec3 vel;
    float mass;
    glm::vec3 force = glm::vec3(0, 0, 0);
    float _1 = 0;
    glm::vec3 torque = glm::vec3(0, 0, 0);
    float _2 = 0;

    PhysicsObject(){}
    PhysicsObject(float radius,
                  float mass,
                  glm::vec3 pos,
                  glm::vec3 vel = glm::vec3(0, 0, 0))
        : radius(radius)
        , mass(mass)
        , pos(pos)
        , vel(vel)
    { }
};

struct Force {
    glm::vec4 force;
    glm::vec4 torque;
};

GLuint read_program(const std::string& path);

class PhysicsObjectBuffer {
public:
    int objects_count = 0;
    int interactions_count = 0;
    GLuint objects_gpu, interactions;
    void set_objects(std::vector<PhysicsObject> &objects);
    std::vector<PhysicsObject> get_data() const;
};

class ForceFieldProgram {
    GLuint interact, aggregate;
public:
    void run(PhysicsObjectBuffer &ffo, bool overwrite = false) const;
    ForceFieldProgram(const std::string& path);
};

class IntegrationProgram {
    GLuint program;
public:
    void run(PhysicsObjectBuffer &ffo);
};


#endif //LAB3_FORCEFIELDPROGRAM_H
