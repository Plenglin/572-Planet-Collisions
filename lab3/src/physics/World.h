//
// Created by astrid on 11/3/20.
//

#ifndef LAB3_WORLD_H
#define LAB3_WORLD_H


#include <vector>
#include <glm/glm.hpp>
#include <memory>

struct Group;

struct Particle {
    // State
    glm::vec3 pos, vel;
    Group *group;

    float mass, radius;
    virtual void integrate(float dt);
    virtual void apply_acc(glm::vec3 r, glm::vec3 da);
    bool is_touching(Particle *other);
};

struct Body : public Particle {
    glm::vec3 ang_vel;

    // Store rotation in this matrix. Quaternion would technically be better but idk how to ¯\_(ツ)_/¯
    glm::mat4 rotation;
    void integrate(float dt) override;
    void apply_acc(glm::vec3 r, glm::vec3 da) override;
};

struct Group {
    std::vector<Particle*> particles;
    bool valid;
    void merge(Group *other);
    void solve();
};

struct World {
    std::vector<Particle*> particles;
    std::vector<std::unique_ptr<Group>> groups;

    World();

    void reset();
    void group();
    void solve_groups();
    void gravitate(float dt);
    void integrate(float dt);

    void step(float dt);
};


#endif //LAB3_WORLD_H
