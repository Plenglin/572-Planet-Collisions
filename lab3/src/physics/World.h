//
// Created by astrid on 11/3/20.
//

#ifndef LAB3_WORLD_H
#define LAB3_WORLD_H


#include <vector>
#include <glm/glm.hpp>
#include <memory>
#include <set>

struct Particle;

struct Contact {
    Particle *a, *b;
    // a's position - b's position
    glm::vec3 normal;
};

// For sorting the contacts in deepest to shallowest order
struct ContactComparator {
    bool operator()(Contact a, Contact b) {
        return glm::length(a.normal) < glm::length(b.normal);
    }
};

struct Particle {
    // State
    glm::vec3 pos, vel;

    float mass, radius;
    virtual void integrate(float dt);
    virtual void apply_acc(glm::vec3 r, glm::vec3 da);
    bool is_touching(Particle *other, glm::vec3 *normal);
    void solve_contacts();
};

struct Body : public Particle {
    glm::vec3 ang_vel;

    // Store rotation in this matrix. Quaternion would technically be better but idk how to ¯\_(ツ)_/¯
    glm::mat4 rotation;
    void integrate(float dt) override;
    void apply_acc(glm::vec3 r, glm::vec3 da) override;
};

struct World {
    std::vector<Particle*> particles;
    // The set of contacts.
    std::set<Contact, ContactComparator> contacts;

    World();

    void reset();
    void find_contacts();
    void solve_contacts();
    void gravitate(float dt);
    void integrate(float dt);

    void step(float dt);
};


#endif //LAB3_WORLD_H
