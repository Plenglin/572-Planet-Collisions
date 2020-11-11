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


struct Particle;

struct Contact {
    Contact(Particle *pParticle, Particle *pParticle1, glm::vec3 vec);

    Particle *a, *b;
    // Collision normal oriented from b to a
    glm::vec3 normal, unit_normal;
    // Intermediate calculations
    glm::vec3 normal_force;
    float normal_force_mag;
    // Are the particles moving into each other?
    bool approaching = false;
    // Are the particles being pushed into each other?
    bool pushing = false;

    void deintersect() const;
    void solve_momentum(float dt);
    void apply_friction();
    void calculate_normal_force();
    bool operator==(Contact other) const;

    void apply_normal_force();
};

// For sorting the contacts in deepest to shallowest order
struct ContactDepthComparator {
    bool operator()(Contact a, Contact b) {
        return glm::length(a.normal) > glm::length(b.normal);
    }
};

namespace std {
    template<>
    struct hash<Contact> {
        std::size_t operator()(const Contact &c) const {
            return ((std::size_t) c.a + 31) + (std::size_t) c.b;
        }
    };
}

struct ContactGroup;

struct GroupSearchData {
    // If this group is stable (i.e. internal gravitation wouldn't change anything)
    bool stable;

    // Particles that are touching.
    std::vector<Particle *> touching;
};

struct Particle {
    // State
    glm::vec3 pos, vel;
    // Accumulator
    glm::vec3 force;
    std::unordered_map<Particle*, Contact*> contacts;
    ContactGroup *group;

    float mass, radius;
    virtual void integrate(float dt);
    virtual void apply_acc(glm::vec3 r, glm::vec3 da);
    bool is_touching(Particle *other, glm::vec3 *normal);

    // Attempts to deintersect this particle alone, given the contacts.
    // May not fully succeed.
    void deintersect();

    void reset();

    // DFS for particles in this group
    GroupSearchData get_group_members(std::unordered_set<Particle *> unvisited);
};

// Represents a bunch of particles touching each other.
struct ContactGroup {
    int id;
    std::unordered_set<Particle*> particles;
    glm::vec3 pos;
    glm::vec3 grav_force;
    float mass;
    bool active;
    explicit ContactGroup(int id);
    void calculate_params();
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
    std::vector<Contact> contacts;
    std::vector<ContactGroup> groups;

    World();

    void reset();
    // Returns false if there are still things to deintersect.
    bool deintersect_all(int max_steps);

    void find_intersections();
    void solve_intersections();
    void solve_contacts();
    void create_groups();
    void gravitate();
    void integrate(float dt);

    void step(float dt);
};


#endif //LAB3_WORLD_H
