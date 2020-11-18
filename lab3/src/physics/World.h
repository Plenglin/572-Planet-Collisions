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


struct Constants {
    float G = 5.0f;
    float RESTITUTION = 0.99f;
    float COLLISION_IMPULSE_TO_FRICTION = 5.0f;
    float STABLE_THRESH = 0.9f;
};

struct Particle;

enum ContactState {
    CONTACT_STATE_APPROACHING,
    CONTACT_STATE_STABLE,
    CONTACT_STATE_LEAVING
};

struct Contact {
    Contact(Particle *a, Particle *b, glm::vec3 normal, glm::vec3 pos);

    Particle *a, *b;
    // Collision normal oriented from b to a
    glm::vec3 normal;
    // Collision position
    glm::vec3 pos;
    unsigned long lifetime = 0;
    ContactState state = CONTACT_STATE_APPROACHING;

    void deintersect() const;
    void solve_momentum(float dt, Constants &constants);
    bool operator==(Contact other) const;
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
    glm::vec3 pos, vel = glm::vec3(0, 0, 0), ang_vel;
    glm::mat4 rot = glm::mat4(1.0f);
    // Accumulator
    glm::vec3 impulse, ang_impulse;
    std::unordered_map<Particle*, Contact*> contacts;
    ContactGroup *group;
    void *userdata;

    float mass, radius, moi;
    Particle(float mass, float radius);
    virtual void integrate(float dt);
    virtual void apply_acc(glm::vec3 r, glm::vec3 da);
    bool is_touching(Particle *other, glm::vec3 *normal, glm::vec3 *cpos);
    void solve_contacts();

    void reset();

    // DFS for particles in this group
    GroupSearchData get_group_members(std::unordered_set<Particle *> unvisited);

    float draw_scale = 1.0f;
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

struct World {
    std::vector<Particle*> particles;
    // The set of contacts.
    std::vector<Contact> contacts;
    std::vector<ContactGroup> groups;
    unsigned long steps = 0;
    Constants constants;

    World();

    void reset();
    bool deintersect_all(int iterations);

    void find_intersections();
    void solve_intersections();
    void solve_contacts(float dt);
    void create_groups();
    void gravitate(float dt);
    void integrate(float dt);

    void step(float dt);
};


#endif //LAB3_WORLD_H
