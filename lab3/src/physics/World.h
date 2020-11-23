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
#include <glad/glad.h>


#define MAX_CONTACTS_PER_PARTICLE 103

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

struct GPUParticle {
    glm::vec3 pos;
    float radius;
    glm::vec3 gravity_acc;
    float mass;
    glm::uint contact_count;
    glm::uint contacts[MAX_CONTACTS_PER_PARTICLE];
};

struct GPUInput {
    glm::uint particles_count;
    glm::uint _1, _2, _3;
    GPUParticle particles[];

    static glm::uint get_size(glm::uint gpu_particle_count);
    void read_from(std::vector<Particle*> &src);
    void write_to(std::vector<Particle *> &dst, std::vector<Contact> &contacts);
};

struct Particle {
    // State
    glm::vec3 pos, vel = glm::vec3(0, 0, 0), ang_vel;
    glm::mat4 rot = glm::mat4(1.0f);
    // Accumulator
    glm::vec3 impulse, ang_impulse;
    glm::vec3 gravity_acc;
    std::unordered_map<Particle*, Contact*> contacts;
    void *userdata;

    float mass, radius, moi;
    Particle(float mass, float radius);
    void integrate(float dt);

    void reset();

    float draw_scale = 1.0f;
};

struct World {
    int computeProgram;

    std::vector<Particle*> particles;
    // The set of contacts.
    std::vector<Contact> contacts;
    unsigned long steps = 0;
    Constants constants;

    World();
    void load_compute();

    void reset();
    void compute_gpu();
    bool deintersect_all(int iterations);

    bool is_touching(Particle *a, Particle *b, glm::vec3 *normal, glm::vec3 *cpos);

    void find_intersections();
    void solve_intersections();
    void solve_contacts(float dt);
    void gravitate(float dt);
    void integrate(float dt);

    void step(float dt);

    GLuint gpu_particles, atomic_buf;
    GLuint ssbo_block_index;
};


#endif //LAB3_WORLD_H
