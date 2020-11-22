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

struct GPUContact {
    glm::vec3 normal;
    uint other;
    glm::vec3 pos;
    uint _1;
};

struct GPUParticle {
    glm::vec3 pos;
    float radius;
    glm::vec3 gravity_acc;
    float mass;
    uint contact_count;
    GPUContact contacts[MAX_CONTACTS_PER_PARTICLE];
};

typedef std::unordered_map<Particle*, std::unordered_map<Particle*, GPUContact>> ContactIndex;

struct GPUInput {
    uint particles_count;
    uint _1, _2, _3;
    GPUParticle particles[];

    static uint get_size(uint gpu_particle_count);
    void read_from(std::vector<Particle*> &src);
    void write_to(float dt, std::vector<Particle *> &dst, ContactIndex &contacts);
};

struct Particle {
    // State
    glm::vec3 pos, vel = glm::vec3(0, 0, 0), ang_vel;
    glm::mat4 rot = glm::mat4(1.0f);
    // Accumulator
    glm::vec3 impulse, ang_impulse;
    std::unordered_map<Particle*, Contact*> contacts;
    void *userdata;

    float mass, radius, moi;
    Particle(float mass, float radius);
    void integrate(float dt);
    bool is_touching(Particle *other, glm::vec3 *normal, glm::vec3 *cpos);

    void reset();

    float draw_scale = 1.0f;
};

struct World {
    int computeProgram;

    std::vector<Particle*> particles;
    // GPU contacts.
    ContactIndex contact_index;
    // The set of contacts.
    std::vector<Contact> contacts;
    unsigned long steps = 0;
    Constants constants;

    World();
    void load_compute();

    void reset();
    bool deintersect_all(int iterations);

    void find_intersections();
    void solve_intersections();
    void solve_contacts(float dt);
    void calculate_gpu(float dt);
    void integrate(float dt);

    void step(float dt);

    GLuint gpu_particles, atomic_buf;
    GLuint ssbo_block_index;
};


#endif //LAB3_WORLD_H
