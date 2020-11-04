//
// Created by astrid on 11/3/20.
//

#include <glm/gtc/matrix_transform.hpp>
#include "World.h"

#define G 0.00001

using namespace glm;

void Particle::integrate(float dt) {
    pos += vel * dt;
}

void Particle::apply_acc(glm::vec3 r, glm::vec3 da) {
    vel += da;
}

World::World() = default;

void World::integrate(float dt) {
    for (auto & particle : particles) {
        particle->integrate(dt);
    }
}

void World::gravitate(float dt) {
    for (auto ita = particles.begin(); ita != particles.end(); ita++) {
        auto *a = *ita;
        for (auto itb = particles.begin(); itb != ita; itb++) {
            auto *b = *itb;
            vec3 r = a->pos - b->pos;
            vec3 unit_r = normalize(r);
            float force_mag = G * a->mass * b->mass / dot(r, r);

            a->vel += dt * unit_r * force_mag / a->mass;
            b->vel += dt * -unit_r * force_mag / b->mass;
        }
    }
}

void World::step(float dt) {
    //group();
    //solve_groups();
    //gravitate(dt);
    integrate(dt);
}

void World::group() {

}

void World::solve_groups() {

}

void Body::integrate(float dt) {
    Particle::integrate(dt);
    rotation = rotate(rotation, dt, ang_vel);
}

void Body::apply_acc(glm::vec3 r, glm::vec3 da) {
    Particle::apply_acc(r, da);
    auto ang_impulse = cross(r, da);
}
