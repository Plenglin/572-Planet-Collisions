//
// Created by astrid on 11/3/20.
//

#include <glm/gtc/matrix_transform.hpp>
#include "World.h"

#define G 100

using namespace glm;

void Particle::integrate(float dt) {
    pos += vel * dt;
}

void Particle::apply_acc(glm::vec3 r, glm::vec3 da) {
    vel += da;
}

bool Particle::is_touching(Particle *b) {
    vec3 r = pos - b->pos;
    float dist2 = dot(r, r);
    float radius_sum = radius + b->radius;

    return dist2 <= radius_sum * radius_sum;
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
            float dist2 = dot(r, r);
            if (dist2 < 1e-5)
                continue;

            vec3 unit_r = normalize(r);
            float force_mag = G * a->mass * b->mass / dist2;

            a->vel -= dt * unit_r * force_mag / a->mass;
            b->vel += dt * unit_r * force_mag / b->mass;
        }
    }
}

void World::step(float dt) {
    reset();
    group();
    solve_groups();

    gravitate(dt);
    integrate(dt);
}

void World::reset() {
    groups.clear();
    for (auto & particle : particles) {
        particle->group = nullptr;
    }
}

void World::group() {
    for (auto ita = particles.begin(); ita != particles.end(); ita++) {
        auto *a = *ita;
        for (auto itb = particles.begin(); itb != ita; itb++) {
            auto *b = *itb;

            if (!a->is_touching(b))
                continue;

            if (a->group) {
                if (b->group) {
                    a->group->merge(b->group);
                } else {
                    b->group = a->group;
                    a->group->particles.push_back(b);
                }
            } else if (b->group) {
                a->group = b->group;
                b->group->particles.push_back(a);
            } else {
                auto group = std::unique_ptr<Group>(new Group());
                group->particles.push_back(a);
                group->particles.push_back(b);
                groups.push_back(std::move(group));
            }
        }
    }
}

void World::solve_groups() {
    for (auto &group: groups) {
        for (int i = 0; i < group->particles.size(); i++) {
            group->solve();
        }
    }
}

void Body::integrate(float dt) {
    Particle::integrate(dt);
    rotation = rotate(rotation, dt, ang_vel);
}

void Body::apply_acc(glm::vec3 r, glm::vec3 da) {
    Particle::apply_acc(r, da);
    auto ang_impulse = cross(r, da);
}

void Group::merge(Group *other) {
    other->valid = false;
    for (auto particle : other->particles) {
        particle->group = this;
        particles.push_back(particle);
    }
    other->particles.clear();
}

void Group::solve() {
    for (auto ita = particles.begin(); ita != particles.end(); ita++) {
        auto *a = *ita;
        for (auto itb = particles.begin(); itb != ita; itb++) {
            auto *b = *itb;

            if (a->is_touching(b)) {
                // Momentum calculation

            }
        }
    }
}
