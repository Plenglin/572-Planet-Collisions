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

bool Particle::is_touching(Particle *b, glm::vec3 *normal) {
    vec3 delta = pos - b->pos;
    float dist = length(delta);
    float radius_sum = radius + b->radius;
    float depth = radius_sum - dist;
    *normal = normalize(delta) * depth;

    return depth >= 0;
}

void Particle::solve_contacts() {
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
    find_contacts();
    solve_contacts();

    gravitate(dt);
    integrate(dt);
}

void World::reset() {
    contacts.clear();
}

void World::find_contacts() {
    for (auto ita = particles.begin(); ita != particles.end(); ita++) {
        auto *a = *ita;
        for (auto itb = particles.begin(); itb != ita; itb++) {
            auto *b = *itb;

            glm::vec3 normal;
            if (!a->is_touching(b, &normal))
                continue;

            contacts.insert(Contact{a, b, normal});
        }
    }
}

void World::solve_contacts() {
    for (auto &c : contacts) {
        // De-intersect particles
        auto a_deflection = c.normal * c.b->mass / (c.a->mass + c.b->mass);
        auto b_deflection = c.normal - a_deflection;
        c.a->pos += a_deflection;
        c.b->pos -= b_deflection;

        // Initial momentums along normal
        auto unit_normal = normalize(c.normal);
        auto pa = dot(unit_normal, c.b->vel);
        auto pb = dot(unit_normal, c.a->vel);
        auto pnet = pa + pb;

        // Initial kinetic energy (times 2)
        auto sa = length(c.a->vel);
        auto sb = length(c.b->vel);
        auto e2_initial = c.a->mass * sa * sa + c.b->mass * sb * sb;

        // Energy after applying restitution
        auto e2_final = e2_initial * 0.8;

        // Quadratic equation parameters to solve for va
        auto quad_a = c.a->mass * c.a->mass;
        auto quad_b = c.a->mass * (c.b->mass - 2 * pnet);
        auto quad_c = pnet * pnet - e2_final * c.b->mass;

        // Perform calculations for final velocities along normal
        auto discrim = quad_b * quad_b - 4 * quad_a * quad_c;
        auto sol_offset = sqrt(discrim) / (2 * quad_a);
        auto sol_center = quad_b / (2 * quad_a);
        auto va_final = sol_center + sol_offset;
        auto vb_final = (pnet - pa) / c.b->mass;

        // Apply momentum calculation
        auto va_orthogonal = c.a->vel - normalize(c.a->vel) * dot(unit_normal, c.a->vel);
        auto vb_orthogonal = c.b->vel - normalize(c.b->vel) * dot(unit_normal, c.b->vel);
        auto va_normal = unit_normal;
        va_normal *= va_final;
        auto vb_normal = unit_normal;
        vb_normal *= vb_final;
        c.a->vel = va_orthogonal + va_normal;
        c.b->vel = vb_orthogonal + vb_normal;
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
