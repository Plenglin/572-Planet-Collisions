//
// Created by astrid on 11/3/20.
//

#include <glm/gtc/matrix_transform.hpp>
#include "World.h"

#define G 30

using namespace glm;

void Particle::integrate(float dt) {
    vel += impulse / mass;
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
    solve_intersections();
    solve_contacts();

    gravitate(dt);
    integrate(dt);
}

void World::reset() {
    for (auto &p : particles) {
        p->impulse = vec3(0, 0, 0);
    }
    contacts.clear();
}

void World::solve_intersections() {
    // Find all contacts
    for (auto ita = particles.begin(); ita != particles.end(); ita++) {
        auto *a = *ita;
        for (auto itb = particles.begin(); itb != ita; itb++) {
            auto *b = *itb;

            glm::vec3 normal;
            if (!a->is_touching(b, &normal))
                continue;

            auto contact = Contact{a, b, normal};
            contacts.insert(contact);  // union of contacts for all iterations
        }
    }
}

void World::solve_contacts() {
    for (int i = 0; i < contacts.size(); i++) {
        for (auto &c : contacts) {
            c.solve_momentum();
        }
    }
    for (auto &c : contacts) {
        c.deintersect();
    }
}

void Contact::solve_momentum() const {
    // Initial momentums along normal
    auto unit_normal = normalize(normal);
    auto va_normal = dot(unit_normal, a->vel);
    auto vb_normal = dot(unit_normal, b->vel);

    // Do nothing if the particles are leaving each other
    if (va_normal > 0 && vb_normal < 0) {
        return;
    }
    auto pa = a->mass * va_normal;
    auto pb = b->mass * vb_normal;
    auto pnet = pa + pb;

    // Stolen from https://en.wikipedia.org/wiki/Coefficient_of_restitution#Derivation
    auto va_final = (pa + pb + b->mass * 0.9 * (vb_normal - va_normal)) / (a->mass + b->mass);
    auto pa_final = a->mass * va_final;
    auto pb_final = pnet - pa_final;
    auto vb_final = pb_final / b->mass;

    // Tangent velocities
    auto va_tangent = a->vel - unit_normal * va_normal;
    auto vb_tangent = b->vel - unit_normal * vb_normal;

    // Apply "friction"
    va_tangent *= 0.9;
    vb_tangent *= 0.9;

    // Perform impulse calculation
    auto dva = unit_normal;
    dva *= va_final;
    auto dvb = unit_normal;
    dvb *= vb_final;
    a->vel = va_tangent + dva;
    b->vel = vb_tangent + dvb;
}

bool Contact::operator==(Contact other) const {
    return a == other.a && b == other.b;
}

void Contact::deintersect() const {
    // De-intersect particles
    auto a_deflection = normal * b->mass / (a->mass + b->mass);
    auto b_deflection = normal - a_deflection;
    a->pos += a_deflection;
    b->pos -= b_deflection;
}

void Body::integrate(float dt) {
    Particle::integrate(dt);
    rotation = rotate(rotation, dt, ang_vel);
}

void Body::apply_acc(glm::vec3 r, glm::vec3 da) {
    Particle::apply_acc(r, da);
    auto ang_impulse = cross(r, da);
}