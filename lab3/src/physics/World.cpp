//
// Created by astrid on 11/3/20.
//

#include <glm/gtc/matrix_transform.hpp>
#include <algorithm>
#include "World.h"
#include<iostream>

using namespace glm;
using namespace std;

void Particle::integrate(float dt) {
    vel += impulse / mass;
    ang_vel += ang_impulse / moi;

    pos += vel * dt;
    auto ang_vel_mag = length(ang_vel);
    if (ang_vel_mag > 1e-3) {
        rot = rotate(glm::mat4(1.0f), ang_vel_mag * dt, ang_vel / ang_vel_mag) * rot;
    }
}

bool Particle::is_touching(Particle *b, glm::vec3 *normal, glm::vec3 *cpos) {
    vec3 delta = pos - b->pos;
    float dist = length(delta);
    vec3 unit_delta = delta / dist;
    float radius_sum = radius + b->radius;
    float depth = radius_sum - dist;
    if (normal != nullptr) {
        *normal = normalize(delta) * depth;
    }
    if (cpos != nullptr) {
        auto center_dist = radius - depth / 2;
        *cpos = pos + unit_delta * center_dist;
    }
    return depth >= 0;
}

void Particle::reset() {
    impulse = vec3(0, 0, 0);
    ang_impulse = vec3(0, 0, 0);
}

Particle::Particle(float mass, float radius) : mass(mass), radius(radius), moi(2.0f/5 * mass * radius * radius) {

}

World::World() = default;

void World::integrate(float dt) {
    for (auto & particle : particles) {
        particle->integrate(dt);
    }
}

//position, mass 552 particles
void World::gravitate(float dt) {
    int numberOfParticles = 0;
    for (auto ita = particles.begin(); ita != particles.end(); ita++) {
        auto *a = *ita;
        numberOfParticles++;
        for (auto itb = particles.begin(); itb != ita; itb++) {
            auto *b = *itb;
            vec3 r = a->pos - b->pos;
            float dist2 = dot(r, r);
            if (dist2 < 1e-5)
                continue;

            /*
            auto contact = a->contacts.find(b);
            if (contact != a->contacts.end() && contact->second->state == CONTACT_STATE_APPROACHING)
                continue;
                */

            vec3 unit_r = normalize(r);
            float specific_acc = constants.G / dist2;

            a->vel -= dt * unit_r * specific_acc * b->mass;
            b->vel += dt * unit_r * specific_acc * a->mass;
        }
    }
    cout << numberOfParticles<< endl;
}

void World::step(float dt) {
    reset();
    find_intersections();
    //solve_intersections();
    gravitate(dt);//Move to cs

    solve_contacts(dt);
    integrate(dt);
    steps++;
}

void World::reset() {
    for (auto &p : particles) {
        p->reset();
    }
}

void World::find_intersections() {
    for (auto it = contacts.begin(); it != contacts.end();) {
        if (!it->a->is_touching(it->b, &it->normal, nullptr)) {
            it->a->contacts.erase(it->b);
            it->b->contacts.erase(it->a);
            it = contacts.erase(it);
            continue;
        }
        it->lifetime++;
        ++it;
    }

    // Find all contacts
    for (auto ita = particles.begin(); ita != particles.end(); ita++) {
        auto *a = *ita;
        for (auto itb = particles.begin(); itb != ita; itb++) {
            auto *b = *itb;

            if (a->contacts.find(b) != a->contacts.end())
                continue;  // Contact already exists

            glm::vec3 normal, cpos;
            if (!a->is_touching(b, &normal, &cpos))
                continue;

            contacts.emplace_back(a, b, normal, cpos);  // union of contacts for all iterations
            auto *ptr = &contacts.back();
            a->contacts[b] = ptr;
            b->contacts[a] = ptr;
        }
    }
    
}

void World::solve_intersections() {
    std::sort(contacts.begin(), contacts.end(), ContactDepthComparator());
    for (auto &c : contacts) {
        c.deintersect();
    }
}

void World::solve_contacts(float dt) {
    // New contacts destabilize their contact group

    // Calculate instantaneous momentums
    float mdt = dt / contacts.size();
    for (int i = 0; i < contacts.size(); i++) {
        for (auto c : contacts) {
            c.solve_momentum(mdt, constants);
        }
    }

    // Zero colliding velocities if they haven't been zeroed yet
    for (auto c : contacts) {
        auto dv = c.a->vel - c.b->vel;
        if (dot(dv, c.normal) < 0) {
            auto pa = c.a->vel * c.a->mass;
            auto pb = c.b->vel * c.b->mass;
            auto pnet = pa + pb;
            if (dot(pnet, pnet) < 1e-5) {
                c.a->vel = c.b->vel = vec3(0, 0, 0);
                continue;
            }
            auto unit_pnet = normalize(pnet);

            c.a->vel = c.a->vel * unit_pnet * dot(unit_pnet, c.a->vel);
            c.b->vel = c.b->vel * unit_pnet * dot(unit_pnet, c.b->vel);
        }
    }
}

bool World::deintersect_all(int iterations) {
    do {
        contacts.clear();
        find_intersections();
        solve_intersections();
    } while (!contacts.empty() && --iterations > 0);
    return !contacts.empty();
}

void Contact::solve_momentum(float dt, Constants &constants) {
    if (state == CONTACT_STATE_STABLE) {
        return;
    }

    // Calculate with b as reference frame at rest.
    auto va = a->vel - b->vel;

    // Note that normal points from b -> a
    auto unit_normal = normalize(normal);
    auto va_normal = dot(unit_normal, va);  // va's normal component

    // If leaving, do nothing
    if (va_normal > 0) {
        state = CONTACT_STATE_LEAVING;
        return;
    }

    // If small relative velocity, it's "stable" and do nothing
    if (dot(va, va) < (constants.STABLE_THRESH * constants.STABLE_THRESH)) {
        state = CONTACT_STATE_STABLE;
        return;
    }

    state = CONTACT_STATE_APPROACHING;
    auto pa = a->mass * va_normal;

    // Stolen from https://en.wikipedia.org/wiki/Coefficient_of_restitution#Derivation with ub = 0
    auto va_final = (pa - b->mass * constants.RESTITUTION * va_normal) / (a->mass + b->mass);
    auto pa_final = a->mass * va_final;

    auto linear_imp_mag = pa_final - pa;
    auto linear_imp = unit_normal * linear_imp_mag;

    // Distance from collision to center
    auto ra = pos - a->pos;
    auto rb = pos - b->pos;
    // Linear-only velocity difference
    auto va_tangent_linear = va - unit_normal * va_normal;
    // Surface velocity difference
    auto surface_va = cross(b->ang_vel, rb) - cross(a->ang_vel, ra);
    // Real surface velocity difference
    auto va_tangent = va_tangent_linear + surface_va;
    auto va_tangent_mag = length(va_tangent);
    if (va_tangent_mag > constants.STABLE_THRESH) {
        auto friction_impulse = va_tangent * (linear_imp_mag * constants.COLLISION_IMPULSE_TO_FRICTION * dt / va_tangent_mag);
        linear_imp += friction_impulse;

        // Angular impulse
        a->ang_vel += cross(friction_impulse, ra) / a->moi;
        b->ang_vel += cross(friction_impulse, rb) / b->moi;
    }

    // Perform impulse calculation
    a->vel += linear_imp / a->mass;
    b->vel -= linear_imp / b->mass;
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

Contact::Contact(Particle *a, Particle *b, glm::vec3 normal, glm::vec3 pos) : a(a), b(b), normal(normal), pos(pos) {

}
