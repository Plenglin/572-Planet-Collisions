//
// Created by astrid on 11/3/20.
//

#include <glm/gtc/matrix_transform.hpp>
#include <algorithm>
#include "World.h"

#define G 10
#define RESTITUTION 0.1

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
    if (normal != nullptr)
        *normal = normalize(delta) * depth;

    return depth >= 0;
}

void Particle::solve_contacts() {
}

void Particle::reset() {
    impulse = vec3(0, 0, 0);
    group = nullptr;
}

GroupSearchData Particle::get_group_members(std::unordered_set<Particle*> unvisited) {
    std::vector<Particle*> touching{this};
    if (contacts.empty()) {
        return GroupSearchData { true, touching };
    }

    std::vector<Particle*> to_visit{this};
    unvisited.erase(this);
    bool stable = true;
    while (!to_visit.empty()) {
        // Pop element
        auto *particle = to_visit.back();
        to_visit.pop_back();

        // We have visited this?
        if (unvisited.find(particle) == unvisited.end()) {
            continue;
        }
        unvisited.erase(particle);

        // Check neighbors
        for (auto &neighbor_pair : particle->contacts) {
            // If not approaching, then unstable
            if (neighbor_pair.second->state != CONTACT_STATE_STABLE) {
                stable = false;
            }
            auto *neighbor = neighbor_pair.first;
            to_visit.push_back(neighbor);
            touching.push_back(neighbor);
        }
    }

    return GroupSearchData {stable, touching};
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

            auto contact = a->contacts.find(b);
            if (contact != a->contacts.end() && contact->second->state == CONTACT_STATE_APPROACHING)
                continue;

            vec3 unit_r = normalize(r);
            float specific_acc = G / dist2;

            a->vel -= dt * unit_r * specific_acc * b->mass;
            b->vel += dt * unit_r * specific_acc * a->mass;
        }
    }
}

void World::step(float dt) {
    reset();
    find_intersections();
    //solve_intersections();
    gravitate(dt);

    solve_contacts();
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
        if (!it->a->is_touching(it->b, &it->normal)) {
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

            glm::vec3 normal;
            if (!a->is_touching(b, &normal))
                continue;

            contacts.emplace_back(a, b, normal);  // union of contacts for all iterations
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

void World::solve_contacts() {
    // New contacts destabilize their contact group

    // Calculate instantaneous momentums
    for (int i = 0; i < contacts.size(); i++) {
        for (auto c : contacts) {
            c.solve_momentum();
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

void World::deintersect_all() {
    do {
        contacts.clear();
        find_intersections();
        solve_intersections();
    } while (!contacts.empty());
}

void World::create_groups() {
    if (particles.empty()) return;

    std::unordered_set<Particle*> unvisited(particles.begin(), particles.end());

    while (!unvisited.empty()) {
        auto p = *unvisited.begin();
        unvisited.erase(p);

        auto result = p->get_group_members(unvisited);
        if (result.stable) {

        }
    }
}

void Contact::solve_momentum() {
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
    if (dot(va, va) < 0.03) {
        state = CONTACT_STATE_STABLE;
        return;
    }

    state = CONTACT_STATE_APPROACHING;
    auto pa = a->mass * va_normal;

    // Stolen from https://en.wikipedia.org/wiki/Coefficient_of_restitution#Derivation with ub = 0
    auto va_final = (pa - b->mass * RESTITUTION * va_normal) / (a->mass + b->mass);
    auto pa_final = a->mass * va_final;
    auto pb_final = pa - pa_final;
    auto vb_final = pb_final / b->mass;

    // Tangent velocity
    auto va_tangent = va - unit_normal * va_normal;

    // Perform impulse calculation
    auto dva = unit_normal;
    dva *= va_final;
    auto dvb = unit_normal;
    dvb *= vb_final;
    a->vel = va_tangent + dva + b->vel;
    b->vel += dvb;
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

Contact::Contact(Particle *a, Particle *b, glm::vec3 normal) : a(a), b(b), normal(normal) {

}

void Body::integrate(float dt) {
    Particle::integrate(dt);
    rotation = rotate(rotation, dt, ang_vel);
}

void Body::apply_acc(glm::vec3 r, glm::vec3 da) {
    Particle::apply_acc(r, da);
    auto ang_impulse = cross(r, da);
}

ContactGroup::ContactGroup(int id) : id(id), active(true), grav_force(0, 0, 0) {

}

void ContactGroup::calculate_params() {

}
