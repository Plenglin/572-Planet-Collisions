//
// Created by astrid on 11/3/20.
//

#include <glm/gtc/matrix_transform.hpp>
#include <algorithm>
#include "World.h"

#define G 3000
#define u_S 1.0f
#define u_K 0.5f
#define RESTITUTION 0.3f

using namespace glm;

struct Decomposition {
    vec3 tangent;
    float normal;
};
Decomposition decompose(vec3 v, vec3 unit_normal) {
    float along_normal = dot(v, unit_normal);
    vec3 tangent = v - unit_normal * along_normal;
    return {tangent, along_normal};
}

void Particle::integrate(float dt) {
    vel += force * dt / mass;
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

void Particle::deintersect() {
    if (contacts.empty()) return;

    vec3 weighted_normals = vec3(0, 0, 0);
    float mass_sum = mass;
    for (auto &pair : contacts) {
        auto *other = pair.first;

        // Ensure normal faces towards this particle
        vec3 normal = pair.second->normal;
        if (pair.second->a == other) {
            normal = -normal;
        }

        weighted_normals += normal * other->mass;
        mass_sum += other->mass;
    }

    this->pos += weighted_normals / mass_sum;
}

void Particle::reset() {
    force = vec3(0, 0, 0);
    contacts.clear();
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
            if (!neighbor_pair.second->approaching) {
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

void World::gravitate() {
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

            a->force -= unit_r * force_mag;
            b->force += unit_r * force_mag;
        }
    }
}

void World::step(float dt) {
    reset();
    find_intersections();
    solve_intersections();

    gravitate();
    solve_contacts();

    for (int i = 0; i < 3; i++) {
        integrate(dt / 3);
    }
}

void World::reset() {
    for (auto &p : particles) {
        p->reset();
    }
    contacts.clear();
}

void World::find_intersections() {
    // Find all contacts
    
    for (auto ita = particles.begin(); ita != particles.end(); ita++) {
        auto *a = *ita;
        for (auto itb = particles.begin(); itb != ita; itb++) {
            auto *b = *itb;

            glm::vec3 normal;
            if (!a->is_touching(b, &normal))
                continue;

            auto contact = Contact(a, b, normal);
            contacts.push_back(contact);  // union of contacts for all iterations
            auto *ptr = &contacts.back();
            a->contacts[b] = ptr;
            b->contacts[a] = ptr;

        }
    }
    
}

void World::solve_intersections() {
    for (auto p : particles) {
        p->deintersect();
    }
}

void World::solve_contacts() {
    for (int i = 0; i < contacts.size(); i++) {
        for (auto c : contacts) {
            c.solve_momentum(0);
        }
    }
    for (auto c : contacts) {
        c.calculate_normal_force();
    }
    for (auto c : contacts) {
        c.apply_normal_force();
    }
}

bool World::deintersect_all(int max_steps) {
    do {
        contacts.clear();
        find_intersections();
        solve_intersections();
    } while (!contacts.empty() && max_steps-- > 0);
    return contacts.empty();
}

void World::create_groups() {
    std::unordered_set<Particle*> unvisited(particles.begin(), particles.end());

}

void Contact::solve_momentum(float dt) {
    // Calculate with b as reference frame.
    auto va = a->vel - b->vel;

    // Note that normal points from b -> a
    auto va_normal = dot(unit_normal, va);  // va's normal component

    // Do nothing if a is leaving b
    if (va_normal > 0) {
        approaching = false;
        return;
    }
    approaching = true;
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

    a->pos += a->vel * dt;
    b->pos += b->vel * dt;
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

Contact::Contact(Particle *a, Particle *b, glm::vec3 normal)
    : a(a), b(b), normal(normal), unit_normal(normalize(normal)), approaching(false) {

}

void Contact::calculate_normal_force() {
    normal_force_mag = dot(b->force - a->force, unit_normal);
    normal_force = unit_normal * normal_force_mag;
    pushing = normal_force_mag < 0;
}

void Contact::apply_normal_force() {
    if (!pushing) return;
    a->force += normal_force;
    b->force -= normal_force;
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
