//
// Created by astrid on 11/3/20.
//

#include <glm/gtc/matrix_transform.hpp>
#include <algorithm>
#include "World.h"
#include "../Program.h"
#include "../GLSL.h"
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

void World::gravitate(float dt) {
    for (auto & particle : particles) {
        auto dv = particle->gravity_acc * dt;
        particle->vel += dv;
    }
}

void World::step(float dt) {
    reset();
    compute_gpu();

    find_intersections();
    gravitate(dt);

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
        if (!is_touching(it->a, it->b, &it->normal, nullptr)) {
            it->a->contacts.erase(it->b);
            it->b->contacts.erase(it->a);
            it = contacts.erase(it);
            continue;
        }
        it->lifetime++;
        ++it;
    }

    // Find all contacts
    for (auto &pa : contact_index) {
        auto a = pa.first;
        auto &bmap = pa.second;
        for (auto &pb : bmap) {
            auto b = pb.first;
            auto &gpuc = pb.second;
            if (a->contacts.find(b) != a->contacts.end())
                continue;  // Contact already exists

            glm::vec3 normal, cpos;
            if (!is_touching(a, b, &normal, &cpos))
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
        compute_gpu();
        find_intersections();
        solve_intersections();
    } while (!contacts.empty() && --iterations > 0);
    return !contacts.empty();
}

void World::compute_gpu() {
    {
        // Write
        glBindBuffer(GL_SHADER_STORAGE_BUFFER, gpu_particles);
        auto *ssbo = static_cast<GPUInput *>(glMapBuffer(GL_SHADER_STORAGE_BUFFER, GL_WRITE_ONLY));
        glBindBuffer(GL_SHADER_STORAGE_BUFFER, 0);

        ssbo->upload(particles);

        glBindBuffer(GL_SHADER_STORAGE_BUFFER, gpu_particles);
        glUnmapBuffer(GL_SHADER_STORAGE_BUFFER);
        glBindBuffer(GL_SHADER_STORAGE_BUFFER, 0); // unbind
    }

    glShaderStorageBlockBinding(computeProgram, ssbo_block_index, 0);
    glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 0, gpu_particles);

    glUseProgram(computeProgram);
    glBindBuffer(GL_ATOMIC_COUNTER_BUFFER, atomic_buf);
    glBindBufferBase(GL_ATOMIC_COUNTER_BUFFER, 0, atomic_buf);

    glDispatchCompute(1, 1, 1);
    glMemoryBarrier(GL_SHADER_IMAGE_ACCESS_BARRIER_BIT);
    glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 0, 0);

    {
        // Read
        glBindBuffer(GL_SHADER_STORAGE_BUFFER, gpu_particles);
        auto *ssbo = static_cast<GPUInput *>(glMapBuffer(GL_SHADER_STORAGE_BUFFER, GL_READ_ONLY));
        glBindBuffer(GL_SHADER_STORAGE_BUFFER, 0);

        contact_index.clear();
        ssbo->download(particles, contact_index);

        glBindBuffer(GL_SHADER_STORAGE_BUFFER, gpu_particles);
        glUnmapBuffer(GL_SHADER_STORAGE_BUFFER);
        glBindBuffer(GL_SHADER_STORAGE_BUFFER, 0);
    }
}

void World::load_compute() {
    std::string ShaderString = readFileAsString("../resources/compute.glsl");
    const char* shader = ShaderString.c_str();
    GLuint computeShader = glCreateShader(GL_COMPUTE_SHADER);
    glShaderSource(computeShader, 1, &shader, nullptr);

    GLint rc;
    CHECKED_GL_CALL(glCompileShader(computeShader));
    CHECKED_GL_CALL(glGetShaderiv(computeShader, GL_COMPILE_STATUS, &rc));
    if (!rc)	//error compiling the shader file
    {
        GLSL::printShaderInfoLog(computeShader);
        std::cout << "Error compiling fragment shader " << std::endl;
        exit(1);
    }

    computeProgram = glCreateProgram();
    glAttachShader(computeProgram, computeShader);
    glLinkProgram(computeProgram);
    glUseProgram(computeProgram);

    ssbo_block_index = glGetProgramResourceIndex(computeProgram, GL_SHADER_STORAGE_BLOCK, "shader_data");
    glShaderStorageBlockBinding(computeProgram, ssbo_block_index, 2);

    GLuint block_index = glGetProgramResourceIndex(computeProgram, GL_SHADER_STORAGE_BLOCK, "shader_data");
    glShaderStorageBlockBinding(computeProgram, block_index, 2);

    glGenBuffers(1, &gpu_particles);
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, gpu_particles);
    glBufferData(GL_SHADER_STORAGE_BUFFER, 1024 * 1024, nullptr, GL_DYNAMIC_COPY);
    glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 0, gpu_particles);

    glGenBuffers(1, &atomic_buf);
    glBindBuffer(GL_ATOMIC_COUNTER_BUFFER, atomic_buf);
    glBufferData(GL_ATOMIC_COUNTER_BUFFER, sizeof(GLuint) * 1, nullptr, GL_DYNAMIC_DRAW);
    glBindBuffer(GL_ATOMIC_COUNTER_BUFFER, 0);
}

bool World::is_touching(Particle *a, Particle *b, glm::vec3 *normal, glm::vec3 *cpos) {
    auto ita = contact_index.find(a);
    if (ita == contact_index.end()) return false;

    auto &mapb = ita->second;
    auto itb = mapb.find(b);
    if (itb == mapb.end()) return false;

    auto gpuc = itb->second;
    if (normal != nullptr)
        *normal = gpuc.normal;
    if (cpos != nullptr)
        *cpos = gpuc.pos;

    return true;
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

uint GPUInput::get_size(glm::uint gpu_particle_count) {
    return 16 + gpu_particle_count * sizeof(GPUParticle);
}

void GPUInput::upload(vector<Particle*> &src) {
    particles_count = src.size();
    for (int i = 0; i < src.size(); i++) {
        auto &p = src[i];
        auto &gpu = particles[i];
        gpu.pos = p->pos;
        gpu.radius = p->radius;
        gpu.mass = p->mass;
        gpu.contact_count = 0;
    }
}

void GPUInput::download(vector<Particle *> &dst, ContactIndex &contacts) {
    for (int i = 0; i < dst.size(); i++) {
        auto p = dst[i];
        auto &gpu = particles[i];
        p->pos = gpu.pos;
        p->radius = gpu.radius;
        p->mass = gpu.mass;
        p->gravity_acc = gpu.gravity_acc;

        uint count = gpu.contact_count;
        if (count > MAX_CONTACTS_PER_PARTICLE) {
            cout << "#" << i << " has " << count << " collisions" << endl;
            count = MAX_CONTACTS_PER_PARTICLE;
        }
        for (int j = 0; j < count; j++) {
            // Build the contact index
            auto &gpu_contact = gpu.contacts[j];
            auto bi = dst[gpu_contact.other];
            contacts[p][bi] = gpu_contact;
        }
    }
}
