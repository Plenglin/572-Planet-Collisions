//
// Created by astrid on 11/12/20.
//

#include <iostream>
#include <utility>
#include <cstring>
#include "ForceFieldProgram.h"
#include "../Program.h"
#include "../GLSL.h"


void PhysicsObjectBuffer::set_objects(std::vector<PhysicsObject> &objects) {
    if (objects_count == 0) {
        glGenBuffers(1, &objects_gpu);
        glGenBuffers(1, &interactions);
    }

    objects_count = objects.size();
    interactions_count = (objects.size() - 1) * objects.size() / 2;
    glBindBuffer(GL_ATOMIC_COUNTER_BUFFER, objects_gpu);
    glBufferData(GL_ATOMIC_COUNTER_BUFFER, sizeof(PhysicsObject) * objects_count, objects.data(), GL_DYNAMIC_DRAW);

    glBindBuffer(GL_ATOMIC_COUNTER_BUFFER, interactions);
    glBufferData(GL_ATOMIC_COUNTER_BUFFER, sizeof(PhysicsObject) * interactions, nullptr, GL_DYNAMIC_DRAW);

    glBindBuffer(GL_ATOMIC_COUNTER_BUFFER, 0);
}

std::vector<PhysicsObject> PhysicsObjectBuffer::get_data() const {
    std::vector<PhysicsObject> output;
    output.resize(objects_count);

    glBindBuffer(GL_SHADER_STORAGE_BUFFER, objects_gpu);
    GLvoid* p = glMapBuffer(GL_SHADER_STORAGE_BUFFER, GL_READ_ONLY);

    memcpy(output.data(), p, sizeof(PhysicsObject) * objects_count);

    glBindBuffer(GL_SHADER_STORAGE_BUFFER, objects_gpu);
    glUnmapBuffer(GL_SHADER_STORAGE_BUFFER);
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, 0);

    return output;
}

ForceFieldProgram::ForceFieldProgram(const std::string& path) {
    static auto _aggregate = read_program("../resources/aggregate.glsl");
    aggregate = _aggregate;
    interact = read_program(path);

    glUseProgram(aggregate);
    glShaderStorageBlockBinding(aggregate, 0, 0);
    glShaderStorageBlockBinding(aggregate, 1, 1);

    glUseProgram(interact);
    glShaderStorageBlockBinding(interact, 0, 0);
    glShaderStorageBlockBinding(interact, 1, 1);

    glUseProgram(0);
}

void ForceFieldProgram::run(PhysicsObjectBuffer &ffo, bool overwrite) const {
    glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 0, ffo.objects_gpu);
    glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 1, ffo.interactions);

    // Interact objects
    glUseProgram(interact);
    glDispatchCompute(ffo.interactions_count, 1, 1);
    glMemoryBarrier(GL_SHADER_IMAGE_ACCESS_BARRIER_BIT);

    // Aggregate forces
    glUseProgram(aggregate);
    glDispatchCompute(ffo.objects_count, 1, 1);
    glMemoryBarrier(GL_SHADER_IMAGE_ACCESS_BARRIER_BIT);
}

GLuint read_program(const std::string& path) {
    std::string ShaderString = readFileAsString(path);
    const char* shader = ShaderString.c_str();
    GLuint computeShader = glCreateShader(GL_COMPUTE_SHADER);
    glShaderSource(computeShader, 1, &shader, nullptr);

    GLint rc;
    CHECKED_GL_CALL(glCompileShader(computeShader));
    CHECKED_GL_CALL(glGetShaderiv(computeShader, GL_COMPILE_STATUS, &rc));
    if (!rc) {
        GLSL::printShaderInfoLog(computeShader);
        std::cout << "Error compiling compute shader " << path << std::endl;
        exit(1);
    }

    GLuint output = glCreateProgram();
    glAttachShader(output, computeShader);
    glLinkProgram(output);
    glDetachShader(output, computeShader);

    return output;
}

void IntegrationProgram::run(PhysicsObjectBuffer &ffo) {

}
