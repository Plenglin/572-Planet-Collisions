//
// Created by astrid on 11/17/20.
//

#include "MeshPart.h"

#include <utility>
#include <glad/glad.h>
#include <glm/gtc/matrix_transform.hpp>
#include <iostream>

using namespace glm;


bool is_inside_triangle(vec3 pv, vec3 edge, vec3 norm) {
    return dot(cross(pv, edge), norm) >= 0;
}

float edge_param(vec3 pv, vec3 edge) {
    return dot(pv, edge) / dot(edge, edge);
}

vec3 get_closest_point(vec3 p, vec3 a, vec3 b, vec3 c) {
    vec3 ea = b - c;
    vec3 eb = c - a;
    vec3 ec = a - b;
    vec3 centroid = (a + b + c) / 3.0f;
    vec3 normal = normalize(cross(-eb, ea));
    vec3 altitude = normal * dot(normal, p - a);
    vec3 p_proj = p - altitude;

    vec3 va = a - centroid;
    vec3 vb = b - centroid;
    vec3 vc = c - centroid;
    vec3 vp = p - centroid;

    // Edge testing direction
    vec3 edge = ec;
    vec3 p_start = b;
    vec3 p_end = a;
    if (dot(va, vp) < 0) {
        edge = ea;
        p_start = c;
        p_end = b;
    } else if (dot(vb, vp) < 0) {
        edge = eb;
        p_start = a;
        p_end = c;
    }

    vec3 v = p_end - p_start;
    float parameter = dot(normalize(edge), normalize(v));

    if (parameter < 0) {
        return p_start;
    }
    if (parameter > 1) {
        return p_end;
    }
    if (dot(cross(vp, edge), normal) < 0) {
        return v * parameter + p_start;
    }
    return p_proj;
}

MeshPart::MeshPart(std::shared_ptr<Shape> shape, std::shared_ptr<Program> prog, int index, int tex) :
    shape(std::move(shape)),
    prog(std::move(prog)),
    index(index),
    tex(tex)
{
    auto &posBuf = this->shape->posBuf[index];
    auto &eleBuf = this->shape->eleBuf[index];

    // Calculate centroid
    for (auto it = eleBuf.begin(); it != eleBuf.end();) {
        unsigned int i = *(it++) * 3;
        vec3 v0 = vec3(posBuf[i], posBuf[i + 1], posBuf[i + 2]);
        i = *(it++) * 3;
        vec3 v1 = vec3(posBuf[i], posBuf[i + 1], posBuf[i + 2]);
        i = *(it++) * 3;
        vec3 v2 = vec3(posBuf[i], posBuf[i + 1], posBuf[i + 2]);

        float area = length(cross(v1 - v0, v2 - v0));
        vec3 ct = (v0 + v1 + v2);
        centroid_offset += ct * area;
        surface_area += area;
    }
    centroid_offset /= surface_area * 3.0f;

    for (auto it = eleBuf.begin(); it != eleBuf.end();) {
        unsigned int i = *(it++) * 3;
        vec3 v0 = vec3(posBuf[i], posBuf[i + 1], posBuf[i + 2]);
        i = *(it++) * 3;
        vec3 v1 = vec3(posBuf[i], posBuf[i + 1], posBuf[i + 2]);
        i = *(it++) * 3;
        vec3 v2 = vec3(posBuf[i], posBuf[i + 1], posBuf[i + 2]);

        vec3 closest_point = get_closest_point(centroid_offset, v0, v1, v2);
        float distance = length(closest_point - centroid_offset);
        avg_radius += distance;
        inner_radius = min(distance, inner_radius);

        // Assuming it's convex, volume is the sum of tetrahedrons, which are half-parallelepipeds
        mat3 parallelepiped = mat3(v0 - centroid_offset, v1 - centroid_offset, v2 - centroid_offset);
        volume += abs(determinant(parallelepiped));
    }
    volume /= 2;
    avg_radius /= posBuf.size();
    std::cout << inner_radius << "," << avg_radius << std::endl;
}

void MeshPart::draw(glm::mat4 &P, glm::mat4 &V, glm::mat4 &M, vec3 &pos) {
    mat4 mat = M * translate(mat4(1.0f), -centroid_offset);
    glUniformMatrix4fv(prog->getUniform("P"), 1, GL_FALSE, &P[0][0]);
    glUniformMatrix4fv(prog->getUniform("V"), 1, GL_FALSE, &V[0][0]);
    glUniformMatrix4fv(prog->getUniform("M"), 1, GL_FALSE, &mat[0][0]);
    glUniform3fv(prog->getUniform("campos"), 1, &pos[0]);
    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, tex);
    shape->draw(prog, false, index);
}
