//
// Created by astrid on 11/17/20.
//

#ifndef LAB3_MESHPART_H
#define LAB3_MESHPART_H


#include <memory>
#include <glm/glm.hpp>
#include "Program.h"
#include "Shape.h"

class MeshPart {
    int index;
    int tex;
    std::shared_ptr<Program> prog;
    std::shared_ptr<Shape> shape;

public:
    glm::vec3 centroid_offset = glm::vec3(0, 0, 0);
    float inner_radius = INFINITY;
    float surface_area = 0;
    float volume = 0;

    MeshPart(std::shared_ptr<Shape> shape, std::shared_ptr<Program> prog, int index, int tex);
    void draw(glm::mat4 &P, glm::mat4 &V, glm::mat4 &M, glm::vec3 &pos);
};


#endif //LAB3_MESHPART_H
