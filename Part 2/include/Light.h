#pragma once

#include "Vec3.h"

class Light {
public:
    Vec3 intensity;
    Vec3 position;
};

class AmbientLight: public Light { //TODO: Is this needed?
};

class PointLight: public Light {
public:
    int id;
    std::vector<std::string> transformationOrder; // Order of transformations (r1 t1 s2 r2 means first rotate with id 1, then translate with id 1, then scale with id 2, then rotate with id 2)
    glm::mat4 modelMatrix = glm::mat4(1.0f); // Identity matrix by default
};
