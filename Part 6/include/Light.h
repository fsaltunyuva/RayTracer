#pragma once

#include "Image.h"
#include "Vec3.h"

class Light {
public:
    Vec3 intensity;
    Vec3 position;
};

class AmbientLight: public Light {
};

class PointLight: public Light {
public:
    int id;
    std::vector<std::string> transformationOrder; // Order of transformations (r1 t1 s2 r2 means first rotate with id 1, then translate with id 1, then scale with id 2, then rotate with id 2)
    glm::mat4 modelMatrix = glm::mat4(1.0f); // Identity matrix by default
};

class AreaLight: public Light {
public:
    int id;
    Vec3 normal;
    float size;
    Vec3 radiance;
    // TODO: Also need transformation info?
};

class DirectionalLight : public Light {
public:
    int id;
    Vec3 direction;
    Vec3 radiance; // Instead of intensity
};

class SpotLight : public Light {
public:
    int id;
    Vec3 direction;
    float coverageAngle;
    float falloffAngle;

    std::vector<std::string> transformationOrder;
    glm::mat4 modelMatrix = glm::mat4(1.0f);
};

enum class EnvMapType {
    LatLong, Probe
};

enum class EnvSampler {
    Uniform, Cosine
};

class SphericalDirectionalLight { // Environment Map Light
public:
    int id;
    EnvMapType type = EnvMapType::LatLong;
    int imageId;
    EnvSampler sampler = EnvSampler::Cosine;
    const Image* image = nullptr;
};
