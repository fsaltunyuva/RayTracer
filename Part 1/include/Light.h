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
};
