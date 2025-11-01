#pragma once

#include <string>
#include "Vec3.h"

using namespace std;

class Material {
public:
    int id;
    string type; // optional
    Vec3 ambientReflectance;
    Vec3 diffuseReflectance;
    Vec3 specularReflectance;
    int phongExponent;
    Vec3 mirrorReflectance; // optional

    float refractionIndex; // optional
    float absorptionIndex; // optional
    Vec3 absorptionCoefficient; // optional
};