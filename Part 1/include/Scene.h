#pragma once

#include <vector>
#include "Camera.h"
#include "Light.h"
#include "Material.h"
#include "Object.h"
#include "Vec3.h"

using namespace std;

class Scene {
public:
    int maxRecursionDepth;
    Vec3 backgroundColor;
    float shadowRayEpsilon;
    float intersectionTestEpsilon;
    vector<Camera> cameras;
    Vec3 ambientLight;
    vector<Light> lights;
    vector<Material> materials;
    vector<Vec3> vertexData;
    Objects objects;

    bool loadScene(const std::string& filePath);
};