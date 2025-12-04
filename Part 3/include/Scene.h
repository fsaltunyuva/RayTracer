#pragma once

#include <vector>
#include "Camera.h"
#include "Light.h"
#include "Material.h"
#include "Object.h"
#include "Vec3.h"
#include "Transformation.h"

using namespace std;

class Scene {
public:
    int maxRecursionDepth;
    Vec3 backgroundColor;
    float shadowRayEpsilon;
    float intersectionTestEpsilon;
    vector<Camera> cameras;
    Vec3 ambientLight;
    vector<PointLight> pointLights;
    vector<AreaLight> areaLights;
    vector<Material> materials;
    vector<Transformation> transformations;
    vector<Vec3> vertexData;
    Objects objects;
    bool loadScene(const std::string& filePath);
    // TODO: Is 4 is a good default value?
    int areaLightNumSamples = 4; // Default number of samples for area lights
};