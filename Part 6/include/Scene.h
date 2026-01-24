#pragma once

#include <vector>

#include "Camera.h"
#include "Light.h"
#include "Material.h"
#include "Object.h"
#include "Vec3.h"
#include "Transformation.h"
#include "Image.h"
#include "Textures.h"
#include "BRDF.h"

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
    bool loadScene(const string& filePath);
    // TODO: Is 64 is a good default value?
    int areaLightNumSamples = 64; // Default number of samples for area lights

    vector<Image> images;
    vector<Vec2> texCoordData;
    Textures textures;

    int backgroundTextureMapId = -1;

    vector<DirectionalLight> directionalLights;
    vector<SpotLight> spotLights;
    vector<SphericalDirectionalLight> envLights;

    vector<BRDF> brdfs;

    const BRDF *getBRDFById(int id) const { // TODO: I should move this to somewhere else
        for (const BRDF &b: brdfs)
            if (b.id == id)
                return &b;

        return nullptr;
    }

    bool zeroBasedIndexing = false;
};