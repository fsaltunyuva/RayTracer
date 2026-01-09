#pragma once

#include <string>
#include "Vec3.h"

using namespace std;

class Camera {
public:
    int id;
    Vec3 position, gaze, up;
    float nearPlane[4];
    float nearDistance;
    struct {int x, y;} imageResolution;

    string imageName;
    vector<string> transformationOrder; // Order of transformations (r1 t1 s2 r2 means first rotate with id 1, then translate with id 1, then scale with id 2, then rotate with id 2)
    glm::mat4 modelMatrix = glm::mat4(1.0f); // Identity matrix by default

    // Needed for dragon_metal.json
    bool hasFovY = false;
    float fovY = 0.0f;
    Vec3 gazePoint;

    // Jittered sampling
    int numSamples = 1; // Default is 1

    // Depth of Field
    float focusDistance = 0.0f; // 0 means no depth of field
    float apertureSize = 0.0f; // 0 means no depth of field

    // Tonemapping
    struct ToneMapSettings {
        string tmo = "Photographic"; // "Photographic", "Filmic", "ACES"
        float key = 0.18f; // TMOOptions first value
        float burnOut = 0.0f; // TMOOptions second value
        float saturation = 1.0f;
        float gamma = 2.2f;
        string extension = "";
    };

    vector<ToneMapSettings> tonemaps;
};
