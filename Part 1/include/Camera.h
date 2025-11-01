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
};
