#pragma once

#include <string>
#include <vector>
#include "Vec3.h"
#include "Vec2.h"

class Image {
public:
    int id = -1;
    std::string path;

    int width = 0, height = 0, channels = 0;
    std::vector<unsigned char> pixels;

    bool load(); // Reads pixels into memory

    // Returns [0,1] rgb
    Vec3 sampleNearest(const Vec2& uv) const;
    Vec3 sampleBilinear(const Vec2& uv) const;
};