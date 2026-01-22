#pragma once

#include <vector>
#include "Vec3.h"
#include "Camera.h"

class Tonemapper {

public:
    static std::vector<unsigned char> toPNG(
        int w, int h,
        const std::vector<Vec3>& hdr,
        const Camera::ToneMapSettings& tm
    );

private:
    static std::vector<unsigned char> photographicToPNG(
        int w, int h, const std::vector<Vec3>& hdr, const Camera::ToneMapSettings& tm);

    static std::vector<unsigned char> acesToPNG(
        int w, int h, const std::vector<Vec3>& hdr, const Camera::ToneMapSettings& tm);

    static std::vector<unsigned char> filmicToPNG(
        int w, int h, const std::vector<Vec3>& hdr, const Camera::ToneMapSettings& tm);

};
