#pragma once

#include <string>
#include <vector>
#include "Vec3.h"

class ImageIO {

public:
    static bool writePNG(const std::string& filename, int w, int h, const std::vector<unsigned char>& rgb);

    static bool writeEXR(const std::string& filename, int w, int h, const std::vector<Vec3>& hdr);

    static bool endsWith(const std::string& s, const std::string& suf);
    static std::string replaceExrWithExtension(const std::string& imageName, const std::string& extension);
};
