#include "../include/ImageIO.h"
#include <iostream>

#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "../stb/stb_image_write.h"

#define TINYEXR_IMPLEMENTATION
#include "../tinyexr/tinyexr.h"

bool ImageIO::writePNG(const std::string& filename, int w, int h, const std::vector<unsigned char>& rgb) {
    if ((int) rgb.size() != w * h * 3) {
        std::cerr << "writePNG: buffer size mismatch\n";
        return false;
    }
    int ok = stbi_write_png(filename.c_str(), w, h, 3, rgb.data(), w * 3);
    if (!ok) std::cerr << "writePNG: failed: " << filename << "\n";
    return ok != 0;
}

bool ImageIO::writeEXR(const std::string& filename, int w, int h, const std::vector<Vec3>& hdr) {
    std::vector<float> rgb(w * h * 3);

    for (int i = 0; i < w * h; ++i) {
        rgb[i * 3 + 0] = hdr[i].x;
        rgb[i * 3 + 1] = hdr[i].y;
        rgb[i * 3 + 2] = hdr[i].z;
    }

    const char* err = nullptr;
    int ret = SaveEXR(static_cast<const float*>(rgb.data()), w, h, 3, 0, filename.c_str(), &err);

    if (ret != TINYEXR_SUCCESS) {
        std::cerr << "EXR write error: " << (err ? err : "unknown") << "\n";
        if (err)
            FreeEXRErrorMessage(err);
        return false;
    }

    return true;
}

bool ImageIO::endsWith(const std::string& s, const std::string& suf) {
    if (s.size() < suf.size())
        return false;

    return std::equal(suf.rbegin(), suf.rend(), s.rbegin());
}

// "head.exr" + "_phot.png" -> "head_phot.png"
std::string ImageIO::replaceExrWithExtension(const std::string& imageName, const std::string& extension) {
    if (endsWith(imageName, ".exr")) {
        return imageName.substr(0, imageName.size() - 4) + extension;
    }

    return imageName + extension;
}
