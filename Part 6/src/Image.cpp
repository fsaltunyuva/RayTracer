#include "../include/Image.h"
#include <algorithm>
#include <iostream>
#include <cmath>
#include <cctype>

#define STB_IMAGE_IMPLEMENTATION
#include "../stb/stb_image.h"

#include "../tinyexr/tinyexr.h"

static float clamp01(float x) {
    return std::max(0.0f, std::min(1.0f, x));
}

static std::string toLower(std::string s) {
    for (char& ch : s)
        ch = (char)std::tolower((unsigned char)ch);
    return s;
}

static bool endsWithCaseInsensitive(const std::string& s, const std::string& suf) {
    if (s.size() < suf.size())
        return false;

    return toLower(s.substr(s.size() - suf.size())) == toLower(suf);
}

// Loading an image from file
bool Image::load() {
    width = height = channels = 0;
    pixels.clear();
    fpixels.clear();
    isHDR = false;

    int w = 0, h = 0, c = 0;

    // .exr
    if (endsWithCaseInsensitive(path, ".exr")) {
        float* rgba = nullptr;
        const char* err = nullptr;

        int ret = LoadEXR(&rgba, &w, &h, path.c_str(), &err);
        if (ret != TINYEXR_SUCCESS) {
            std::cerr << "LoadEXR failed: " << path
                      << " | err: " << (err ? err : "unknown") << "\n";
            if (err) FreeEXRErrorMessage(err);
            return false;
        }

        width = w;
        height = h;
        channels = 3;
        isHDR = true;

        fpixels.resize(width * height * 3);

        // LoadEXR returns RGBA (4 floats per pixel)
        for (int i = 0; i < width * height; ++i) {
            fpixels[i*3 + 0] = rgba[i*4 + 0];
            fpixels[i*3 + 1] = rgba[i*4 + 1];
            fpixels[i*3 + 2] = rgba[i*4 + 2];
        }

        free(rgba);
        return true;
    }

    // .hdr
    if (endsWithCaseInsensitive(path, ".hdr")) {
        float* data = stbi_loadf(path.c_str(), &w, &h, &c, 3);
        if (!data) {
            std::cerr << "stbi_loadf failed: " << path << " reason: " << stbi_failure_reason() << "\n";
            return false;
        }

        width = w;
        height = h;
        channels = 3;
        isHDR = true;

        fpixels.assign(data, data + (width * height * channels));
        stbi_image_free(data);
        return true;
    }

    // .png, .jpg
    unsigned char* data = stbi_load(path.c_str(), &w, &h, &c, 3);
    if (!data) {
        std::cerr << "stbi_load failed: " << path << " reason: " << stbi_failure_reason() << "\n";
        return false;
    }

    width = w;
    height = h;
    channels = 3;
    isHDR = false;

    pixels.assign(data, data + (width * height * channels));
    stbi_image_free(data);
    return true;
}

Vec3 Image::sampleNearest(const Vec2& uv) const {
    if (width <= 0 || height <= 0) return Vec3(0,0,0);
    if (!isHDR && pixels.empty()) return Vec3(0,0,0);
    if (isHDR && fpixels.empty()) return Vec3(0,0,0);

    // Clamp [0,1]
    float u = clamp01(uv.u);
    float v = clamp01(uv.v);

    int x = (int)std::floor(u * (width - 1) + 0.5f);
    int y = (int)std::floor(v * (height - 1) + 0.5f);

    x = std::max(0, std::min(width - 1, x));
    y = std::max(0, std::min(height - 1, y));

    int idx = (y * width + x) * 3;

    if (isHDR) {
        return Vec3(fpixels[idx + 0], fpixels[idx + 1], fpixels[idx + 2]);
    } else {
        float r = pixels[idx + 0] / 255.0f;
        float g = pixels[idx + 1] / 255.0f;
        float b = pixels[idx + 2] / 255.0f;
        return Vec3(r, g, b);
    }
}

Vec3 Image::sampleBilinear(const Vec2& uv) const {
    if (width <= 0 || height <= 0) return Vec3(0,0,0);
    if (!isHDR && pixels.empty()) return Vec3(0,0,0);
    if (isHDR && fpixels.empty()) return Vec3(0,0,0);

    // Wrap [0,1) like your old code
    float u = uv.u - std::floor(uv.u);
    float v = uv.v - std::floor(uv.v);

    float x = u * width;
    float y = v * height;

    float x_coord = x - 0.5f;
    float y_coord = y - 0.5f;

    int x0 = (int)std::floor(x_coord);
    int y0 = (int)std::floor(y_coord);
    int x1 = x0 + 1;
    int y1 = y0 + 1;

    float dx = x_coord - x0;
    float dy = y_coord - y0;

    auto clampX = [&](int val) { return std::max(0, std::min(width - 1, val)); };
    auto clampY = [&](int val) { return std::max(0, std::min(height - 1, val)); };

    int x0c = clampX(x0);
    int x1c = clampX(x1);
    int y0c = clampY(y0);
    int y1c = clampY(y1);

    auto getPixel = [&](int px, int py) {
        int idx = (py * width + px) * 3;
        if (isHDR) {
            return Vec3(fpixels[idx + 0], fpixels[idx + 1], fpixels[idx + 2]);
        } else {
            return Vec3(pixels[idx + 0] / 255.0f,
                        pixels[idx + 1] / 255.0f,
                        pixels[idx + 2] / 255.0f);
        }
    };

    Vec3 c00 = getPixel(x0c, y0c);
    Vec3 c10 = getPixel(x1c, y0c);
    Vec3 c01 = getPixel(x0c, y1c);
    Vec3 c11 = getPixel(x1c, y1c);

    Vec3 top = c00.scale(1.0f - dx).add(c10.scale(dx));
    Vec3 bot = c01.scale(1.0f - dx).add(c11.scale(dx));
    return top.scale(1.0f - dy).add(bot.scale(dy));
}
