#include "../include/Image.h"
#include <algorithm>

#define STB_IMAGE_IMPLEMENTATION
#include <iostream>

#include "../stb/stb_image.h"

// Loading an image from file using stb_image
bool Image::load() {
    int w, h, c;
    unsigned char* data = stbi_load(path.c_str(), &w, &h, &c, 3); // Forcing 3 channels (RGB)

    if (!data) {
        std::cerr << "stbi_load failed: " << path << "\n";
        width = height = channels = 0;
        pixels.clear();
        return false;
    }

    width = w;
    height = h;
    channels = 3;

    pixels.assign(data, data + (width * height * channels)); // Copy data to vector
    stbi_image_free(data); // Free the original data
    return true;
}

// Clamp function to keep values between 0 and 1
static float clamp01(float x) {
    return std::max(0.0f, std::min(1.0f, x));
}

Vec3 Image::sampleNearest(const Vec2& uv) const {
    if (width <= 0 || height <= 0 || pixels.empty()) return Vec3(0,0,0);

    // Clamp [0,1]
    float u = clamp01(uv.u);
    float v = clamp01(uv.v);

    // Find nearest texel (pixel center)
    int x = (int) std::floor(u * (width - 1) + 0.5f);
    int y = (int) std::floor(v * (height - 1) + 0.5f);

    // Clamp to image boundaries
    x = std::max(0, std::min(width - 1, x));
    y = std::max(0, std::min(height - 1, y));

    // Get pixel color
    int idx = (y * width + x) * channels;
    float r = pixels[idx + 0] / 255.0f;
    float g = pixels[idx + 1] / 255.0f;
    float b = pixels[idx + 2] / 255.0f;

    return Vec3(r, g, b);
}

// Bilinear filtering
Vec3 Image::sampleBilinear(const Vec2& uv) const {
    // If image is not loaded properly
    if (width <= 0 || height <= 0 || pixels.empty()) return Vec3(0,0,0);

    // UV Coordinates
    float u = uv.u;
    float v = uv.v;

    // Clamp [0,1]
    u = u - std::floor(u);
    v = v - std::floor(v);

    // Pixel space coordinates
    float x = u * width;
    float y = v * height;

    // Pixel coordinates relative to pixel centers
    float x_coord = x - 0.5f;
    float y_coord = y - 0.5f;

    // Pixel indices
    int x0 = (int) std::floor(x_coord);
    int y0 = (int) std::floor(y_coord);
    int x1 = x0 + 1;
    int y1 = y0 + 1;

    // Distance
    float dx = x_coord - x0;
    float dy = y_coord - y0;

    // Controlling boundaries
    auto clampX = [&](int val) { return std::max(0, std::min(width - 1, val)); };
    auto clampY = [&](int val) { return std::max(0, std::min(height - 1, val)); };

    int x0c = clampX(x0);
    int x1c = clampX(x1);
    int y0c = clampY(y0);
    int y1c = clampY(y1);

    // Get 4 neighboring pixel colors
    auto getPixel = [&](int px, int py) {
        int idx = (py * width + px) * channels;
        return Vec3(pixels[idx] / 255.0f, pixels[idx+1] / 255.0f, pixels[idx+2] / 255.0f);
    };

    // Get colors of the four surrounding pixels
    Vec3 c00 = getPixel(x0c, y0c);
    Vec3 c10 = getPixel(x1c, y0c);
    Vec3 c01 = getPixel(x0c, y1c);
    Vec3 c11 = getPixel(x1c, y1c);

    // Interpolate in X direction
    Vec3 top = c00.scale(1.0f - dx).add(c10.scale(dx));
    Vec3 bot = c01.scale(1.0f - dx).add(c11.scale(dx));

    // Interpolate in Y direction and return final color
    return top.scale(1.0f - dy).add(bot.scale(dy));
}

