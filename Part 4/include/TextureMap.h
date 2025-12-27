#pragma once

#include "Image.h"
#include "Texture.h"
#include "Vec3.h"

class TextureMap {
public:
    int id;
    TextureType type;
    DecalMode decalMode;

    virtual ~TextureMap() = default; // Virtual destructor for proper cleanup of derived classes
};

class ImageTextureMap : public TextureMap {
public:
    int imageId;
    float bumpFactor = 0.01f; // Only used if decalMode == BumpNormal // TODO: Is this a good default value?
    const Image* image = nullptr;

    InterpolationMode interpolation = InterpolationMode::Nearest;

    float normalizer = 255.0f; // Optional
};

class PerlinTextureMap : public TextureMap {
public:
    float noiseScale = 1.0f;
    NoiseConversion conversion = NoiseConversion::Linear;
    int numOctaves = 1;
    float bumpFactor = 1.0f;
};

class CheckerTextureMap : public TextureMap {
public:
    float scale = 1.0f;
    float offset = 0.0f;
    Vec3 blackColor;
    Vec3 whiteColor;
};

