#pragma once
#include "TextureMap.h"

class Textures {
public:
    std::vector<ImageTextureMap> imageTextures;
    std::vector<PerlinTextureMap> perlinTextures;
    std::vector<CheckerTextureMap> checkerTextures;
};
