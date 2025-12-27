#pragma once

enum class TextureType {
    Image,
    Perlin,
    Checkerboard
};

enum class DecalMode {
    ReplaceKd,
    BlendKd,
    ReplaceKs,
    ReplaceBackground,
    ReplaceNormal,
    BumpNormal,
    ReplaceAll
};

enum class InterpolationMode {
    Nearest,
    Bilinear,
    Trilinear
};

enum class NoiseConversion {
    AbsVal,
    Linear
};