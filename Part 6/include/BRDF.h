#pragma once

#include "Vec3.h"
#include <string>

enum class BRDFType {
    OriginalBlinnPhong,
    OriginalPhong,
    ModifiedBlinnPhong,
    ModifiedPhong,
    TorranceSparrow,
    Unknown
};

inline BRDFType brdfTypeFromString(const std::string &s) {
    if (s == "OriginalBlinnPhong")
        return BRDFType::OriginalBlinnPhong;

    if (s == "OriginalPhong")
        return BRDFType::OriginalPhong;

    if (s == "ModifiedBlinnPhong")
        return BRDFType::ModifiedBlinnPhong;

    if (s == "ModifiedPhong")
        return BRDFType::ModifiedPhong;

    if (s == "TorranceSparrow")
        return BRDFType::TorranceSparrow;

    return BRDFType::Unknown;
}

struct BRDF {
    int id = -1;
    BRDFType type = BRDFType::Unknown;

    float exponent = 0.0f;
    bool normalized = false;
    bool kdFresnel = false;

    Vec3 eval(const Vec3 &nRaw, const Vec3 &wiRaw, const Vec3 &woRaw, const Vec3 &kd, const Vec3 &ks, float eta, float k) const;
};
