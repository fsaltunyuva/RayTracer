#include <cmath>
#include <algorithm>
#include "../include/BRDF.h"

static float clamp01(float x) {
    return std::max(0.0f, std::min(1.0f, x));
}

static Vec3 reflectLocal(const Vec3 &I, const Vec3 &N) {
    return I.subtract(N.scale(2.0f * I.dot(N)));
}

static float schlickF(float cosBeta, float eta) {
    cosBeta = clamp01(cosBeta);
    float R0 = (eta - 1.0f) / (eta + 1.0f);
    R0 = R0 * R0;
    float t = 1.0f - cosBeta;
    return R0 + (1.0f - R0) * (t * t * t * t * t);
}

// f(wi,wo)
Vec3 BRDF::eval(const Vec3 &nRaw, const Vec3 &wiRaw, const Vec3 &woRaw, const Vec3 &kd, const Vec3 &ks, float eta, float k) const {
    Vec3 n = nRaw.normalize();
    Vec3 wi = wiRaw.normalize();
    Vec3 wo = woRaw.normalize();

    float cosI = n.dot(wi);
    float cosO = n.dot(wo);

    if (cosI <= 0.0f) return Vec3(0, 0, 0);

    float cosI_clamped = std::max(1e-6f, cosI);
    float cosO_clamped = std::max(1e-6f, cosO);

    const float PI = M_PI;
    const float INV_PI = 1.0f / PI;

    auto cosAlphaR = [&]() -> float {
        Vec3 r = reflectLocal(wi.scale(-1.0f), n).normalize();
        return clamp01(r.dot(wo));
    };

    auto cosAlphaH = [&]() -> float {
        Vec3 wh = wi.add(wo).normalize();
        return clamp01(wh.dot(n));
    };

    Vec3 diffuse = kd.scale(INV_PI);

    float specScalar = 0.0f;

    switch (type) {
        case BRDFType::OriginalPhong: {
            // kd + ks * cos^p(ar)/costhetai
            float c = std::pow(cosAlphaR(), exponent);
            specScalar = c / cosI;
            break;
        }

        case BRDFType::ModifiedPhong: {
            // kd + ks * cos^p(ar)
            if (normalized) {
                float c = std::pow(cosAlphaR(), exponent);
                specScalar = ((exponent + 2.0f) / (2.0f * PI)) * c;
            }
            else {
                float c = std::pow(cosAlphaR(), exponent);
                specScalar = c;
            }
            break;
        }

        case BRDFType::OriginalBlinnPhong: {
            float c = std::pow(cosAlphaH(), exponent);
            specScalar = c / cosI;
            break;
        }

        case BRDFType::ModifiedBlinnPhong: {
            if (normalized) {
                float c = std::pow(cosAlphaH(), exponent); // brdf.pdf uses cosAlphaR but I think it's a typo
                specScalar = ((exponent + 8.0f) / (8.0f * PI)) * c;
            }
            else {
                float c = std::pow(cosAlphaH(), exponent);
                specScalar = c;
            }
            break;
        }

        case BRDFType::TorranceSparrow: {
            diffuse = kd.scale(1.0f / PI);

            Vec3 wh = wi.add(wo).normalize();
            float nDotWh = clamp01(n.dot(wh));
            float woDotWh = clamp01(wo.dot(wh));

            float D = ((exponent + 2.0f) / (2.0f * PI)) * std::pow(nDotWh, exponent);

            float denomG = std::max(1e-6f, woDotWh);
            float g1 = (2.0f * nDotWh * cosO_clamped) / denomG;
            float g2 = (2.0f * nDotWh * cosI_clamped) / denomG;
            float G = std::min(1.0f, std::min(g1, g2));
            G = std::max(0.0f, G);

            // Fresnel
            float F;
            if (k > 0.0f) {
                float R0 = ((eta - 1.0f) * (eta - 1.0f) + k * k) / ((eta + 1.0f) * (eta + 1.0f) + k * k);
                float t = 1.0f - woDotWh;
                F = R0 + (1.0f - R0) * (t * t * t * t * t);
            } else {
                F = schlickF(woDotWh, eta);
            }

            if (kdFresnel) {
                diffuse = diffuse.scale(1.0f - F);
            }

            specScalar = (D * F * G) / (4.0f * cosI_clamped * cosO_clamped);
            break;
        }

        default:
            return Vec3(0, 0, 0);
    }

    Vec3 specular = ks.scale(specScalar);
    return diffuse.add(specular);
}
