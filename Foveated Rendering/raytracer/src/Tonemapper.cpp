#include "../include/Tonemapper.h"
#include <cmath>
#include <algorithm>
#include <vector>

static float clamp01(float v) {
    return (v < 0.0f) ? 0.0f : (v > 1.0f) ? 1.0f : v;
}

static float getLuminance(const Vec3& c) {
    return 0.2126f * c.x + 0.7152f * c.y + 0.0722f * c.z;
}

// Reinhard log-average luminance
static float computeLogAverageLuminance(const std::vector<Vec3>& hdr) {
    if (hdr.empty())
        return 1.0f;

    const float delta = 1e-4f;
    double sumLog = 0.0;

    for (const auto& c : hdr) {
        float lum = getLuminance(c);
        if (lum < 0.0f) lum = 0.0f;
        sumLog += std::log(delta + lum);
    }

    return (float) std::exp(sumLog / (double) hdr.size());
}

static float computeWhitePointExposed(const std::vector<float>& Yin, float exposure, float burnPct) {
    const int N = (int)Yin.size();
    if (N == 0) return 1e9f;

    if (burnPct <= 1e-6f) return 1e9f; // No burn

    std::vector<float> L = Yin;

    for (float& v : L) {
        v = std::max(0.0f, v) * exposure;
    }

    float percentile = (100.0f - burnPct) / 100.0f;
    int k = (int) std::floor(percentile * (N - 1));
    k = std::max(0, std::min(N - 1, k));

    std::nth_element(L.begin(), L.begin() + k, L.end());
    return std::max(L[k], 1e-6f);
}


static Vec3 reconstructColor(const Vec3& colorIn, float Yin, float Yout, float saturation) {
    if (Yin < 1e-6f) return Vec3(0, 0, 0);

    float ratioR = colorIn.x / Yin;
    float ratioG = colorIn.y / Yin;
    float ratioB = colorIn.z / Yin;

    if (std::abs(saturation - 1.0f) > 1e-4f) {
        ratioR = std::pow(std::max(0.0f, ratioR), saturation);
        ratioG = std::pow(std::max(0.0f, ratioG), saturation);
        ratioB = std::pow(std::max(0.0f, ratioB), saturation);
    }

    return Vec3(Yout * ratioR, Yout * ratioG, Yout * ratioB);
}


static float acesMap(float x) {
    const float A = 2.51f;
    const float B = 0.03f;
    const float C = 2.43f;
    const float D = 0.59f;
    const float E = 0.14f;

    return (x * (A * x + B)) / (x * (C * x + D) + E);
}

static float filmicMap(float x) {
    // map(L) = ((L(aL + cb) + de) / (L(aL + b) + df)) - e/f
    const float a = 0.22f;
    const float b = 0.30f;
    const float c = 0.10f;
    const float d = 0.20f;
    const float e = 0.01f;
    const float f = 0.30f;

    float num = x * (a * x + c * b) + d * e;
    float den = x * (a * x + b) + d * f;
    return (num / den) - (e / f);
}


std::vector<unsigned char> Tonemapper::toPNG(int w, int h, const std::vector<Vec3>& hdr, const Camera::ToneMapSettings& tm){
    std::string t = tm.tmo;

    for (auto& ch : t)
        ch = (char) std::tolower((unsigned char) ch);

    if (t == "aces")
        return acesToPNG(w, h, hdr, tm);
    if (t == "filmic" || t == "film")
        return filmicToPNG(w, h, hdr, tm);

    return photographicToPNG(w, h, hdr, tm);
}


std::vector<unsigned char> Tonemapper::photographicToPNG(int w, int h, const std::vector<Vec3>& hdr, const Camera::ToneMapSettings& tm){
    const int N = w * h;

    if ((int) hdr.size() != N) return {};

    std::vector<float> Yin(N);

    for (int i = 0; i < N; ++i)
        Yin[i] = getLuminance(hdr[i]);

    float Yavg = computeLogAverageLuminance(hdr);
    float key = tm.key;
    float exposure = (Yavg > 1e-6f) ? (key / Yavg) : 1.0f;

    float Lwhite = computeWhitePointExposed(Yin, exposure, tm.burnOut);
    float Lwhite2 = Lwhite * Lwhite;

    float sat = tm.saturation;
    float gamma = (tm.gamma > 1e-6f ? tm.gamma : 2.2f);
    float invGamma = 1.0f / gamma;

    std::vector<unsigned char> out(N * 3);

    for (int i = 0; i < N; ++i) {
        Vec3 color = hdr[i];
        float Yin_i = Yin[i];

        float L = std::max(0.0f, Yin_i) * exposure;

        float Yout = (L * (1.0f + (L / Lwhite2))) / (1.0f + L);
        Yout = clamp01(Yout);

        Vec3 c = reconstructColor(color, Yin_i, Yout, sat);

        c.x = std::pow(clamp01(c.x), invGamma);
        c.y = std::pow(clamp01(c.y), invGamma);
        c.z = std::pow(clamp01(c.z), invGamma);

        out[i * 3 + 0] = (unsigned char) std::lround(255.0f * c.x);
        out[i * 3 + 1] = (unsigned char) std::lround(255.0f * c.y);
        out[i * 3 + 2] = (unsigned char) std::lround(255.0f * c.z);
    }

    return out;
}

std::vector<unsigned char> Tonemapper::acesToPNG(int w, int h, const std::vector<Vec3>& hdr, const Camera::ToneMapSettings& tm){
    const int N = w * h;

    if ((int) hdr.size() != N) return {};

    std::vector<float> Yin(N);

    for (int i = 0; i < N; ++i)
        Yin[i] = getLuminance(hdr[i]);

    float Yavg = computeLogAverageLuminance(hdr);
    float key = tm.key;
    float exposure = (Yavg > 1e-6f) ? (key / Yavg) : 1.0f;

    // map(L)/map(Lw)
    float Lw = computeWhitePointExposed(Yin, exposure, tm.burnOut);
    float denom = std::max(acesMap(Lw), 1e-6f);

    float sat = tm.saturation;
    float gamma = (tm.gamma > 1e-6f ? tm.gamma : 2.2f);
    float invGamma = 1.0f / gamma;

    std::vector<unsigned char> out(N * 3);

    for (int i = 0; i < N; ++i) {
        Vec3 color = hdr[i];
        float Yin_i = Yin[i];

        float Ls = std::max(0.0f, Yin_i) * exposure;
        float Yout = acesMap(Ls) / denom;
        Yout = clamp01(Yout);

        Vec3 c = reconstructColor(color, Yin_i, Yout, sat);

        c.x = std::pow(clamp01(c.x), invGamma);
        c.y = std::pow(clamp01(c.y), invGamma);
        c.z = std::pow(clamp01(c.z), invGamma);

        out[i * 3 + 0] = (unsigned char) std::lround(255.0f * c.x);
        out[i * 3 + 1] = (unsigned char) std::lround(255.0f * c.y);
        out[i * 3 + 2] = (unsigned char) std::lround(255.0f * c.z);
    }

    return out;
}


std::vector<unsigned char> Tonemapper::filmicToPNG(int w, int h, const std::vector<Vec3>& hdr, const Camera::ToneMapSettings& tm){
    const int N = w * h;
    if ((int) hdr.size() != N) return {};

    std::vector<float> Yin(N);
    for (int i = 0; i < N; ++i) Yin[i] = getLuminance(hdr[i]);

    float Yavg = computeLogAverageLuminance(hdr);
    float key = tm.key;
    float exposure = (Yavg > 1e-6f) ? (key / Yavg) : 1.0f;

    // map(L)/map(Lw)
    float Lw = computeWhitePointExposed(Yin, exposure, tm.burnOut);
    float denom = std::max(filmicMap(Lw), 1e-6f);

    float sat = tm.saturation;
    float gamma = (tm.gamma > 1e-6f ? tm.gamma : 2.2f);
    float invGamma = 1.0f / gamma;

    std::vector<unsigned char> out(N * 3);

    for (int i = 0; i < N; ++i) {
        Vec3 color = hdr[i];
        float Yin_i = Yin[i];

        float Ls = std::max(0.0f, Yin_i) * exposure;
        float Yout = filmicMap(Ls) / denom;
        Yout = clamp01(Yout);

        Vec3 c = reconstructColor(color, Yin_i, Yout, sat);

        c.x = std::pow(clamp01(c.x), invGamma);
        c.y = std::pow(clamp01(c.y), invGamma);
        c.z = std::pow(clamp01(c.z), invGamma);

        out[i * 3 + 0] = (unsigned char) std::lround(255.0f * c.x);
        out[i * 3 + 1] = (unsigned char) std::lround(255.0f * c.y);
        out[i * 3 + 2] = (unsigned char) std::lround(255.0f * c.z);
    }

    return out;
}