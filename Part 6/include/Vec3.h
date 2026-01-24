#pragma once
#include "../json/json.hpp"
#include "../glm/glm.hpp"

class Vec3 {
public:
    float x, y, z;

    Vec3(): x(0), y(0), z(0) {}
    Vec3(float x, float y, float z) : x(x), y(y), z(z) {}
    Vec3(glm::vec3 v) : x(v.x), y(v.y), z(v.z) {}
    Vec3(glm::vec4 v) : x(v.x), y(v.y), z(v.z) {}
    Vec3(float s) : x(s), y(s), z(s) {}

    Vec3 subtract(const Vec3& other) const;
    float dot(const Vec3& other) const;
    Vec3 cross(const Vec3& other) const;
    Vec3 normalize() const;
    Vec3 scale(float s) const;
    Vec3 add(const Vec3& other) const;
    Vec3 multiply(const Vec3& other) const;
    float length() const;
    float length_squared() const;

    Vec3 operator-() const {
        return Vec3(-x, -y, -z);
    }

    float& operator[](int i) {
        if (i == 0) return x;
        if (i == 1) return y;
        return z;
    }

    const float& operator[](int i) const {
        if (i == 0) return x;
        if (i == 1) return y;
        return z;
    }
};
