#include "../include/Vec3.h"
#include <cmath>

Vec3 Vec3::subtract(const Vec3& other) const {
    return Vec3(x - other.x, y - other.y, z - other.z);
}

float Vec3::dot(const Vec3& other) const {
    return x * other.x + y * other.y + z * other.z;
}

Vec3 Vec3::cross(const Vec3& other) const {
    return Vec3(
            y * other.z - z * other.y,
            z * other.x - x * other.z,
            x * other.y - y * other.x
    );
}

float Vec3::length() const {
    return std::sqrt(x * x + y * y + z * z);
}

float Vec3::length_squared() const { // In some cases, squared length is useful to avoid sqrt computation
    return x * x + y * y + z * z;
}

Vec3 Vec3::normalize() const {
    float length = std::sqrt(x * x + y * y + z * z);
    if (length == 0.0f) {
        return *this; // return the zero vector itself to avoid division by zero
    }
    return Vec3(x / length, y / length, z / length);
}

Vec3 Vec3::scale(float s) const {
    return Vec3(x * s, y * s, z * s);
}

Vec3 Vec3::add(const Vec3& other) const {
    return Vec3(x + other.x, y + other.y, z + other.z);
}

Vec3 Vec3::multiply(const Vec3& other) const {
    return Vec3(x * other.x, y * other.y, z * other.z);
}