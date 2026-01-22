#include "../include/Vec2.h"

Vec2 Vec2::scale(float s) const {
    return Vec2(u * s, v * s);
}

Vec2 Vec2::add(const Vec2 &other) const {
    return Vec2(u + other.u, v + other.v);
}
