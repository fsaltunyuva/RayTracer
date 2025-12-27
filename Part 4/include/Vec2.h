#pragma once

class Vec2 {
public:
    float u, v;

    Vec2(): u(0), v(0) {}
    Vec2(float u, float v) : u(u), v(v) {}
    Vec2 scale(float s) const;
    Vec2 add(const Vec2& other) const;
};