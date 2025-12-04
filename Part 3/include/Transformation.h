#pragma once
#include "Vec3.h"
#include "../glm/glm.hpp"
#include "../glm/gtc/matrix_transform.hpp"
#include <string>

enum class TransformationType {
    Scaling,
    Translation,
    Rotation,
    Composite
};

class Transformation {
public:
    int id;
    TransformationType type;

    Vec3 data;
    float angle; // for rotation
    Vec3 axis; // for rotation
    glm::mat4 matrix;

    Transformation();

    static Transformation fromScaling(int id, const Vec3& s);
    static Transformation fromTranslation(int id, const Vec3& t);
    static Transformation fromRotation(int id, float angleDeg, const Vec3& axis);
    static Transformation fromComposite(int id, const glm::mat4& m);

    glm::mat4 getMatrix() const;
};
