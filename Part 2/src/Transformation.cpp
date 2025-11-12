#include "../include/Transformation.h"

Transformation::Transformation()
        : id(0), type(TransformationType::Scaling), angle(0.0f),
          axis(0.0f, 0.0f, 0.0f), data(0.0f, 0.0f, 0.0f), matrix(1.0f) {}

Transformation Transformation::fromScaling(int id, const Vec3& s) {
    Transformation t;
    t.id = id;
    t.type = TransformationType::Scaling;
    t.data = s;
    glm::vec3 g(s.x, s.y, s.z);
    t.matrix = glm::scale(glm::mat4(1.0f), g);
    return t;
}

Transformation Transformation::fromTranslation(int id, const Vec3& tr) {
    Transformation t;
    t.id = id;
    t.type = TransformationType::Translation;
    t.data = tr;
    glm::vec3 g(tr.x, tr.y, tr.z);
    t.matrix = glm::translate(glm::mat4(1.0f), g);
    return t;
}

Transformation Transformation::fromRotation(int id, float angleDeg, const Vec3& ax) {
    Transformation t;
    t.id = id;
    t.type = TransformationType::Rotation;
    t.angle = angleDeg;
    t.axis = ax;
    glm::vec3 g(ax.x, ax.y, ax.z);
    t.matrix = glm::rotate(glm::mat4(1.0f), glm::radians(angleDeg), g);
    return t;
}

Transformation Transformation::fromComposite(int id, const glm::mat4& m) {
    Transformation t;
    t.id = id;
    t.type = TransformationType::Composite;
    t.matrix = m;
    return t;
}

glm::mat4 Transformation::getMatrix() const {
    return matrix;
}
