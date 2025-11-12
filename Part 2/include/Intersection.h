#pragma once
#include "Vec3.h"
#include <limits>

// Forward declarations to avoid circular dependencies
class Mesh;
class MeshInstance;
enum class ShadingMode : int;

struct IntersectionInfo {
    bool hit = false;
    float t = std::numeric_limits<float>::max(); // Intersection distance
    int materialId = -1;
    Vec3 hitPoint;
    Vec3 hitNormal;

    int i0 = -1, i1 = -1, i2 = -1;
    ShadingMode shading = static_cast<ShadingMode>(0);
    const Mesh* hitMesh = nullptr; // Pointing to the intersected mesh
    const MeshInstance* hitInstance = nullptr; // Pointing to the intersected mesh instance
    glm::mat4 modelMatrix = glm::mat4(1.0f);
};
