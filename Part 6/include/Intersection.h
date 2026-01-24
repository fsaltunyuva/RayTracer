#pragma once
#include "Vec3.h"
#include "Vec2.h"
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

    // Triangle indices in vertex arrays
    int i0 = -1, i1 = -1, i2 = -1;

    // Barycentric (local space)
    float w0 = 0.0f, w1 = 0.0f, w2 = 0.0f;
    bool hasBarycentrics = false;

    // UV coordinates for texture mapping
    Vec2 hitUV;
    bool hasUV = false;

    // Tangent space vectors for normal mapping
    Vec3 tangentW; // World space
    Vec3 bitangentW; // World space
    bool hasTBN = false;

    // Texture map IDs (if any)
    const std::vector<int>* textureMapIds = nullptr;

    ShadingMode shading = static_cast<ShadingMode>(0);
    const Mesh* hitMesh = nullptr; // Pointing to the intersected mesh
    const MeshInstance* hitInstance = nullptr; // Pointing to the intersected mesh instance
    glm::mat4 modelMatrix = glm::mat4(1.0f);

    // Path Tracing
    bool isEmissive = false;
    Vec3 emission = Vec3(0, 0, 0);
};
