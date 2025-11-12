#pragma once
#include "Vec3.h"
#include <vector>
#include "Intersection.h"

class Mesh;

struct AABB {
    // Axis-Aligned Bounding Box
    Vec3 min;
    Vec3 max;

    AABB() : min(Vec3(FLT_MAX)), max(Vec3(-FLT_MAX)) {}

    // Expand the AABB to include point p
    void expand(const Vec3& p) {
        min.x = std::min(min.x, p.x);
        min.y = std::min(min.y, p.y);
        min.z = std::min(min.z, p.z);
        max.x = std::max(max.x, p.x);
        max.y = std::max(max.y, p.y);
        max.z = std::max(max.z, p.z);
    }

    // Ray-AABB intersection test
    bool intersect(const Vec3& origin, const Vec3& dir, float& tmin, float& tmax) const {
        float t0 = 0.0f, t1 = FLT_MAX;
        for (int i = 0; i < 3; i++) {
            float invD = 1.0f / dir[i];
            float tNear = (min[i] - origin[i]) * invD;
            float tFar  = (max[i] - origin[i]) * invD;
            if (invD < 0.0f) std::swap(tNear, tFar);
            t0 = std::max(t0, tNear);
            t1 = std::min(t1, tFar);
            if (t0 > t1) return false;
        }
        tmin = t0;
        tmax = t1;
        return true;
    }
};

// BVH Node
struct BVHNode {
    AABB bbox;
    BVHNode* left = nullptr;
    BVHNode* right = nullptr;
    std::vector<int> triangleIndices; // Indices of triangles in this leaf node

    bool isLeaf() const { return left == nullptr && right == nullptr; } // Leaf if no children
};

class BVH {
public:
    BVHNode* root = nullptr;
    std::vector<Vec3> vertices;
    std::vector<int> indices;
    glm::mat4 modelMatrix;

    void build(const std::vector<int>& meshIndices, const std::vector<Vec3>& sceneVertices, const glm::mat4& transform); // Build BVH from mesh indices and scene vertices
    bool intersect(const Vec3& rayOrigin, const Vec3& rayDir, float& closestT, IntersectionInfo& info) const; // Ray-BVH intersection

private:
    BVHNode* buildRecursive(const std::vector<int>& tris, int depth); // Recursive BVH builder
    bool intersectNode(BVHNode* node, const Vec3& rayOrigin, const Vec3& rayDir, float& closestT, IntersectionInfo& info) const; // Recursive intersection test
};