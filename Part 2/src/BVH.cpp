#include "../include/BVH.h"
#include "../include/Intersector.h"
#include <algorithm>
#include <numeric>
#include <cfloat>

void BVH::build(const std::vector<int>& meshIndices, const std::vector<Vec3>& sceneVertices, const glm::mat4& transform) {
    modelMatrix = transform;
    indices = meshIndices; // copy indices

    // Transform vertices to world space
    vertices.resize(sceneVertices.size());
    for (size_t i = 0; i < sceneVertices.size(); ++i) {
        glm::vec4 vLocal = glm::vec4(sceneVertices[i].x, sceneVertices[i].y, sceneVertices[i].z, 1.0f);
        glm::vec4 vWorld = modelMatrix * vLocal;
        vertices[i] = Vec3(vWorld.x, vWorld.y, vWorld.z);
    }

    std::vector<int> allTris(indices.size() / 3);
    std::iota(allTris.begin(), allTris.end(), 0);
    root = buildRecursive(allTris, 0);
}

BVHNode* BVH::buildRecursive(const std::vector<int>& tris, int depth) {
    BVHNode* node = new BVHNode();

    // Compute bounding box for this node
    for (int triIdx : tris) {
        int i0 = indices[3 * triIdx] - 1;
        int i1 = indices[3 * triIdx + 1] - 1;
        int i2 = indices[3 * triIdx + 2] - 1;

        node->bbox.expand(vertices[i0]);
        node->bbox.expand(vertices[i1]);
        node->bbox.expand(vertices[i2]);
    }

    // Base case, if few triangles or max depth reached, make leaf
    if (tris.size() <= 4 || depth > 25) {
        node->triangleIndices = tris;
        return node;
    }

    // Split along the longest axis
    Vec3 diag = node->bbox.max.subtract(node->bbox.min);
    int axis = 0;
    if (diag.y > diag.x && diag.y > diag.z) axis = 1;
    else if (diag.z > diag.x) axis = 2;

    float mid = (node->bbox.min[axis] + node->bbox.max[axis]) * 0.5f;

    std::vector<int> leftTris, rightTris;
    // Reserve space to avoid multiple allocations
    leftTris.reserve(tris.size() / 2);
    rightTris.reserve(tris.size() / 2);

    for (int triIdx : tris) {
        int i0 = indices[3 * triIdx] - 1;
        int i1 = indices[3 * triIdx + 1] - 1;
        int i2 = indices[3 * triIdx + 2] - 1;

        float centroid = (vertices[i0][axis] + vertices[i1][axis] + vertices[i2][axis]) / 3.0f;
        if (centroid < mid) {
            leftTris.push_back(triIdx);
        } else {
            rightTris.push_back(triIdx);
        }
    }

    // If division fails, make leaf
    if (leftTris.empty() || rightTris.empty()) {
        node->triangleIndices = tris;
        return node;
    }

    node->left = buildRecursive(leftTris, depth + 1);
    node->right = buildRecursive(rightTris, depth + 1);

    return node;
}

bool BVH::intersect(const Vec3& rayOrigin, const Vec3& rayDir, float& closestT, IntersectionInfo& info) const {
    Vec3 d = rayDir.normalize();
    return intersectNode(root, rayOrigin, d, closestT, info);
}

bool BVH::intersectNode(BVHNode* node, const Vec3& rayOrigin, const Vec3& rayDir,
                        float& closestT, IntersectionInfo& info) const {
    if (!node) return false;

    float tmin, tmax;

    // AABB and ray does not intersect or intersection is farther than closestT
    if (!node->bbox.intersect(rayOrigin, rayDir, tmin, tmax) || tmin > closestT) {
        return false;
    }

    bool hit = false;

    if (node->isLeaf()) {
        for (int triIdx : node->triangleIndices) {
            int i0 = indices[3 * triIdx] - 1;
            int i1 = indices[3 * triIdx + 1] - 1;
            int i2 = indices[3 * triIdx + 2] - 1;

            float tTri;
            Vec3 triNormal;

            // vertices[] is already in world space
            if (Intersector::RayTriangle(rayOrigin, rayDir, vertices[i0], vertices[i1], vertices[i2], tTri, false, triNormal)) {
                if (tTri > 1e-4f && tTri < closestT) { // TODO: Use epsilon from Intersector?
                    closestT = tTri;
                    hit = true;
                    info.hit = true;
                    info.t = tTri;
                    info.hitPoint = rayOrigin.add(rayDir.scale(tTri));
                    info.hitNormal = triNormal.normalize();
                    info.i0 = indices[3 * triIdx] - 1;
                    info.i1 = indices[3 * triIdx + 1] - 1;
                    info.i2 = indices[3 * triIdx + 2] - 1;
                }
            }
        }
        return hit;
    }
    
    bool hitLeft = intersectNode(node->left, rayOrigin, rayDir, closestT, info);
    bool hitRight = intersectNode(node->right, rayOrigin, rayDir, closestT, info);

    return hitLeft || hitRight;
}