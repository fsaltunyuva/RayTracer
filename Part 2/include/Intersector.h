#pragma once
#include "Vec3.h"
#include "Scene.h"
#include "Intersection.h"

class Intersector {
public:
    // Explicit constructor to avoid implicit conversions
    explicit Intersector(const Scene& scene) : scene(scene) {}

    IntersectionInfo findClosestIntersection(const Vec3& rayOrigin,
                                 const Vec3& rayDir,
                                 bool isShadowRay = false) const;

    static bool RaySphere(Vec3 rayOrigin, Vec3 rayDir,
                          Vec3 sphereCenter, float radius, float tOut[2]);

    static bool RayTriangle(const Vec3& rayOrigin, const Vec3& rayDir,
                            const Vec3& v0, const Vec3& v1, const Vec3& v2,
                            float& t, bool backFaceCull, Vec3& hitNormalOut);

    static bool RayPlane(const Vec3& rayOrigin, const Vec3& rayDir,
                         const Vec3& planePoint, const Vec3& planeNormal,
                         float& t);

    static void Barycentrics(const Vec3& P, const Vec3& v0, const Vec3& v1, const Vec3& v2,
                             float &w0, float &w1, float &w2);

private:
    const Scene& scene;
};
