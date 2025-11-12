#include "../include/Intersector.h"
#include <limits>

// Ray-sphere intersection function
bool Intersector::RaySphere(Vec3 rayOrigin, Vec3 rayDir,
                            Vec3 sphereCenter, float radius, float tOut[2]) {

    Vec3 pc = rayOrigin.subtract(sphereCenter);// p - c

    float a = rayDir.dot(rayDir); // d . d
    float b = 2.0f * pc.dot(rayDir); // 2d . (p - c)
    float c = pc.dot(pc) - radius * radius; // (p - c) . (p - c) - r^2

    float discriminant = b * b - 4 * a * c; // b^2 - 4ac

    if (discriminant < 0) return false; // No intersection

    float sD = sqrt(discriminant);

    tOut[0] = (-b - sD) / (2.0f * a); // Nearest intersection point
    tOut[1] = (-b + sD) / (2.0f * a); // Nearest intersection point
    return true;
}

// Ray-triangle intersection function (barycentric / plane method)
bool Intersector::RayTriangle(const Vec3& rayOrigin, const Vec3& rayDir,
                              const Vec3& v0, const Vec3& v1, const Vec3& v2,
                              float& t, bool backFaceCull, Vec3& hitNormalOut) {

    Vec3 e1 = v1.subtract(v0);
    Vec3 e2 = v2.subtract(v0);
    Vec3 N  = e1.cross(e2);

    // Degenerate triangle check
    float N_len_sq = N.length_squared();

    // If the area is too small, consider it degenerate
    if (N_len_sq < 1e-12f) { // TODO: Get epsilon from scene
        return false; // Degenerate triangle
    }

    float NdotD = N.dot(rayDir);

    if (fabs(NdotD) < 1e-8f) return false; // Plane and ray are parallel (TODO: Get epsilon from scene)

    if (backFaceCull && NdotD >= 0.0f) return false; // Early Back-face culling

    // Compute t (intersection point) using plane equation
    t = N.dot(v0.subtract(rayOrigin)) / NdotD;
    if (t < 0) return false; // The Triangle is behind the ray

    // Check if the intersection point is inside the triangle using edge function
    Vec3 P = rayOrigin.add(rayDir.scale(t));

    // inside test
    Vec3 C;
    {
        Vec3 edge = v1.subtract(v0);
        Vec3 vp   = P.subtract(v0);
        C = edge.cross(vp);
        if (N.dot(C) < 0) return false;
    }
    {
        Vec3 edge = v2.subtract(v1);
        Vec3 vp   = P.subtract(v1);
        C = edge.cross(vp);
        if (N.dot(C) < 0) return false;
    }
    {
        Vec3 edge = v0.subtract(v2);
        Vec3 vp   = P.subtract(v2);
        C = edge.cross(vp);
        if (N.dot(C) < 0) return false;
    }

    hitNormalOut = N; // // not normalized yet

    return true;
}

bool Intersector::RayPlane(const Vec3& rayOrigin, const Vec3& rayDir,
                           const Vec3& planePoint, const Vec3& planeNormal,
                           float& t) {
    // T = (P - RayOrigin) . Normal / (RayDir . Normal)
    // RayDir . Normal = 0 -> Parallel
    float NdotD = planeNormal.dot(rayDir);

    if (std::fabs(NdotD) < 1e-6f) return false;

    Vec3 PmO = planePoint.subtract(rayOrigin);

    float NdotP = planeNormal.dot(PmO);

    t = NdotP / NdotD;

    return t > 0.0f;
}

void Intersector::Barycentrics(const Vec3& P, const Vec3& v0, const Vec3& v1, const Vec3& v2,
                               float &w0, float &w1, float &w2) {
    Vec3 N = (v1.subtract(v0)).cross(v2.subtract(v0));
    float invNN = 1.0f / std::max(1e-20f, N.dot(N));
    w0 = N.dot((v1.subtract(P)).cross(v2.subtract(P))) * invNN;
    w1 = N.dot((v2.subtract(P)).cross(v0.subtract(P))) * invNN;
    w2 = 1.0f - w0 - w1;
}

IntersectionInfo Intersector::findClosestIntersection(const Vec3& rayOrigin,
                                          const Vec3& rayDir,
                                          bool isShadowRay) const
{
    IntersectionInfo info;

    float intersectionTestEpsilon;
    if (isShadowRay) intersectionTestEpsilon = scene.shadowRayEpsilon;
    else intersectionTestEpsilon = scene.intersectionTestEpsilon;

    float closestT = numeric_limits<float>::max();

    // Sphere intersection
    for (const auto& sphere : scene.objects.spheres) {
        glm::mat4 M = sphere.modelMatrix;
        glm::mat4 invM = glm::inverse(M);

        // Transform ray to local space of sphere
        glm::vec3 ro_local = glm::vec3(invM * glm::vec4(rayOrigin.x, rayOrigin.y, rayOrigin.z, 1.0f));
        glm::vec3 rd_local = glm::vec3(invM * glm::vec4(rayDir.x, rayDir.y, rayDir.z, 0.0f));

        Vec3 centerLocal = scene.vertexData[sphere.center - 1];
        float t[2];
        if (RaySphere(Vec3(ro_local), Vec3(rd_local), centerLocal, sphere.radius, t)) {
            float tCandidate = std::numeric_limits<float>::max();

            if (t[0] > intersectionTestEpsilon) tCandidate = t[0];
            if (t[1] > intersectionTestEpsilon && t[1] < tCandidate) tCandidate = t[1];

            if (tCandidate < std::numeric_limits<float>::max()) {
                glm::vec3 localHit = ro_local + tCandidate * rd_local;
                glm::vec3 worldHit = glm::vec3(M * glm::vec4(localHit, 1.0f));

                float tWorld = glm::length(worldHit - glm::vec3(rayOrigin.x, rayOrigin.y, rayOrigin.z));

                if (tWorld > intersectionTestEpsilon && tWorld < closestT) {
                    glm::vec3 localN = glm::normalize(localHit - glm::vec3(centerLocal.x, centerLocal.y, centerLocal.z));
                    glm::vec3 worldN = glm::normalize(glm::vec3(glm::transpose(glm::inverse(M)) * glm::vec4(localN, 0.0f)));

                    closestT = tWorld;
                    info.hit = true;
                    info.materialId = sphere.materialId;
                    info.t = tWorld;
                    info.hitPoint = Vec3(worldHit.x, worldHit.y, worldHit.z);
                    info.hitNormal = Vec3(worldN.x, worldN.y, worldN.z);
                    if (info.hitNormal.dot(rayDir) > 0)
                        info.hitNormal = info.hitNormal.scale(-1.0f);
                }
            }
        }
    }

    // Triangle intersection
    for (const auto& tri : scene.objects.triangles) {
        float t;
        glm::mat4 M = tri.modelMatrix;

        // If determinant is negative, it's a mirrored transform (0, -1, 0 scale for example)
        //bool isMirrored = glm::determinant(M) < 0.0f; // TODO: Slows the process?
        bool performCulling = !isShadowRay /* && !isMirrored*/;

        Vec3 v0_local = scene.vertexData[tri.indices[0] - 1];
        Vec3 v1_local = scene.vertexData[tri.indices[1] - 1];
        Vec3 v2_local = scene.vertexData[tri.indices[2] - 1];

        Vec3 v0 = Vec3(M * glm::vec4(v0_local.x, v0_local.y, v0_local.z, 1.0f));
        Vec3 v1 = Vec3(M * glm::vec4(v1_local.x, v1_local.y, v1_local.z, 1.0f));
        Vec3 v2 = Vec3(M * glm::vec4(v2_local.x, v2_local.y, v2_local.z, 1.0f));

        Vec3 triN;

        if (RayTriangle(rayOrigin, rayDir, v0, v1, v2, t, performCulling, triN)) {
            if (t > intersectionTestEpsilon && t < closestT) {
                closestT = t;
                info.hit = true;
                info.materialId = tri.materialId;
                info.t = t;
                info.hitPoint = rayOrigin.add(rayDir.scale(t));

                glm::mat4 normalM = glm::transpose(glm::inverse(M));
                glm::vec3 nWorld = glm::normalize(glm::vec3(normalM * glm::vec4(triN.x, triN.y, triN.z, 0.0f)));
                info.hitNormal = Vec3(nWorld);
                if (info.hitNormal.dot(rayDir) > 0) info.hitNormal = info.hitNormal.scale(-1.0f);
            }
        }
    }

    // Mesh Faces intersection
    for (const auto& mesh : scene.objects.meshes) {
        if (mesh.bvh) {
            float tCandidate = closestT;
            if (mesh.bvh->intersect(rayOrigin, rayDir, tCandidate, info)) {
                if (tCandidate < closestT) {
                    closestT = tCandidate;

                    info.materialId = mesh.materialId;
                    info.hitMesh = &mesh;
                    info.hitInstance = nullptr;
                    info.shading = mesh.shadingMode;
                    info.modelMatrix = mesh.modelMatrix;

                    if (info.hitNormal.dot(rayDir) > 0)
                        info.hitNormal = info.hitNormal.scale(-1.0f);
                }
            }
            continue; // skip triangle test if BVH is present
        }

        glm::mat4 M = mesh.modelMatrix;
        glm::mat4 normalM = glm::transpose(glm::inverse(M));

        // If determinant is negative, it's a mirrored transform (0, -1, 0 scale for example)
        //bool isMirrored = glm::determinant(M) < 0.0f; // TODO: Slows the process?

        for (size_t i = 0; i + 2 < mesh.data.size(); i += 3) {
            int i0 = mesh.data[i] - 1, i1 = mesh.data[i+1] - 1, i2 = mesh.data[i+2] - 1;

            Vec3 v0 = Vec3(M * glm::vec4(scene.vertexData[i0].x, scene.vertexData[i0].y, scene.vertexData[i0].z, 1.0f));
            Vec3 v1 = Vec3(M * glm::vec4(scene.vertexData[i1].x, scene.vertexData[i1].y, scene.vertexData[i1].z, 1.0f));
            Vec3 v2 = Vec3(M * glm::vec4(scene.vertexData[i2].x, scene.vertexData[i2].y, scene.vertexData[i2].z, 1.0f));

            float t;
            Vec3 triN;

            bool performCulling = !isShadowRay /*&& !isMirrored*/;

            if (RayTriangle(rayOrigin, rayDir, v0, v1, v2, t, performCulling, triN)) {
                if (t > intersectionTestEpsilon && t < closestT) {
                    closestT = t;
                    info.hit = true;
                    info.materialId = mesh.materialId;
                    info.t = t;
                    info.hitPoint = rayOrigin.add(rayDir.scale(t));

                    // normal transformation (inverse-transpose of M)
                    glm::vec3 nWorld = glm::normalize(glm::vec3(normalM * glm::vec4(triN.x, triN.y, triN.z, 0.0f)));
                    info.hitNormal = Vec3(nWorld);

                    if (info.hitNormal.dot(rayDir) > 0)
                        info.hitNormal = info.hitNormal.scale(-1.0f);

                    info.i0 = i0; info.i1 = i1; info.i2 = i2;
                    info.shading = mesh.shadingMode; // ShadingMode::Flat | Smooth
                    info.hitMesh = &mesh;
                    info.modelMatrix = mesh.modelMatrix;
                    info.hitInstance = nullptr;
                }
            }
        }
    }

    // MeshInstance intersection
    for (const auto& instance : scene.objects.meshInstances) {
        // Get base mesh (found in main.cpp)
        const Mesh* baseMesh = instance.resolvedBaseMesh;

        if (instance.bvh) {
            float tCandidate = closestT;
            if (instance.bvh->intersect(rayOrigin, rayDir, tCandidate, info)) {
                if (tCandidate > intersectionTestEpsilon && tCandidate < closestT) {
                    closestT = tCandidate;

                    // Use instance's own material if set, otherwise use base mesh's material
                    info.materialId = (instance.materialId != -1) ? instance.materialId : baseMesh->materialId;
                    info.hitInstance = &instance;
                    info.hitMesh = baseMesh;
                    info.modelMatrix = instance.modelMatrix;
                    info.shading = baseMesh->shadingMode;

                    if (info.hitNormal.dot(rayDir) > 0)
                        info.hitNormal = info.hitNormal.scale(-1.0f);
                }
            }
            continue; // Skip triangle test if BVH is present
        }

        glm::mat4 M = instance.modelMatrix;
        glm::mat4 normalM = transpose(glm::inverse(M));

        // If determinant is negative, it's a mirrored transform (0, -1, 0 scale for example)
        bool isMirrored = determinant(M) < 0.0f;

        for (size_t i = 0; i + 2 < baseMesh->data.size(); i += 3) {
            int i0 = baseMesh->data[i] - 1, i1 = baseMesh->data[i+1] - 1, i2 = baseMesh->data[i+2] - 1;

            Vec3 v0 = Vec3(M * glm::vec4(scene.vertexData[i0].x, scene.vertexData[i0].y, scene.vertexData[i0].z, 1.0f));
            Vec3 v1 = Vec3(M * glm::vec4(scene.vertexData[i1].x, scene.vertexData[i1].y, scene.vertexData[i1].z, 1.0f));
            Vec3 v2 = Vec3(M * glm::vec4(scene.vertexData[i2].x, scene.vertexData[i2].y, scene.vertexData[i2].z, 1.0f));

            float t;
            Vec3 triN;

            bool performCulling = !isShadowRay && !isMirrored; // Back-face culling only for non-shadow rays and non-mirrored instances

            if (RayTriangle(rayOrigin, rayDir, v0, v1, v2, t, performCulling, triN)) {
                if (t > intersectionTestEpsilon && t < closestT) {
                    closestT = t;
                    info.hit = true;
                    info.materialId = (instance.materialId != -1) ? instance.materialId : baseMesh->materialId;
                    info.t = t;
                    info.hitPoint = rayOrigin.add(rayDir.scale(t));

                    glm::vec3 nWorld = glm::normalize(glm::vec3(normalM * glm::vec4(triN.x, triN.y, triN.z, 0.0f)));
                    info.hitNormal = Vec3(nWorld);
                    if (info.hitNormal.dot(rayDir) > 0) info.hitNormal = info.hitNormal.scale(-1.0f);

                    info.i0 = i0; info.i1 = i1; info.i2 = i2;
                    info.shading = baseMesh->shadingMode;
                    info.hitMesh = baseMesh;
                    info.hitInstance = &instance;
                    info.modelMatrix = instance.modelMatrix;
                }
            }
        }
    }

    // Plane intersection
    for (const auto& plane : scene.objects.planes) {
        float t;
        glm::mat4 M = plane.modelMatrix;
        glm::mat4 normalM = glm::transpose(glm::inverse(M));

        Vec3 p0_local = scene.vertexData[plane.point - 1];
        glm::vec3 p0_world = glm::vec3(M * glm::vec4(p0_local.x, p0_local.y, p0_local.z, 1.0f));
        glm::vec3 N_world  = glm::normalize(glm::vec3(normalM * glm::vec4(plane.normal.x, plane.normal.y, plane.normal.z, 0.0f)));

        if (glm::determinant(M) < 0.0f) // Flip if mirror transform
            N_world = -N_world;

        if (RayPlane(rayOrigin, rayDir, Vec3(p0_world), Vec3(N_world), t)) {
            if (t > intersectionTestEpsilon && t < closestT) {
                closestT = t;
                info.hit = true;
                info.materialId = plane.materialId;
                info.t = t;
                info.hitPoint = rayOrigin.add(rayDir.scale(t));
                info.hitNormal = Vec3(N_world);

                if (info.hitNormal.dot(rayDir) > 0)
                    info.hitNormal = info.hitNormal.scale(-1.0f);
            }
        }
    }

    // If smooth shading, compute interpolated normal
    if (info.hit && info.hitMesh && info.shading == ShadingMode::Smooth) {
        const glm::mat4 modelM = info.modelMatrix;
        glm::mat4 invM = glm::inverse(modelM);

        // Hit point in local space
        glm::vec4 localHit = invM * glm::vec4(info.hitPoint.x, info.hitPoint.y, info.hitPoint.z, 1.0f);
        Vec3 P_local(localHit.x, localHit.y, localHit.z);

        const Vec3 &sv0 = scene.vertexData[info.i0];
        const Vec3 &sv1 = scene.vertexData[info.i1];
        const Vec3 &sv2 = scene.vertexData[info.i2];

        // Local-space barycentric coordinates
        float w0, w1, w2;
        Barycentrics(P_local, sv0, sv1, sv2, w0, w1, w2);

        // Pick the correct normal array (instance or base mesh)
        const vector<Vec3> &normals =
                (info.hitInstance != nullptr) ? info.hitInstance->perVertexNormal
                                              : info.hitMesh->perVertexNormal;

        // Per-vertex normals
        const Vec3 &n0 = normals[info.i0];
        const Vec3 &n1 = normals[info.i1];
        const Vec3 &n2 = normals[info.i2];

        // Interpolated normal
        Vec3 n = n0.scale(w0).add(n1.scale(w1)).add(n2.scale(w2));
        if (n.length_squared() > 0.0f) {
            n = n.normalize();

            // Flip normal if needed
            if (n.dot(rayDir) > 0)
                n = n.scale(-1.0f);

            info.hitNormal = n;
        }
    }

    return info;
}
