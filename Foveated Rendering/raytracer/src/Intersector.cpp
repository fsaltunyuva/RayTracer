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
    if (N_len_sq < 1e-20f) { // TODO: Get epsilon from scene
        return false; // Degenerate triangle
    }

    float NdotD = N.dot(rayDir);

    if (fabs(NdotD) < 1e-12f) return false; // Plane and ray are parallel (TODO: Get epsilon from scene)

    if (backFaceCull && NdotD >= 0.0f) return false; // Early Back-face culling

    // Compute t (intersection point) using plane equation
    t = N.dot(v0.subtract(rayOrigin)) / NdotD;
    if (t < 0) return false; // The triangle is behind the ray

    // Check if the intersection point is inside the triangle using edge function
    Vec3 P = rayOrigin.add(rayDir.scale(t));

    // Inside test
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

    hitNormalOut = N; // Not normalized yet

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
                                          bool isShadowRay, float rayTime) const
{
    IntersectionInfo info;
    info.isEmissive = false;

    float intersectionTestEpsilon;
    if (isShadowRay) intersectionTestEpsilon = scene.shadowRayEpsilon;
    else intersectionTestEpsilon = scene.intersectionTestEpsilon;

    float closestT = numeric_limits<float>::max();

    // Sphere intersection
    for (const auto& sphere : scene.objects.spheres) {
        glm::mat4 M = sphere.modelMatrix;
        glm::mat4 invM = glm::inverse(M);

        // Motion blur offset (world space)
        Vec3 blurOffset = sphere.motionBlur.scale(rayTime);

        // Treat object as static, move ray origin instead
        Vec3 rayOriginBlur = rayOrigin.subtract(blurOffset);

        // Transform ray to local space of sphere
        glm::vec3 ro_local = glm::vec3(invM * glm::vec4(rayOriginBlur.x, rayOriginBlur.y, rayOriginBlur.z, 1.0f));
        glm::vec3 rd_local = glm::vec3(invM * glm::vec4(rayDir.x, rayDir.y, rayDir.z, 0.0f));

        Vec3 centerLocal = scene.vertexData[sphere.center - 1];
        float t[2];
        if (RaySphere(Vec3(ro_local), Vec3(rd_local), centerLocal, sphere.radius, t)) {
            float tCandidate = std::numeric_limits<float>::max();

            if (t[0] > intersectionTestEpsilon) tCandidate = t[0];
            if (t[1] > intersectionTestEpsilon && t[1] < tCandidate) tCandidate = t[1];

            if (tCandidate < std::numeric_limits<float>::max()) {
                glm::vec3 localHit = ro_local + tCandidate * rd_local;
                glm::vec3 worldHitStatic = glm::vec3(M * glm::vec4(localHit, 1.0f));

                // Apply motion blur offset to world hit point
                glm::vec3 blurOffsetVec(blurOffset.x, blurOffset.y, blurOffset.z);
                glm::vec3 worldHit = worldHitStatic + blurOffsetVec;

                float tWorld = glm::length(worldHit - glm::vec3(rayOrigin.x, rayOrigin.y, rayOrigin.z));

                if (tWorld > intersectionTestEpsilon && tWorld < closestT) {
                    // localN: unit normal in object/local space
                    glm::vec3 localN = glm::normalize(localHit - glm::vec3(centerLocal.x, centerLocal.y, centerLocal.z));

                    // Spherical UV mapping
                    float phi = std::atan2(localN.z, localN.x);
                    float theta = std::acos(std::clamp(localN.y, -1.0f, 1.0f));

                    float u = (-phi + (float) M_PI) / (2.0f * (float) M_PI);
                    float v = theta / (float) M_PI;

                    u = u - std::floor(u); // Wrap [0,1)

                    // Store UVs
                    info.hitUV = Vec2(u, v);
                    info.hasUV = true;

                    // dphi and dtheta for unit sphere parameterization
                    float sinT = std::sin(theta);
                    float cosT = std::cos(theta);
                    float sinP = std::sin(phi);
                    float cosP = std::cos(phi);

                    // Tangent and Bitangent in local space
                    glm::vec3 Tlocal(-sinT * sinP, 0.0f, sinT * cosP);
                    glm::vec3 Blocal(cosT * cosP, -sinT, cosT * sinP);

                    // Transform to world using normal matrix (same as normals w=0)
                    glm::mat4 normalM = glm::transpose(glm::inverse(M));
                    glm::vec3 Tw = glm::normalize(glm::vec3(normalM * glm::vec4(Tlocal, 0.0f)));
                    glm::vec3 Bw = glm::normalize(glm::vec3(normalM * glm::vec4(Blocal, 0.0f)));

                    // Orthonormalize against world normal (to avoid issues if scaled non-uniformly)
                    glm::vec3 Nw = glm::normalize(glm::vec3(normalM * glm::vec4(localN, 0.0f)));
                    Tw = glm::normalize(Tw - Nw * glm::dot(Nw, Tw));
                    Bw = glm::normalize(glm::cross(Nw, Tw));

                    // Store TBN
                    info.tangentW = Vec3(Tw.x, Tw.y, Tw.z);
                    info.bitangentW = Vec3(Bw.x, Bw.y, Bw.z);
                    info.hasTBN = true;

                    glm::vec3 worldN = glm::normalize(glm::vec3(glm::transpose(glm::inverse(M)) * glm::vec4(localN, 0.0f)));

                    info.modelMatrix = M;
                    info.textureMapIds = &sphere.textureMapIds;
                    closestT = tWorld;
                    info.hit = true;
                    info.materialId = sphere.materialId;
                    info.t = tWorld;
                    info.isEmissive = false;
                    info.hitPoint = Vec3(worldHit.x, worldHit.y, worldHit.z);
                    info.hitNormal = Vec3(worldN.x, worldN.y, worldN.z);
                    if (info.hitNormal.dot(rayDir) > 0)
                        info.hitNormal = info.hitNormal.scale(-1.0f);
                }
            }
        }
    }

    // Light Sphere intersection
    for (const auto &ls: scene.objects.lightSpheres) {
        glm::mat4 M = ls.modelMatrix;
        glm::mat4 invM = glm::inverse(M);

        // Motion blur offset (world space)
        Vec3 blurOffset = ls.motionBlur.scale(rayTime);

        // Treat object as static, move ray origin instead
        Vec3 rayOriginBlur = rayOrigin.subtract(blurOffset);

        // Transform ray to local space of sphere
        glm::vec3 ro_local = glm::vec3(invM * glm::vec4(rayOriginBlur.x, rayOriginBlur.y, rayOriginBlur.z, 1.0f));
        glm::vec3 rd_local = glm::vec3(invM * glm::vec4(rayDir.x, rayDir.y, rayDir.z, 0.0f));

        Vec3 centerLocal = scene.vertexData[ls.center - 1];
        float t[2];
        if (RaySphere(Vec3(ro_local), Vec3(rd_local), centerLocal, ls.radius, t)) {
            float tCandidate = std::numeric_limits<float>::max();

            if (t[0] > intersectionTestEpsilon) tCandidate = t[0];
            if (t[1] > intersectionTestEpsilon && t[1] < tCandidate) tCandidate = t[1];

            if (tCandidate < std::numeric_limits<float>::max()) {
                glm::vec3 localHit = ro_local + tCandidate * rd_local;
                glm::vec3 worldHitStatic = glm::vec3(M * glm::vec4(localHit, 1.0f));

                // Apply motion blur offset to world hit point
                glm::vec3 blurOffsetVec(blurOffset.x, blurOffset.y, blurOffset.z);
                glm::vec3 worldHit = worldHitStatic + blurOffsetVec;

                float tWorld = glm::length(worldHit - glm::vec3(rayOrigin.x, rayOrigin.y, rayOrigin.z));

                if (tWorld > intersectionTestEpsilon && tWorld < closestT) {
                    // localN: unit normal in object/local space
                    glm::vec3 localN = glm::normalize(localHit - glm::vec3(centerLocal.x, centerLocal.y, centerLocal.z));

                    // Spherical UV mapping
                    float phi = std::atan2(localN.z, localN.x);
                    float theta = std::acos(std::clamp(localN.y, -1.0f, 1.0f));

                    float u = (-phi + (float) M_PI) / (2.0f * (float) M_PI);
                    float v = theta / (float) M_PI;

                    u = u - std::floor(u); // Wrap [0,1)

                    // Store UVs
                    info.hitUV = Vec2(u, v);
                    info.hasUV = true;

                    // dphi and dtheta for unit sphere parameterization
                    float sinT = std::sin(theta);
                    float cosT = std::cos(theta);
                    float sinP = std::sin(phi);
                    float cosP = std::cos(phi);

                    // Tangent and Bitangent in local space
                    glm::vec3 Tlocal(-sinT * sinP, 0.0f, sinT * cosP);
                    glm::vec3 Blocal(cosT * cosP, -sinT, cosT * sinP);

                    // Transform to world using normal matrix (same as normals w=0)
                    glm::mat4 normalM = glm::transpose(glm::inverse(M));
                    glm::vec3 Tw = glm::normalize(glm::vec3(normalM * glm::vec4(Tlocal, 0.0f)));
                    glm::vec3 Bw = glm::normalize(glm::vec3(normalM * glm::vec4(Blocal, 0.0f)));

                    // Orthonormalize against world normal (to avoid issues if scaled non-uniformly)
                    glm::vec3 Nw = glm::normalize(glm::vec3(normalM * glm::vec4(localN, 0.0f)));
                    Tw = glm::normalize(Tw - Nw * glm::dot(Nw, Tw));
                    Bw = glm::normalize(glm::cross(Nw, Tw));

                    // Store TBN
                    info.tangentW = Vec3(Tw.x, Tw.y, Tw.z);
                    info.bitangentW = Vec3(Bw.x, Bw.y, Bw.z);
                    info.hasTBN = true;

                    glm::vec3 worldN = glm::normalize(glm::vec3(glm::transpose(glm::inverse(M)) * glm::vec4(localN, 0.0f)));

                    info.modelMatrix = M;
                    info.textureMapIds = &ls.textureMapIds;
                    closestT = tWorld;
                    info.hit = true;
                    info.materialId = ls.materialId;
                    info.t = tWorld;
                    info.hitPoint = Vec3(worldHit.x, worldHit.y, worldHit.z);
                    info.hitNormal = Vec3(worldN.x, worldN.y, worldN.z);

                    // Light Sphere specific
                    info.isEmissive = true;
                    info.emission = ls.radiance;

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

        // Motion blur offset
        Vec3 blurOffset = tri.motionBlur.scale(rayTime);
        Vec3 rayOriginBlur = rayOrigin.subtract(blurOffset);

        Vec3 triN;

        if (RayTriangle(rayOriginBlur, rayDir, v0, v1, v2, t, performCulling, triN)) {
            if (t > intersectionTestEpsilon && t < closestT) {
                closestT = t;
                info.hit = true;
                info.materialId = tri.materialId;
                info.t = t;
                info.hitPoint = rayOrigin.add(rayDir.scale(t));
                info.textureMapIds = &tri.textureMapIds;
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

            Vec3 blurOffset = mesh.motionBlur.scale(rayTime);
            Vec3 rayOriginBlur = rayOrigin.subtract(blurOffset);

            if (mesh.bvh->intersect(rayOriginBlur, rayDir, tCandidate, info, intersectionTestEpsilon)) {
                if (tCandidate < closestT) {
                    closestT = tCandidate;

                    info.textureMapIds = &mesh.textureMapIds;
                    info.materialId = mesh.materialId;
                    info.hitMesh = &mesh;
                    info.hitInstance = nullptr;
                    info.shading = mesh.shadingMode;
                    info.modelMatrix = mesh.modelMatrix;

                    // t and real hit point correction with original ray
                    info.t = tCandidate;
                    info.hitPoint = rayOrigin.add(rayDir.scale(tCandidate));

                    if (info.hitNormal.dot(rayDir) > 0)
                        info.hitNormal = info.hitNormal.scale(-1.0f);
                }
            }
            continue; // Skip triangle test if BVH is present
        }

        glm::mat4 M = mesh.modelMatrix;
        glm::mat4 normalM = glm::transpose(glm::inverse(M));

        // If determinant is negative, it's a mirrored transform (0, -1, 0 scale for example)
        //bool isMirrored = glm::determinant(M) < 0.0f; // TODO: Slows the process?

        // Motion blur offset
        Vec3 blurOffset = mesh.motionBlur.scale(rayTime);
        Vec3 rayOriginBlur = rayOrigin.subtract(blurOffset);

        for (size_t i = 0; i + 2 < mesh.data.size(); i += 3) {
            int i0 = mesh.data[i] - 1, i1 = mesh.data[i+1] - 1, i2 = mesh.data[i+2] - 1;

            Vec3 v0 = Vec3(M * glm::vec4(scene.vertexData[i0].x, scene.vertexData[i0].y, scene.vertexData[i0].z, 1.0f));
            Vec3 v1 = Vec3(M * glm::vec4(scene.vertexData[i1].x, scene.vertexData[i1].y, scene.vertexData[i1].z, 1.0f));
            Vec3 v2 = Vec3(M * glm::vec4(scene.vertexData[i2].x, scene.vertexData[i2].y, scene.vertexData[i2].z, 1.0f));

            float t;
            Vec3 triN;

            bool performCulling = !isShadowRay /*&& !isMirrored*/;

            if (RayTriangle(rayOriginBlur, rayDir, v0, v1, v2, t, performCulling, triN)) {
                if (t > intersectionTestEpsilon && t < closestT) {
                    closestT = t;
                    info.hit = true;
                    info.materialId = mesh.materialId;
                    info.t = t;
                    info.hitPoint = rayOrigin.add(rayDir.scale(t));
                    info.hitNormal = triN.normalize();

                    if (info.hitNormal.dot(rayDir) > 0)
                        info.hitNormal = info.hitNormal.scale(-1.0f);

                    info.i0 = i0; info.i1 = i1; info.i2 = i2;
                    info.shading = mesh.shadingMode; // ShadingMode::Flat | Smooth
                    info.hitMesh = &mesh;
                    info.modelMatrix = mesh.modelMatrix;
                    info.hitInstance = nullptr;
                    info.textureMapIds = &mesh.textureMapIds;
                }
            }
        }
    }

    // Light Mesh Faces intersection
    for (const auto &lm: scene.objects.lightMeshes) {
        if (lm.bvh) {
            float tCandidate = closestT;

            Vec3 blurOffset = lm.motionBlur.scale(rayTime);
            Vec3 rayOriginBlur = rayOrigin.subtract(blurOffset);

            if (lm.bvh->intersect(rayOriginBlur, rayDir, tCandidate, info, intersectionTestEpsilon)) {
                if (tCandidate < closestT) {
                    closestT = tCandidate;

                    info.textureMapIds = &lm.textureMapIds;
                    info.materialId = lm.materialId;
                    info.hitMesh = &lm;
                    info.hitInstance = nullptr;
                    info.shading = lm.shadingMode;
                    info.modelMatrix = lm.modelMatrix;

                    // t and real hit point correction with original ray
                    info.t = tCandidate;
                    info.hitPoint = rayOrigin.add(rayDir.scale(tCandidate));

                    info.isEmissive = true;
                    info.emission = lm.radiance;

                    if (info.hitNormal.dot(rayDir) > 0)
                        info.hitNormal = info.hitNormal.scale(-1.0f);
                }
            }
            continue; // Skip triangle test if BVH is present
        }

        glm::mat4 M = lm.modelMatrix;
        glm::mat4 normalM = glm::transpose(glm::inverse(M));

        // If determinant is negative, it's a mirrored transform (0, -1, 0 scale for example)
        //bool isMirrored = glm::determinant(M) < 0.0f; // TODO: Slows the process?

        // Motion blur offset
        Vec3 blurOffset = lm.motionBlur.scale(rayTime);
        Vec3 rayOriginBlur = rayOrigin.subtract(blurOffset);

        for (size_t i = 0; i + 2 < lm.data.size(); i += 3) {
            int i0 = lm.data[i] - 1, i1 = lm.data[i + 1] - 1, i2 = lm.data[i + 2] - 1;

            Vec3 v0 = Vec3(M * glm::vec4(scene.vertexData[i0].x, scene.vertexData[i0].y, scene.vertexData[i0].z, 1.0f));
            Vec3 v1 = Vec3(M * glm::vec4(scene.vertexData[i1].x, scene.vertexData[i1].y, scene.vertexData[i1].z, 1.0f));
            Vec3 v2 = Vec3(M * glm::vec4(scene.vertexData[i2].x, scene.vertexData[i2].y, scene.vertexData[i2].z, 1.0f));

            float t;
            Vec3 triN;

            bool performCulling = true;

            if (RayTriangle(rayOriginBlur, rayDir, v0, v1, v2, t, performCulling, triN)) {
                if (t > intersectionTestEpsilon && t < closestT) {
                    closestT = t;
                    info.hit = true;
                    info.materialId = lm.materialId;
                    info.t = t;
                    info.hitPoint = rayOrigin.add(rayDir.scale(t));
                    info.hitNormal = triN.normalize();

                    if (info.hitNormal.dot(rayDir) > 0)
                        info.hitNormal = info.hitNormal.scale(-1.0f);

                    info.i0 = i0; info.i1 = i1; info.i2 = i2;
                    info.shading = lm.shadingMode; // ShadingMode::Flat | Smooth
                    info.hitMesh = &lm;
                    info.modelMatrix = lm.modelMatrix;
                    info.hitInstance = nullptr;
                    info.textureMapIds = &lm.textureMapIds;

                    // Light Mesh specific
                    info.isEmissive = true;
                    info.emission = lm.radiance;
                }
            }
        }
    }

    // MeshInstance intersection
    for (const auto& instance : scene.objects.meshInstances) {
        // Get base mesh (found in main.cpp)
        const Mesh* baseMesh = instance.resolvedBaseMesh;

        Vec3 blurOffset = instance.motionBlur.scale(rayTime);
        Vec3 rayOriginBlur = rayOrigin.subtract(blurOffset);

        if (instance.bvh) {
            float tCandidate = closestT;
            if (instance.bvh->intersect(rayOriginBlur, rayDir, tCandidate, info, intersectionTestEpsilon)) {
                if (tCandidate > intersectionTestEpsilon && tCandidate < closestT) {
                    closestT = tCandidate;

                    // Use instance's own material if set, otherwise use base mesh's material
                    info.materialId = (instance.materialId != -1) ? instance.materialId : baseMesh->materialId;
                    info.hitInstance = &instance;
                    info.hitMesh = baseMesh;
                    info.modelMatrix = instance.modelMatrix;
                    info.shading = baseMesh->shadingMode;

                    // TODO: Is textureMapIds also needed from instance?
                    // if (!instance.textureMapIds.empty()) // Use instance's own texture maps if available
                    //     info.textureMapIds = &instance.textureMapIds;
                    // else
                    //     info.textureMapIds = &baseMesh->textureMapIds;

                    // t and real hit point correction with original ray
                    info.t = tCandidate;
                    info.hitPoint = rayOrigin.add(rayDir.scale(tCandidate));

                    if (info.hitNormal.dot(rayDir) > 0)
                        info.hitNormal = info.hitNormal.scale(-1.0f);
                }
            }
            continue; // Skip triangle test if BVH is present
        }

        glm::mat4 M = instance.modelMatrix;
        // glm::mat4 normalM = transpose(glm::inverse(M)); // TODO: Not used?

        // If determinant is negative, it's a mirrored transform (0, -1, 0 scale for example)
        bool isMirrored = determinant(M) < 0.0f; // TODO: Slows the process?

        for (size_t i = 0; i + 2 < baseMesh->data.size(); i += 3) {
            int i0 = baseMesh->data[i] - 1, i1 = baseMesh->data[i+1] - 1, i2 = baseMesh->data[i+2] - 1;

            Vec3 v0 = Vec3(M * glm::vec4(scene.vertexData[i0].x, scene.vertexData[i0].y, scene.vertexData[i0].z, 1.0f));
            Vec3 v1 = Vec3(M * glm::vec4(scene.vertexData[i1].x, scene.vertexData[i1].y, scene.vertexData[i1].z, 1.0f));
            Vec3 v2 = Vec3(M * glm::vec4(scene.vertexData[i2].x, scene.vertexData[i2].y, scene.vertexData[i2].z, 1.0f));

            float t;
            Vec3 triN;

            bool performCulling = !isShadowRay && !isMirrored; // Back-face culling only for non-shadow rays and non-mirrored instances

            if (RayTriangle(rayOriginBlur, rayDir, v0, v1, v2, t, performCulling, triN)) {
                if (t > intersectionTestEpsilon && t < closestT) {
                    closestT = t;
                    info.hit = true;
                    info.materialId = (instance.materialId != -1) ? instance.materialId : baseMesh->materialId;
                    info.t = t;
                    info.hitPoint = rayOrigin.add(rayDir.scale(t));
                    info.hitNormal = triN.normalize();

                    if (info.hitNormal.dot(rayDir) > 0) info.hitNormal = info.hitNormal.scale(-1.0f);

                    info.i0 = i0; info.i1 = i1; info.i2 = i2;
                    info.shading = baseMesh->shadingMode;
                    info.hitMesh = baseMesh;
                    info.hitInstance = &instance;
                    info.modelMatrix = instance.modelMatrix;
                    if (!instance.textureMapIds.empty()) // Use instance's own texture maps if available
                        info.textureMapIds = &instance.textureMapIds;
                    else
                        info.textureMapIds = &baseMesh->textureMapIds;
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

        // Motion blur
        Vec3 blurOffset = plane.motionBlur.scale(rayTime);
        Vec3 rayOriginBlur = rayOrigin.subtract(blurOffset);

        if (RayPlane(rayOriginBlur, rayDir, Vec3(p0_world), Vec3(N_world), t)) {
            if (t > intersectionTestEpsilon && t < closestT) {
                closestT = t;
                info.hit = true;
                info.materialId = plane.materialId;
                info.t = t;
                info.hitPoint = rayOrigin.add(rayDir.scale(t));
                info.hitNormal = Vec3(N_world);
                info.textureMapIds = &plane.textureMapIds;
                info.modelMatrix = M;

                // Compute tangent and bitangent for plane
                Vec3 N = info.hitNormal.normalize();
                Vec3 tmp = (std::fabs(N.x) > 0.9f) ? Vec3(0,0,1) : Vec3(1,0,0);
                Vec3 T = tmp.cross(N).normalize();
                Vec3 B = N.cross(T).normalize();
                info.tangentW = T;
                info.bitangentW = B;
                info.hasTBN = true;

                if (info.hitNormal.dot(rayDir) > 0)
                    info.hitNormal = info.hitNormal.scale(-1.0f);
            }
        }
    }

    //  Compute barycentric coordinates if triangle hit
    if (info.hit && info.hitMesh && info.i0 >= 0 && info.i1 >= 0 && info.i2 >= 0) {
        const glm::mat4 modelM = info.modelMatrix;
        glm::mat4 invM = glm::inverse(modelM);

        // Rollback motion blur for barycentric calc (same logic you already use)
        Vec3 P_world = info.hitPoint;
        if (info.hitInstance != nullptr) {
            P_world = P_world.subtract(info.hitInstance->motionBlur.scale(rayTime));
        } else {
            P_world = P_world.subtract(info.hitMesh->motionBlur.scale(rayTime));
        }

        // Hit point in local space
        glm::vec4 localHit = invM * glm::vec4(P_world.x, P_world.y, P_world.z, 1.0f);
        Vec3 P_local(localHit.x, localHit.y, localHit.z);

        const Vec3 &sv0 = scene.vertexData[info.i0];
        const Vec3 &sv1 = scene.vertexData[info.i1];
        const Vec3 &sv2 = scene.vertexData[info.i2];

        float w0, w1, w2;
        Barycentrics(P_local, sv0, sv1, sv2, w0, w1, w2);

        info.w0 = w0; info.w1 = w1; info.w2 = w2;
        info.hasBarycentrics = true;

        // UV (if available)
        if (!scene.texCoordData.empty()) {
            int ti0 = info.i0;
            int ti1 = info.i1;
            int ti2 = info.i2;

            if (info.hitMesh) { // If hitMesh is valid, adjust texture indices
                ti0 = (info.i0 - info.hitMesh->vertexOffset) + info.hitMesh->textureOffset;
                ti1 = (info.i1 - info.hitMesh->vertexOffset) + info.hitMesh->textureOffset;
                ti2 = (info.i2 - info.hitMesh->vertexOffset) + info.hitMesh->textureOffset;
            }

            // Safe UV check (if out of bounds, return (0,0))
            auto safeUV = [&](int idx) -> Vec2 {
                if (idx >= 0 && idx < scene.texCoordData.size()) return scene.texCoordData[idx];
                return Vec2(0,0);
            };

            const Vec2 &uv0 = safeUV(ti0);
            const Vec2 &uv1 = safeUV(ti1);
            const Vec2 &uv2 = safeUV(ti2);

            // Interpolated UV
            info.hitUV = Vec2(
                uv0.u * w0 + uv1.u * w1 + uv2.u * w2,
                uv0.v * w0 + uv1.v * w1 + uv2.v * w2
            );
            info.hasUV = true;

            // TBN Compute for normal mapping (world space)
            // Local triangle positions
            const Vec3 &p0L = scene.vertexData[info.i0];
            const Vec3 &p1L = scene.vertexData[info.i1];
            const Vec3 &p2L = scene.vertexData[info.i2];

            // World positions
            glm::mat4 M = info.modelMatrix;
            auto toWorld = [&](const Vec3& p) {
                glm::vec4 w = M * glm::vec4(p.x, p.y, p.z, 1.0f);
                return Vec3(w.x, w.y, w.z);
            };

            Vec3 p0 = toWorld(p0L);
            Vec3 p1 = toWorld(p1L);
            Vec3 p2 = toWorld(p2L);

            Vec3 e1 = p1.subtract(p0);
            Vec3 e2 = p2.subtract(p0);

            float du1 = uv1.u - uv0.u;
            float dv1 = uv1.v - uv0.v;
            float du2 = uv2.u - uv0.u;
            float dv2 = uv2.v - uv0.v;

            float det = du1 * dv2 - dv1 * du2; // Determinant

            if (std::fabs(det) > 1e-12f) {
                float invDet = 1.0f / det;

                Vec3 T = (e1.scale(dv2).subtract(e2.scale(dv1))).scale(invDet);
                Vec3 B = (e2.scale(du1).subtract(e1.scale(du2))).scale(invDet);

                // Orthonormalize with current shading normal
                Vec3 N = info.hitNormal.normalize();
                T = T.subtract(N.scale(N.dot(T))).normalize();

                // Handedness fix (mirrored UVs)
                float handedness = (det < 0.0f) ? -1.0f : 1.0f;
                B = (N.cross(T)).scale(handedness).normalize();

                info.tangentW = T;
                info.bitangentW = B;
                info.hasTBN = true;
            }
        }
    }

    // If smooth shading, compute interpolated normal
    if (info.hit && info.hitMesh && info.shading == ShadingMode::Smooth) {
        const glm::mat4 modelM = info.modelMatrix;
        glm::mat4 invM = glm::inverse(modelM);
        glm::mat4 normalM = glm::transpose(glm::inverse(modelM));

        Vec3 P_world = info.hitPoint;
        if (info.hitInstance != nullptr) {
            P_world = P_world.subtract(info.hitInstance->motionBlur.scale(rayTime));
        }
        else {
            P_world = P_world.subtract(info.hitMesh->motionBlur.scale(rayTime));
        }

        // Hit point in local space
        glm::vec4 localHit = invM * glm::vec4(P_world.x, P_world.y, P_world.z, 1.0f);
        Vec3 P_local(localHit.x, localHit.y, localHit.z);

        const Vec3 &sv0 = scene.vertexData[info.i0];
        const Vec3 &sv1 = scene.vertexData[info.i1];
        const Vec3 &sv2 = scene.vertexData[info.i2];

        float w0, w1, w2;
        Barycentrics(P_local, sv0, sv1, sv2, w0, w1, w2);

        const std::vector<Vec3> &normals =(info.hitInstance != nullptr) ? info.hitInstance->perVertexNormal : info.hitMesh->perVertexNormal;

        int ni0 = info.i0, ni1 = info.i1, ni2 = info.i2;
        if (info.hitMesh) {
            ni0 = info.i0 - info.hitMesh->vertexOffset;
            ni1 = info.i1 - info.hitMesh->vertexOffset;
            ni2 = info.i2 - info.hitMesh->vertexOffset;
        }

        // Safety check for normals
        if (ni0 < 0 || ni1 < 0 || ni2 < 0 || ni0 >= (int) normals.size() || ni1 >= (int) normals.size() || ni2 >= (int) normals.size()) {
            return info;
        }

        // Interpolate in local space
        Vec3 nL = normals[ni0].scale(w0).add(normals[ni1].scale(w1)).add(normals[ni2].scale(w2));
        if (nL.length_squared() > 0.0f) {
            nL = nL.normalize();

            // Transform normal to world space
            glm::vec3 nW = glm::normalize(glm::vec3(normalM * glm::vec4(nL.x, nL.y, nL.z, 0.0f)));
            Vec3 n(Vec3(nW.x, nW.y, nW.z));

            // Flip if needed
            if (n.dot(rayDir) > 0.0f) n = n.scale(-1.0f);

            info.hitNormal = n;

            // Recompute TBN if available
            if (info.hasTBN) {
                Vec3 N = info.hitNormal.normalize();
                Vec3 T = info.tangentW;
                T = T.subtract(N.scale(N.dot(T))).normalize();
                Vec3 B = N.cross(T).normalize();
                info.tangentW = T;
                info.bitangentW = B;
            }
        }
    }


    return info;
}
