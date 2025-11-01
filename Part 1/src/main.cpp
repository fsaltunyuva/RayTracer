#include <fstream>
#include <iostream>
#include <chrono>
#include "../json/json.hpp"
#include <omp.h> // OpenMP for parallel processing

#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "../stb/stb_image_write.h"

#include "../include/Scene.h"

using json = nlohmann::json;
using namespace std;

static bool rayIntersectsSphere(Vec3 rayOrigin, Vec3 rayDir, Vec3 sphereCenter, float radius, float t[]);
static bool rayIntersectsTriangle(const Vec3& rayOrigin, const Vec3& rayDir, const Vec3& v0, const Vec3& v1, const Vec3& v2, float &t, bool backFaceCull, Vec3 &hitNormalOut);
static bool isInShadow(const Scene& scene, const Vec3& shadowRayOrigin, const Vec3& shadowRayDir, float lightDistance);
static bool rayIntersectsPlane(const Vec3& rayOrigin, const Vec3& rayDir, const Vec3& planePoint, const Vec3& planeNormal, float &t);
static void barycentrics(const Vec3& P, const Vec3& v0, const Vec3& v1, const Vec3& v2, float &w0, float &w1, float &w2);

Vec3 traceRay(const Scene& scene, const Vec3& rayOrigin, const Vec3& rayDir, int depth);

// Reflect and Refract Functions
static Vec3 reflect(const Vec3& I, const Vec3& N);
static bool refract(const Vec3& I, const Vec3& N, float eta, Vec3& T);
static float fresnelDielectric(float cosI, float eta);
static Vec3 fresnelConductor(const Vec3& I, const Vec3& N, float eta, float k);

struct IntersectionInfo {
    bool hit = false;
    float t = numeric_limits<float>::max(); // Intersection distance
    int materialId = -1;
    Vec3 hitPoint;
    Vec3 hitNormal;

    int i0 = -1, i1 = -1, i2 = -1;
    ShadingMode shading = ShadingMode::Flat;
    const Mesh* hitMesh = nullptr;
};

static IntersectionInfo findClosestIntersection(const Scene& scene, const Vec3& rayOrigin, const Vec3& rayDir, bool isShadowRay);

int main(int argc, char *argv[]) {
    string inputFile;

    if (argc > 1) {
        inputFile = argv[1];
    }

    Scene scene;
    if (!scene.loadScene(inputFile)) {
        cerr << "JSON Parse Error." << std::endl;
        return 1;
    }

    auto start_total = chrono::high_resolution_clock::now();

    // Precompute per-vertex normals for smooth shaded meshes
    for (auto &mesh : scene.objects.meshes) {
        if (mesh.shadingMode != ShadingMode::Smooth) continue;

        mesh.perVertexNormal.assign(scene.vertexData.size(), Vec3(0,0,0));

        for (size_t i = 0; i + 2 < mesh.data.size(); i += 3) {
            int i0 = mesh.data[i] - 1;
            int i1 = mesh.data[i+1] - 1;
            int i2 = mesh.data[i+2] - 1;
            const Vec3 &v0 = scene.vertexData[i0];
            const Vec3 &v1 = scene.vertexData[i1];
            const Vec3 &v2 = scene.vertexData[i2];
            Vec3 faceN = v1.subtract(v0).cross(v2.subtract(v0));
            mesh.perVertexNormal[i0] = mesh.perVertexNormal[i0].add(faceN);
            mesh.perVertexNormal[i1] = mesh.perVertexNormal[i1].add(faceN);
            mesh.perVertexNormal[i2] = mesh.perVertexNormal[i2].add(faceN);
        }
        for (size_t vi = 0; vi < mesh.perVertexNormal.size(); ++vi) { // Normalize
            if (mesh.perVertexNormal[vi].length_squared() > 0.0f)
                mesh.perVertexNormal[vi] = mesh.perVertexNormal[vi].normalize();
        }
    }

    for (size_t ci = 0; ci < scene.cameras.size(); ++ci) { // For each camera
        const auto &cam = scene.cameras[ci];

        int width = cam.imageResolution.x;
        int height = cam.imageResolution.y;

        float left = cam.nearPlane[0];
        float right = cam.nearPlane[1];
        float bottom = cam.nearPlane[2];
        float top = cam.nearPlane[3];
        float nearDist = cam.nearDistance;

        Vec3 camPos = cam.position;
        Vec3 gaze = cam.gaze.normalize();
        Vec3 up = cam.up.normalize();

        Vec3 w = gaze.scale(-1.0f); // -gaze
        Vec3 u_vec = (up.cross(w)).normalize();
        Vec3 v_vec = w.cross(u_vec);

        std::vector<unsigned char> pixels(width * height * 3);

        auto start_cam = chrono::high_resolution_clock::now();

#pragma omp parallel for schedule(dynamic)
        for (int y = 0; y < height; ++y) {
            for (int x = 0; x < width; ++x) {
                // Image plane coordinates
                // Normalize coordinates to [-1, 1]
                float su = left + (right - left) * (x + 0.5f) / width;
                float sv = bottom + (top - bottom) * (y + 0.5f) / height;

                // Point on image plane
                Vec3 P = camPos
                        .subtract(w.scale(nearDist))
                        .add(u_vec.scale(su))
                        .add(v_vec.scale(sv));

                // Ray direction
                Vec3 rayDir = (P.subtract(camPos)).normalize();

                // Flip y coordinate for image origin at a bottom-left (In PNG, origin is at top-left)
                int flippedY = height - 1 - y;
                int index = (flippedY * width + x) * 3;

                // Shading
                Vec3 color = traceRay(scene, camPos, rayDir, 0);

                auto clamp255 = [](float v) { return v < 0.0f ? 0.0f : (v > 255.0f ? 255.0f : v); };
                pixels[index + 0] = static_cast<unsigned char>(clamp255(color.x));
                pixels[index + 1] = static_cast<unsigned char>(clamp255(color.y));
                pixels[index + 2] = static_cast<unsigned char>(clamp255(color.z));
            }
        }

        stbi_write_png(
                cam.imageName.c_str(),
                width, height,
                3, // RGB
                pixels.data(),
                width * 3
        );

        auto end_cam = chrono::high_resolution_clock::now();
        chrono::duration<double> elapsed_cam = end_cam - start_cam;
        cout << "[Camera " << ci << "] " << cam.imageName
                  << " rendered in " << elapsed_cam.count() << " s.\n";
        cout << "Image saved to " << cam.imageName << std::endl;
    }

    auto end_total = chrono::high_resolution_clock::now();
    chrono::duration<double> elapsed_total = end_total - start_total;
    cout << "All cameras rendered in " << elapsed_total.count() << " s.\n";

    return 0;
}

#pragma region Intersection Functions

// Ray-sphere intersection function
static bool rayIntersectsSphere(Vec3 rayOrigin, Vec3 rayDir, Vec3 sphereCenter, float radius, float t[]) {
    Vec3 pc = rayOrigin.subtract(sphereCenter); // p - c

    float a = rayDir.dot(rayDir); // d . d
    float b = 2.0f * pc.dot(rayDir); // 2d . (p - c)
    float c = pc.dot(pc) - radius * radius; // (p - c) . (p - c) - r^2

    float discriminant = b * b - 4 * a * c; // b^2 - 4ac

    if (discriminant < 0) {
        return false;  // No intersection
    } else {
        t[0] = (-b - (float) sqrt(discriminant)) / (2.0f * a); // Nearest intersection point
        t[1] = (-b + (float) sqrt(discriminant)) / (2.0f * a); // Farthest intersection point
        return true;
    }
}

// Ray-triangle intersection function (barycentric / plane method)
static bool rayIntersectsTriangle(
        const Vec3& rayOrigin,
        const Vec3& rayDir,
        const Vec3& v0,
        const Vec3& v1,
        const Vec3& v2,
        float &t,
        bool backFaceCull,
        Vec3 &hitNormalOut // non normalized normal
) {
    Vec3 edge1 = v1.subtract(v0);
    Vec3 edge2 = v2.subtract(v0);
    Vec3 N = edge1.cross(edge2);

    // Degenerate triangle check
    float N_len_sq = N.length_squared();

    // If the area is too small, consider it degenerate
    if (N_len_sq < 1e-12f) { // TODO: Get epsilon from scene
        return false; // Degenerate triangle
    }

    float NdotD = N.dot(rayDir);

    if (fabs(NdotD) < 1e-8) return false; // Plane and ray are parallel (TODO: Get epsilon from scene)

    if (backFaceCull && NdotD >= 0.0f) return false; // Early Back-face culling

    // Compute t (intersection point) using plane equation
    t = N.dot(v0.subtract(rayOrigin)) / NdotD;
    if (t < 0) return false; // The Triangle is behind the ray

    // Check if the intersection point is inside the triangle using edge function
    Vec3 P = rayOrigin.add(rayDir.scale(t));

    Vec3 C;

    Vec3 edge0 = v1.subtract(v0);
    Vec3 vp0 = P.subtract(v0);
    C = edge0.cross(vp0);
    if (N.dot(C) < 0) return false;

    Vec3 edge1_ = v2.subtract(v1);
    Vec3 vp1 = P.subtract(v1);
    C = edge1_.cross(vp1);
    if (N.dot(C) < 0) return false;

    Vec3 edge2_ = v0.subtract(v2);
    Vec3 vp2 = P.subtract(v2);
    C = edge2_.cross(vp2);
    if (N.dot(C) < 0) return false;

    hitNormalOut = N; // not normalized yet

    return true;
}

static bool rayIntersectsPlane(
        const Vec3& rayOrigin,
        const Vec3& rayDir,
        const Vec3& planePoint,
        const Vec3& planeNormal,
        float &t
) {
    // T = (P - RayOrigin) . Normal / (RayDir . Normal)
    // RayDir . Normal = 0 -> Parallel
    float NdotD = planeNormal.dot(rayDir);

    if (fabs(NdotD) < 1e-6) return false;

    Vec3 P_minus_Origin = planePoint.subtract(rayOrigin);

    float NdotP = planeNormal.dot(P_minus_Origin);

    t = NdotP / NdotD;

    return t > 0;
}

static IntersectionInfo findClosestIntersection(const Scene& scene, const Vec3& rayOrigin, const Vec3& rayDir, bool isShadowRay = false) {
    IntersectionInfo info;

    float intersectionTestEpsilon;

    if(isShadowRay) intersectionTestEpsilon = scene.shadowRayEpsilon;
    else intersectionTestEpsilon = scene.intersectionTestEpsilon;

    float closestT = numeric_limits<float>::max();

    // Sphere intersection
    for (const auto& sphere : scene.objects.spheres) {
        float t[2];
        if (rayIntersectsSphere(rayOrigin, rayDir, scene.vertexData[sphere.center - 1], sphere.radius, t)) {
            float tCandidate = std::numeric_limits<float>::max();

            if (t[0] > intersectionTestEpsilon) tCandidate = t[0];
            if (t[1] > intersectionTestEpsilon && t[1] < tCandidate) tCandidate = t[1];

            if (tCandidate < closestT) {
                closestT = tCandidate;
                info.hit = true;
                info.materialId = sphere.materialId;
                info.t = closestT;
                info.hitPoint = rayOrigin.add(rayDir.scale(closestT));
                info.hitNormal = (info.hitPoint.subtract(scene.vertexData[sphere.center - 1])).normalize();
            }
        }
    }

    // Triangle intersection
    for (const auto& tri : scene.objects.triangles) {
        float tTri;
        const Vec3& v0 = scene.vertexData[tri.indices[0] - 1]; // -1 for 0-based index
        const Vec3& v1 = scene.vertexData[tri.indices[1] - 1];
        const Vec3& v2 = scene.vertexData[tri.indices[2] - 1];

        Vec3 triNormalNotNormalized;

        if (rayIntersectsTriangle(rayOrigin, rayDir, v0, v1, v2, tTri, !isShadowRay, triNormalNotNormalized)) {
            if (tTri > intersectionTestEpsilon && tTri < closestT) {
                closestT = tTri;
                info.hit = true;
                info.materialId = tri.materialId;
                info.t = closestT;
                info.hitPoint = rayOrigin.add(rayDir.scale(closestT));

                info.hitNormal = triNormalNotNormalized.normalize();

                if (info.hitNormal.dot(rayDir) > 0) info.hitNormal = info.hitNormal.scale(-1.0f);
            }
        }
    }

    // Mesh Faces intersection
    for (const auto& mesh : scene.objects.meshes) {
        for (size_t i = 0; i + 2 < mesh.data.size(); i += 3) {
            float tTri;
            const Vec3& v0 = scene.vertexData[mesh.data[i] - 1]; // -1 for 0-based index
            const Vec3& v1 = scene.vertexData[mesh.data[i + 1] - 1];
            const Vec3& v2 = scene.vertexData[mesh.data[i + 2] - 1];

            Vec3 triNormalNotNormalized;

            if (rayIntersectsTriangle(rayOrigin, rayDir, v0, v1, v2, tTri, !isShadowRay, triNormalNotNormalized)) {
                if (tTri > intersectionTestEpsilon && tTri < closestT) {
                    closestT = tTri;
                    info.hit = true;
                    info.materialId = mesh.materialId;
                    info.t = closestT;
                    info.hitPoint = rayOrigin.add(rayDir.scale(closestT));

                    info.hitNormal = triNormalNotNormalized.normalize();

                    if (info.hitNormal.dot(rayDir) > 0) info.hitNormal = info.hitNormal.scale(-1.0f);
                    info.i0 = mesh.data[i]     - 1;
                    info.i1 = mesh.data[i + 1] - 1;
                    info.i2 = mesh.data[i + 2] - 1;
                    info.shading = mesh.shadingMode; // ShadingMode::Flat | Smooth
                    info.hitMesh = &mesh;
                }
            }
        }
    }

    // Plane intersection
    for (const auto& plane : scene.objects.planes) {
        float tPlane;
        const Vec3& p0 = scene.vertexData[plane.point - 1]; // -1 for 0-based index
        const Vec3& N = plane.normal;

        if (rayIntersectsPlane(rayOrigin, rayDir, p0, N, tPlane)) {
            if (tPlane > intersectionTestEpsilon && tPlane < closestT) {
                closestT = tPlane;
                info.hit = true;
                info.materialId = plane.materialId;
                info.t = closestT;
                info.hitPoint = rayOrigin.add(rayDir.scale(closestT));

                info.hitNormal = N;

                // Ensure the normal is facing against the ray
                if (info.hitNormal.dot(rayDir) > 0) info.hitNormal = info.hitNormal.scale(-1.0f);
            }
        }
    }

    // If smooth shading, compute interpolated normal
    if (info.hit && info.hitMesh && info.shading == ShadingMode::Smooth) {
        const Vec3 &sv0 = scene.vertexData[info.i0];
        const Vec3 &sv1 = scene.vertexData[info.i1];
        const Vec3 &sv2 = scene.vertexData[info.i2];

        float w0, w1, w2;
        barycentrics(info.hitPoint, sv0, sv1, sv2, w0, w1, w2);

        const Vec3 &n0 = info.hitMesh->perVertexNormal[info.i0];
        const Vec3 &n1 = info.hitMesh->perVertexNormal[info.i1];
        const Vec3 &n2 = info.hitMesh->perVertexNormal[info.i2];

        Vec3 n = n0.scale(w0).add(n1.scale(w1)).add(n2.scale(w2));
        if (n.length_squared() > 0.0f) {
            n = n.normalize();
            if (n.dot(rayDir) > 0) n = n.scale(-1.0f); // Ensure normal is against the ray
            info.hitNormal = n; // Update normal to interpolated normal
        }
    }

    return info;
}

#pragma endregion

static bool isInShadow(const Scene& scene, const Vec3& shadowRayOrigin, const Vec3& shadowRayDir, float lightDistance) {
    IntersectionInfo info = findClosestIntersection(scene, shadowRayOrigin, shadowRayDir, true);

    if (info.hit) {
        if (info.t > 0.0f && info.t < lightDistance) {
            return true; // It is in shadow
        }
    }

    return false; // Not in shadow
}

Vec3 traceRay(const Scene& scene, const Vec3& rayOrigin, const Vec3& rayDir, int depth) {
    if (depth > scene.maxRecursionDepth) {
        return scene.backgroundColor; // limit reached
    }

    IntersectionInfo info = findClosestIntersection(scene, rayOrigin, rayDir);

    if (!info.hit || info.materialId == -1) {
        return scene.backgroundColor;
    }

    int hitMaterialId = info.materialId;
    Vec3 hitNormal = info.hitNormal;
    Vec3 hitPoint = info.hitPoint;

    Material material;
    for (const auto &m : scene.materials) {
        if (m.id == hitMaterialId) { material = m; break; }
    }

    // Ambient
    Vec3 color = material.ambientReflectance.multiply(scene.ambientLight);

    for (const auto &light : scene.lights) {
        Vec3 lightDir = (light.position.subtract(hitPoint)).normalize();
        float lightDist = (light.position.subtract(hitPoint)).length();
        Vec3 shadowOrigin = hitPoint.add(hitNormal.scale(scene.shadowRayEpsilon));

        if (isInShadow(scene, shadowOrigin, lightDir, lightDist)) continue;

        // Diffuse
        Vec3 effIntensity = light.intensity.scale(1.0f / (lightDist * lightDist));
        float NdotL = max(0.0f, hitNormal.dot(lightDir));
        Vec3 diffuse = material.diffuseReflectance.multiply(effIntensity).scale(NdotL);

        // Specular
        Vec3 viewDir = rayOrigin.subtract(hitPoint).normalize();
        Vec3 halfVec = (lightDir.add(viewDir)).normalize();
        float NdotH = max(0.0f, hitNormal.dot(halfVec));
        Vec3 specular = material.specularReflectance.multiply(effIntensity).scale(pow(NdotH, material.phongExponent));

        color = color.add(diffuse).add(specular);
    }

    // Mirror Reflection and Refraction
    if (material.mirrorReflectance.x > 0 || material.mirrorReflectance.y > 0 || material.mirrorReflectance.z > 0 || material.type == "dielectric" || material.type == "conductor") {

        // Reflection Ray
        Vec3 reflectDir = reflect(rayDir, hitNormal).normalize();
        Vec3 reflectOrigin = hitPoint.add(hitNormal.scale(scene.shadowRayEpsilon));
        Vec3 reflectedColor = traceRay(scene, reflectOrigin, reflectDir, depth + 1);

        // Refraction Ray (for dielectric only)
        Vec3 refractedColor;
        float R = 1.0f; // Reflection coefficient

        if (material.type == "dielectric") { // Glass, Water, etc.
            float ni = 1.0f; // Index of Refraction of incident medium (air)
            float nt = material.refractionIndex; // Index of Refraction of transmitted medium (material)

            Vec3 N = hitNormal;
            float cosI = -rayDir.dot(N);

            // Is ray inside the object or outside?
            if (cosI < 0) {
                // From inside to outside
                swap(ni, nt);
                N = N.scale(-1.0f);
                cosI = -rayDir.dot(N); // Recalculate cosI because N changed
            }

            float eta = ni / nt; // Relative Index of Refraction
            R = fresnelDielectric(cosI, eta); // Fresnel reflection coefficient

            // Refraction and Reflection blending
            if (R < 1.0f) { // If there is no total internal reflection
                Vec3 refractDir;
                if (refract(rayDir, N, eta, refractDir)) {
                    Vec3 refractOrigin = hitPoint.subtract(N.scale(scene.shadowRayEpsilon)); // Move inside the surface
                    refractedColor = traceRay(scene, refractOrigin, refractDir, depth + 1);

                    // Beer's Law for absorption inside the medium
                    Vec3 absorption = material.absorptionCoefficient;
                    float distance = info.t; // Distance traveled inside the medium
                    Vec3 beer = Vec3(exp(-absorption.x * distance),
                                     exp(-absorption.y * distance),
                                     exp(-absorption.z * distance));
                    refractedColor = refractedColor.multiply(beer);

                    float T = 1.0f - R; // Transmittance
                    color = color.add(reflectedColor.multiply(material.mirrorReflectance).scale(R)); // Reflectance
                    color = color.add(refractedColor.scale(T)); // Transmittance
                } else {
                    // Total Internal Reflection case
                    color = color.add(reflectedColor.multiply(material.mirrorReflectance).scale(1.0f));
                }
            } else {
                // Total Internal Reflection case
                color = color.add(reflectedColor.multiply(material.mirrorReflectance));
            }

        } else if (material.type == "conductor") {
            // Fresnel reflection with absorption index
            Vec3 I = rayDir.normalize();
            Vec3 N = hitNormal.normalize();
            Vec3 fresnel = fresnelConductor(I, N, material.refractionIndex, material.absorptionIndex);

            // Modulate reflected color with Fresnel term
            Vec3 reflectedColorModulated = reflectedColor.multiply(material.mirrorReflectance);
            color = color.add(reflectedColorModulated.multiply(fresnel));

        } else if (material.mirrorReflectance.x > 0 || material.mirrorReflectance.y > 0 || material.mirrorReflectance.z > 0) {
            // Pure mirror reflection
            color = color.add(reflectedColor.multiply(material.mirrorReflectance));
        }
    }

    return color;
}

// Calculates the reflection vector
static Vec3 reflect(const Vec3& I, const Vec3& N) {
    // R = I - 2 * (I . N) * N
    return I.subtract(N.scale(2.0f * I.dot(N)));
}

// Calculates the refraction vector (Snell's Law). Checks for total internal reflection.
static bool refract(const Vec3& I, const Vec3& N, float eta, Vec3& T) {
    float NdotI = N.dot(I);
    float k = 1.0f - eta * eta * (1.0f - NdotI * NdotI);

    if (k < 0.0f) {
        return false; // Total Internal Reflection
    } else {
        // T = eta * I + (eta * NdotI - sqrt(k)) * N
        T = I.scale(eta).subtract(N.scale(eta * NdotI + sqrt(k)));
        T = T.normalize();
        return true;
    }
}

// Calculates the Fresnel reflection coefficient for dielectrics
static float fresnelDielectric(float cosI, float eta) {
    float sinI_sq = 1.0f - cosI * cosI;
    float sinT_sq = eta * eta * sinI_sq;

    if (sinT_sq >= 1.0f) {
        return 1.0f; // Total Internal Reflection
    }

    float cosT = sqrt(1.0f - sinT_sq);
    float Rs = (eta * cosI - cosT) / (eta * cosI + cosT);
    float Rp = (cosI - eta * cosT) / (cosI + eta * cosT);

    // Average
    return (Rs * Rs + Rp * Rp) / 2.0f;
}

// Fresnel reflection for conductors
static Vec3 fresnelConductor(const Vec3& I, const Vec3& N, float eta, float k) {
    float cosi = fabs(I.dot(N));
    float cosi2 = cosi * cosi;

    // (η^2 + k^2)
    float eta2 = eta * eta;
    float k2 = k * k;
    float twoEtaCosi = 2.0f * eta * cosi;

    float Rs_num = (eta2 + k2) - twoEtaCosi + cosi2;
    float Rs_den = (eta2 + k2) + twoEtaCosi + cosi2;
    float Rs = Rs_num / Rs_den;

    float Rp_num = (eta2 + k2) * cosi2 - twoEtaCosi + 1.0f;
    float Rp_den = (eta2 + k2) * cosi2 + twoEtaCosi + 1.0f;
    float Rp = Rp_num / Rp_den;

    float R = (Rs + Rp) * 0.5f;

    return Vec3(R, R, R);
}

static void barycentrics(const Vec3& P, const Vec3& v0, const Vec3& v1, const Vec3& v2,
                         float &w0, float &w1, float &w2)
{
    Vec3 N = (v1.subtract(v0)).cross(v2.subtract(v0));
    float invNN = 1.0f / std::max(1e-20f, N.dot(N));
    w0 = N.dot((v1.subtract(P)).cross(v2.subtract(P))) * invNN;
    w1 = N.dot((v2.subtract(P)).cross(v0.subtract(P))) * invNN;
    w2 = 1.0f - w0 - w1;
}
