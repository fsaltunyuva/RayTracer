#include <fstream>
#include <iostream>
#include <chrono>
#include "../json/json.hpp"
#include <omp.h> // OpenMP for parallel processing
#include <random>

#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "../stb/stb_image_write.h"

#include "../include/Scene.h"
#include "../include/Intersection.h"
#include "../include/Intersector.h"

using json = nlohmann::json;
using namespace std;

static bool isInShadow(const Scene& scene, const Vec3& shadowRayOrigin, const Vec3& shadowRayDir, float lightDistance, Intersector& intersector, float rayTime);
Vec3 traceRay(const Scene& scene, const Vec3& rayOrigin, const Vec3& rayDir, int depth, Intersector& intersector, float rayTime);

// Reflect and Refract Functions
static Vec3 reflect(const Vec3& I, const Vec3& N);
static bool refract(const Vec3& I, const Vec3& N, float eta, Vec3& T);
static float fresnelDielectric(float cosI, float eta);
static Vec3 fresnelConductor(const Vec3& I, const Vec3& N, float eta, float k);
static Vec3 perturbDirection(const Vec3& idealDir, float roughness);
static void buildAreaLightBasis(const Vec3& nL, Vec3& u, Vec3& v);

// Get Transformation Matrix from transformation order and list
glm::mat4 getTransformationMatrix(const std::vector<std::string>& order,
                                  const std::vector<Transformation>& transformations)
{
    glm::mat4 M(1.0f);
    for (const string& token : order) {
        char type = token[0]; // 's', 't', 'r', 'c'
        int id = std::stoi(token.substr(1)); // Extract ID
        for (const auto& t : transformations) {
            if (t.id == id) {
                if ((type == 's' && t.type == TransformationType::Scaling) ||
                    (type == 't' && t.type == TransformationType::Translation) ||
                    (type == 'r' && t.type == TransformationType::Rotation) ||
                    (type == 'c' && t.type == TransformationType::Composite))
                {
                    M = t.getMatrix() * M;
                }
            }
        }
    }
    return M;
}

// Find the base mesh for a given baseId (handles nested instances like marching_dragons.json)
const Mesh* findBaseMesh(const Scene& scene, int baseId) {
    // First look in Meshes
    for (const auto& m : scene.objects.meshes) {
        if (m.id == baseId)
            return &m;
    }
    // Then look in MeshInstances
    for (const auto& inst : scene.objects.meshInstances) {
        if (inst.id == baseId) {
            // Recursive call to resolve nested instances
            return findBaseMesh(scene, inst.baseMeshId);
        }
    }
    return nullptr; // Could not find
}

// Get Model Matrix for MeshInstance (handles base mesh or instance)
glm::mat4 getInstanceModelMatrix(const Scene& scene, const MeshInstance& instance) {
    glm::mat4 instM = getTransformationMatrix(instance.transformationOrder, scene.transformations);

    if (instance.resetTransform)
        return instM; // Use instance's own transform

    for (const auto& mesh : scene.objects.meshes) { // First look in Meshes
        if (mesh.id == instance.baseMeshId)
            return instM * mesh.modelMatrix;
    }
    for (const auto& inst : scene.objects.meshInstances) { // Then look in MeshInstances
        if (inst.id == instance.baseMeshId)
            return instM * getInstanceModelMatrix(scene, inst);
    }

    return instM;
}

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

    // Precompute model matrices for all objects
    for (auto& s : scene.objects.spheres)
        s.modelMatrix = getTransformationMatrix(s.transformationOrder, scene.transformations);

    for (auto& m : scene.objects.meshes)
        m.modelMatrix = getTransformationMatrix(m.transformationOrder, scene.transformations);

    for (auto& t : scene.objects.triangles)
        t.modelMatrix = getTransformationMatrix(t.transformationOrder, scene.transformations);

    for (auto& p : scene.objects.planes)
        p.modelMatrix = getTransformationMatrix(p.transformationOrder, scene.transformations);
    
    for (auto& mi : scene.objects.meshInstances)
        mi.modelMatrix = getInstanceModelMatrix(scene, mi);

    for (auto& cam : scene.cameras)
        cam.modelMatrix = getTransformationMatrix(cam.transformationOrder, scene.transformations);

    for (auto& cam : scene.cameras) {
        // Apply transformation to position vector
        glm::vec4 pos = cam.modelMatrix * glm::vec4(cam.position.x, cam.position.y, cam.position.z, 1.0f);
        cam.position = Vec3(pos.x, pos.y, pos.z);

        // Apply transformation to gaze and up vectors
        glm::vec4 gazeDir = cam.modelMatrix * glm::vec4(cam.gaze.x, cam.gaze.y, cam.gaze.z, 0.0f);
        glm::vec4 upDir = cam.modelMatrix * glm::vec4(cam.up.x, cam.up.y, cam.up.z, 0.0f);

        cam.gaze = Vec3(gazeDir.x, gazeDir.y, gazeDir.z).normalize();
        cam.up = Vec3(upDir.x, upDir.y, upDir.z).normalize();
    }

    for (auto& light : scene.pointLights) {
        light.modelMatrix = getTransformationMatrix(light.transformationOrder, scene.transformations);

        // Apply transformation to position vector
        glm::vec4 transformed = light.modelMatrix * glm::vec4(light.position.x, light.position.y, light.position.z, 1.0f);
        light.position = Vec3(transformed.x, transformed.y, transformed.z);
    }

    // Resolve base meshes for all mesh instances (for marching_dragons.json case)
    for (auto& instance : scene.objects.meshInstances) {
        instance.resolvedBaseMesh = findBaseMesh(scene, instance.baseMeshId);
    }

#pragma region BVH
    // Create BVH for all meshes
    for (auto& mesh : scene.objects.meshes) {
        mesh.bvh = new BVH();
        mesh.bvh->build(mesh.data, scene.vertexData, mesh.modelMatrix);

        // TODO: cornellbox_boxes_dynamic.json black render fix
        // if (mesh.data.size() > 300) { // Only build BVH for large meshes
        //     mesh.bvh = new BVH();
        //     mesh.bvh->build(mesh.data, scene.vertexData, mesh.modelMatrix);
        // } else {
        //     mesh.bvh = nullptr;  // Go brute force for small meshes
        // }
    }

    // Create BVH for all mesh instances
    for (auto& instance : scene.objects.meshInstances) {
        const Mesh* realBaseMesh = findBaseMesh(scene, instance.baseMeshId);

        instance.resolvedBaseMesh = realBaseMesh; // Store resolved base mesh

        instance.bvh = new BVH();
        // Using realBaseMesh->data but sending instance.modelMatrix for correct transformation
        instance.bvh->build(realBaseMesh->data, scene.vertexData, instance.modelMatrix);
    }
#pragma endregion

    // Precompute per-vertex normals for smooth shaded meshes
    for (auto &mesh : scene.objects.meshes) {
        if (mesh.shadingMode != ShadingMode::Smooth) continue;

        // Initialize normals to zero
        mesh.perVertexNormal.assign(scene.vertexData.size(), Vec3(0,0,0));

        for (size_t i = 0; i + 2 < mesh.data.size(); i += 3) {
            int i0 = mesh.data[i] - 1;
            int i1 = mesh.data[i+1] - 1;
            int i2 = mesh.data[i+2] - 1;
            const Vec3 &v0 = scene.vertexData[i0];
            const Vec3 &v1 = scene.vertexData[i1];
            const Vec3 &v2 = scene.vertexData[i2];
            Vec3 faceN = v1.subtract(v0).cross(v2.subtract(v0)); // Unnormalized face normal by cross product
            // Accumulate face normal to each vertex normal
            mesh.perVertexNormal[i0] = mesh.perVertexNormal[i0].add(faceN);
            mesh.perVertexNormal[i1] = mesh.perVertexNormal[i1].add(faceN);
            mesh.perVertexNormal[i2] = mesh.perVertexNormal[i2].add(faceN);
        }
        // Normalize and transform normals to world space
        glm::mat3 normalMatrix = glm::transpose(glm::inverse(glm::mat3(mesh.modelMatrix)));

        // Normalize all vertex normals
        for (size_t vi = 0; vi < mesh.perVertexNormal.size(); ++vi) {
            if (mesh.perVertexNormal[vi].length_squared() > 0.0f) {
                glm::vec3 nLocal = glm::vec3(mesh.perVertexNormal[vi].x,
                                             mesh.perVertexNormal[vi].y,
                                             mesh.perVertexNormal[vi].z);

                //glm::vec3 nWorld = glm::normalize(normalMatrix * nLocal);
                //mesh.perVertexNormal[vi] = Vec3(nWorld);
                mesh.perVertexNormal[vi] = mesh.perVertexNormal[vi].normalize();

            }
        }
    }

    // Compute per-vertex normals for mesh instances (apply transforms properly)
    for (auto &instance : scene.objects.meshInstances) {
        // Find base mesh
        const Mesh* base = instance.resolvedBaseMesh;

        if (!base || base->shadingMode != ShadingMode::Smooth) continue;

        instance.perVertexNormal.resize(base->perVertexNormal.size());

        // Apply instance's transform to base mesh normals
        glm::mat3 normalMatrix = glm::transpose(glm::inverse(glm::mat3(instance.modelMatrix)));

        for (size_t i = 0; i < base->perVertexNormal.size(); ++i) {
            glm::vec3 nLocal(base->perVertexNormal[i].x,
                             base->perVertexNormal[i].y,
                             base->perVertexNormal[i].z);

            glm::vec3 nWorld = glm::normalize(normalMatrix * nLocal);
            instance.perVertexNormal[i] = Vec3(nWorld.x, nWorld.y, nWorld.z);
        }
    }

    for (size_t ci = 0; ci < scene.cameras.size(); ++ci) { // For each camera
        const auto &cam = scene.cameras[ci];

        int width = cam.imageResolution.x;
        int height = cam.imageResolution.y;

        float left, right, bottom, top;
        float nearDist = cam.nearDistance;

        if (cam.hasFovY) { // lookAt camera
            float fovY = glm::radians(cam.fovY);
            float aspect = (float)width / (float)height;
            top = nearDist * tan(fovY / 2.0f);
            bottom = -top;
            right = top * aspect;
            left = -right;
        } else { // using NearPlane
            left = cam.nearPlane[0];
            right = cam.nearPlane[1];
            bottom = cam.nearPlane[2];
            top = cam.nearPlane[3];
        }

        Vec3 camPos = cam.position;
        Vec3 gaze = cam.gaze.normalize();
        Vec3 up = cam.up.normalize();

        Vec3 w = gaze.scale(-1.0f); // -gaze
        Vec3 u_vec = (up.cross(w)).normalize();
        Vec3 v_vec = w.cross(u_vec);

        vector<unsigned char> pixels(width * height * 3);

        auto start_cam = chrono::high_resolution_clock::now();

#pragma omp parallel
        {
            Intersector threadIntersector(scene); // Thread-local intersector for parallel safety

#pragma omp for schedule(dynamic)
            for (int y = 0; y < height; ++y) {
                for (int x = 0; x < width; ++x) {
                    int N = cam.numSamples;
                    int S = (int) sqrt((float) N); // S x S grid
                    Vec3 pixelColor(0, 0, 0);

                    std::mt19937 rng(1234 + y * width + x * 137); // Seed per pixel for reproducibility (same results for each run)
                    std::uniform_real_distribution<float> dist(0.0f, 1.0f); // Jitter distribution [0,1)

                    vector<pair<float, float> > jitterSamples; // pair of (jx, jy)
                    jitterSamples.reserve(N); // Preallocate

                    // Generate stratified jitter samples
                    for (int sy = 0; sy < S; sy++) {
                        for (int sx = 0; sx < S; sx++) {
                            float jx = (sx + dist(rng)) / S; // stratified + jitter
                            float jy = (sy + dist(rng)) / S; // stratified + jitter

                            jitterSamples.push_back({jx, jy});
                        }
                    }

                    // Shuffle the jitter samples to avoid correlation
                    shuffle(jitterSamples.begin(), jitterSamples.end(), rng);

                    for (int k = 0; k < N; k++) { // Shoot N rays per pixel
                        float jx = jitterSamples[k].first;
                        float jy = jitterSamples[k].second;

                        // Pixel normalized coordinate
                        float u = (x + jx) / width;
                        float v = (y + jy) / height;

                        // Image plane coordinates
                        float su = left + (right - left) * u;
                        float sv = bottom + (top - bottom) * v;

                        // Point on image plane
                        Vec3 P = camPos
                                .subtract(w.scale(nearDist))
                                .add(u_vec.scale(su))
                                .add(v_vec.scale(sv));

                        Vec3 rayOrigin;
                        Vec3 rayDir;

                        bool useDoF = (cam.apertureSize > 0.0f && cam.focusDistance > 0.0f);

                        if (useDoF) {
                            Vec3 dirPinhole = P.subtract(camPos); // dir = s - e
                            float t_f = cam.focusDistance / nearDist;
                            Vec3 p_focal = camPos.add(dirPinhole.scale(t_f));

                            // Sample point on lens (square aperture in u-v plane)
                            float e1 = dist(rng); // E1
                            float e2 = dist(rng); // E2

                            // e + apertureSize * (u(E1 - 0.5) + v(E2 - 0.5))
                            float lensX = (e1 - 0.5f) * cam.apertureSize;
                            float lensY = (e2 - 0.5f) * cam.apertureSize;

                            // Aperture sample point
                            Vec3 lensPoint = camPos
                                    .add(u_vec.scale(lensX))
                                    .add(v_vec.scale(lensY));

                            rayOrigin = lensPoint;
                            rayDir = (p_focal.subtract(lensPoint)).normalize();
                        } else {
                            // Old pinhole behavior
                            rayOrigin = camPos;
                            rayDir = (P.subtract(camPos)).normalize();
                        }

                        // For motion blur (0.0 to 1.0)
                        float rayTime = dist(rng); // Constant for this ray

                        // Shading
                        Vec3 sampleColor = traceRay(scene, rayOrigin, rayDir, 0, threadIntersector, rayTime);

                        pixelColor = pixelColor.add(sampleColor);
                    }

                    // Average over samples
                    pixelColor = pixelColor.scale(1.0f / (float) N);

                    // Flip y coordinate for image origin at a bottom-left (In PNG, origin is at top-left)
                    int flippedY = height - 1 - y;
                    int index = (flippedY * width + x) * 3;

                    auto clamp255 = [](float v) {
                        return v < 0.0f ? 0.0f : (v > 255.0f ? 255.0f : v);
                    };

                    pixels[index + 0] = static_cast<unsigned char>(clamp255(pixelColor.x));
                    pixels[index + 1] = static_cast<unsigned char>(clamp255(pixelColor.y));
                    pixels[index + 2] = static_cast<unsigned char>(clamp255(pixelColor.z));
                }
            }
        } // Parallel region end

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

static bool isInShadow(const Scene& scene, const Vec3& shadowRayOrigin, const Vec3& shadowRayDir, float lightDistance, Intersector& intersector, float rayTime) {
    IntersectionInfo info = intersector.findClosestIntersection(shadowRayOrigin, shadowRayDir, true, rayTime);

    if (info.hit) {
        if (info.t > scene.shadowRayEpsilon && info.t < lightDistance - scene.shadowRayEpsilon) {
            return true; // It is in shadow
        }
    }

    return false; // Not in shadow
}

Vec3 traceRay(const Scene& scene, const Vec3& rayOrigin, const Vec3& rayDir, int depth, Intersector& intersector, float rayTime) {
    if (depth > scene.maxRecursionDepth) {
        return scene.backgroundColor; // limit reached
    }

    Vec3 rayDirN = rayDir.normalize();

    // To sample area lights (for each thread)
    static thread_local std::mt19937 rng(2025); // Fixed seed per thread
    static thread_local std::uniform_real_distribution<float> dist01(0.0f, 1.0f); // [0,1)

    IntersectionInfo info = intersector.findClosestIntersection(rayOrigin, rayDirN, false, rayTime);

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

    // Point Lights Shadow and Shading
    for (const auto &light : scene.pointLights) {
        Vec3 lightDir = (light.position.subtract(hitPoint)).normalize();
        float lightDist = (light.position.subtract(hitPoint)).length();
        Vec3 shadowOrigin = hitPoint.add(hitNormal.scale(scene.shadowRayEpsilon));

        if (isInShadow(scene, shadowOrigin, lightDir, lightDist, intersector, rayTime)) continue;

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

    // Area Lights Shadow and Shading
    for (const auto &light : scene.areaLights) {
        // Build area light basis
        Vec3 uL, vL;
        buildAreaLightBasis(light.normal, uL, vL);

        int numSamples = max(1, scene.areaLightNumSamples);
        float A = light.size * light.size; // Area of square light

        Vec3 areaContribution(0, 0, 0);

        for (int s = 0; s < numSamples; ++s) {
            float xi1 = dist01(rng);
            float xi2 = dist01(rng);

            float half = light.size * 0.5f;
            // [-half, half] range
            float sx = (xi1 * 2.0f - 1.0f) * half;
            float sy = (xi2 * 2.0f - 1.0f) * half;

            // Sampled point
            Vec3 lightPoint = light.position
                            .add(uL.scale(sx))
                            .add(vL.scale(sy));

            Vec3 toLight = lightPoint.subtract(hitPoint);
            float d2 = toLight.length_squared();

            if (d2 <= 0.0f) continue;
            float d = sqrt(d2);

            // Direction from hit point to light sample
            Vec3 wi = toLight.scale(1.0f / d);

            // float cosLight = std::max(0.0f, light.normal.normalize().dot(wi.scale(-1.0f)));
            // float cosLight = std::max(0.0f, -light.normal.normalize().dot(wi.scale(-1.0f)));
            float cosLight = std::max(0.0f, std::fabs(light.normal.normalize().dot(wi.scale(-1.0f))));

            if (cosLight <= 0.0f) continue;

            // Shadow ray
            Vec3 shadowOrigin = hitPoint.add(hitNormal.scale(scene.shadowRayEpsilon));
            if (isInShadow(scene, shadowOrigin, wi, d, intersector, rayTime))
                continue;

            // (A / N) * (n_l dot w_i) / d^2
            float geom = (A * cosLight) / (d2 * (float) numSamples);

            Vec3 Li = light.radiance;
            Vec3 eff = Li.scale(geom);

            // Diffuse
            float NdotL = std::max(0.0f, hitNormal.dot(wi));
            Vec3 diffuse = material.diffuseReflectance.multiply(eff).scale(NdotL);

            // Specular (Phong)
            Vec3 viewDir = rayOrigin.subtract(hitPoint).normalize();
            Vec3 halfVec = wi.add(viewDir).normalize();
            float NdotH = std::max(0.0f, hitNormal.dot(halfVec));
            Vec3 specular = material.specularReflectance
                                .multiply(eff)
                                .scale(std::pow(NdotH, material.phongExponent));

            areaContribution = areaContribution.add(diffuse).add(specular);
        }

        color = color.add(areaContribution);
    }


    // Mirror Reflection and Refraction
    if (material.mirrorReflectance.x > 0 || material.mirrorReflectance.y > 0 || material.mirrorReflectance.z > 0 || material.type == "dielectric" || material.type == "conductor") {

        // Reflection Ray
        Vec3 idealReflectDir = reflect(rayDir, hitNormal).normalize();

        // Perturb reflection direction based on roughness
        Vec3 reflectDir = perturbDirection(idealReflectDir, material.roughness);
        Vec3 reflectOrigin = hitPoint.add(hitNormal.scale(scene.shadowRayEpsilon));
        Vec3 reflectedColor = traceRay(scene, reflectOrigin, reflectDir, depth + 1, intersector, rayTime);

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
                Vec3 idealRefractDir;
                if (refract(rayDir, N, eta, idealRefractDir)) {
                    // Perturb refraction direction based on roughness
                    Vec3 refractDir = perturbDirection(idealRefractDir, material.roughness);

                    Vec3 refractOrigin = hitPoint.subtract(N.scale(scene.shadowRayEpsilon)); // Move inside the surface
                    refractedColor = traceRay(scene, refractOrigin, refractDir, depth + 1, intersector, rayTime);

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

// To get a random point inside a unit sphere
static Vec3 randomInUnitSphere() {
    static thread_local std::mt19937 rng(1337); // Thread-local RNG for parallel safety
    std::uniform_real_distribution<float> dist(-1.0f, 1.0f);

    while (true) { //
        Vec3 p(dist(rng), dist(rng), dist(rng));
        if (p.length_squared() > 0.0f && p.length_squared() <= 1.0f)
            return p;
    }
}

// Perturb the ideal direction based on roughness
static Vec3 perturbDirection(const Vec3& idealDir, float roughness) {
    if (roughness <= 0.0f)
        return idealDir;

    float r = std::min(roughness, 1.0f);

    Vec3 randVec = randomInUnitSphere();
    Vec3 perturbed = idealDir.add(randVec.scale(r));

    return perturbed.normalize();
}

// Build an orthonormal basis (u, v) for the area light's surface from its normal
static void buildAreaLightBasis(const Vec3& nL, Vec3& u, Vec3& v) {
    Vec3 n = nL.normalize();
    // Choose a helper vector that is not parallel to n
    Vec3 tmp = (fabs(n.x) > 0.9f) ? Vec3(0.0f, 1.0f, 0.0f) : Vec3(1.0f, 0.0f, 0.0f);
    u = tmp.cross(n).normalize(); // tangent
    v = n.cross(u).normalize(); // bitangent
}