#include <fstream>
#include <iostream>
#include <chrono>
#include "../json/json.hpp"
#include <omp.h> // OpenMP for parallel processing
#include <random>

#include "../include/Scene.h"
#include "../include/Intersection.h"
#include "../include/Intersector.h"
#include "../include/PerlinNoise.h"
#include "../include/ImageIO.h"
#include "../include/Tonemapper.h"

using json = nlohmann::json;
using namespace std;

static bool isInShadow(const Scene& scene, const Vec3& shadowRayOrigin, const Vec3& shadowRayDir, float lightDistance, Intersector& intersector, float rayTime);
Vec3 traceRay(const Scene& scene, const Vec3& rayOrigin, const Vec3& rayDir, int depth, Intersector& intersector, float rayTime, const Vec2& bgUV, std::mt19937& rng);
Vec3 tracePath(const Scene& scene, const Vec3& rayOrigin, const Vec3& rayDir, const Camera& cam, Intersector& intersector, float rayTime, const Vec2& bgUV, std::mt19937& rng);


// Reflect and Refract Functions
static Vec3 reflect(const Vec3& I, const Vec3& N);
static bool refract(const Vec3& I, const Vec3& N, float eta, Vec3& T);
static float fresnelDielectric(float cosI, float eta);
static Vec3 fresnelConductor(const Vec3& I, const Vec3& N, float eta, float k);
static Vec3 perturbDirection(const Vec3& idealDir, float roughness, std::mt19937& rng);
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

// Find ImageTextureMap by ID
static const ImageTextureMap* findImageTexMap(const Scene& scene, int texMapId) {
    for (const auto& t : scene.textures.imageTextures) {
        if (t.id == texMapId) return &t;
    }
    return nullptr;
}

// From the homework PDF
static Vec3 sampleCheckerboard(const CheckerTextureMap& tm, const Vec3& pos)
{
    bool x = ((int)std::floor((pos.x + tm.offset) * tm.scale)) % 2;
    bool y = ((int)std::floor((pos.y + tm.offset) * tm.scale)) % 2;
    bool z = ((int)std::floor((pos.z + tm.offset) * tm.scale)) % 2;

    bool xorXY = (x != y);
    if (xorXY != z) return tm.blackColor;
    else return tm.whiteColor;
}

// Find CheckerTextureMap by ID
static const CheckerTextureMap* findCheckerTexMap(const Scene& scene, int texMapId) {
    for (const auto& t : scene.textures.checkerTextures)
        if (t.id == texMapId) return &t;
    return nullptr;
}

// Find PerlinTextureMap by ID
static const PerlinTextureMap* findPerlinTexMap(const Scene& scene, int texMapId) {
    for (const auto& t : scene.textures.perlinTextures)
        if (t.id == texMapId) return &t;
    return nullptr;
}

// Perlin Texture Value
static float perlinTextureValue(const PerlinTextureMap& tm, const Vec3& posLocalOrWorld)
{
    Vec3 p = posLocalOrWorld.scale(tm.noiseScale);

    // K = 1 if numOctaves is not defined
    int K = (tm.numOctaves > 0) ? tm.numOctaves : 1;

    float s = 0.0f;
    float amp = 1.0f;
    float freq = 1.0f;
    float ampSum = 0.0f;

    static PerlinNoise gPerlin;

    for (int k = 0; k < K; ++k) {
        Vec3 q = p.scale(freq);
        float n = gPerlin.noise(q.x, q.y, q.z); // [-1, 1]


        // As stated in the homework PDF
        if (tm.conversion == NoiseConversion::AbsVal) {
            s += amp * std::fabs(n);
        } else {
            s += amp * n;
        }

        ampSum += amp;
        freq *= 2.0f;
        amp *= 0.5f;
    }

    float out01 = 0.0f;

    if (tm.conversion == NoiseConversion::AbsVal) {
        out01 = s / ampSum;
    } else {
        float norm = s / ampSum; // [-1, 1]
        out01 = norm * 0.5f + 0.5f; // [0, 1]
    }

    out01 = std::max(0.0f, std::min(1.0f, out01));
    return out01;
}

// Sample Perlin Texture
static Vec3 samplePerlinTex(const PerlinTextureMap& tm, const Vec3& pos)
{
    float g = perlinTextureValue(tm, pos); // [0,1]
    return Vec3(g, g, g);
}

static void buildONB(const Vec3& n, Vec3& t, Vec3& b) {
    Vec3 nn = n.normalize();
    Vec3 tmp = (fabs(nn.x) > 0.9f) ? Vec3(0,1,0) : Vec3(1,0,0);
    t = tmp.cross(nn).normalize();
    b = nn.cross(t).normalize();
}

static Vec3 sampleHemisphere(const Vec3& N, EnvSampler sampler, float u1, float u2, float& pdfOut) {
    Vec3 t, b;
    buildONB(N, t, b);

    float x, y, z;

    if (sampler == EnvSampler::Uniform) {
        // Uniform hemisphere: pdf = 1/(2*pi)
        float zLocal = u1;
        float r = sqrt(std::max(0.0f, 1.0f - zLocal*zLocal));
        float phi = 2.0f * (float) M_PI * u2;
        x = r * cos(phi);
        y = r * sin(phi);
        z = zLocal;

        pdfOut = 1.0f / (2.0f * (float) M_PI);
    } else {
        // Cosine-weighted pdf = cosTheta/pi
        float r = sqrt(u1);
        float phi = 2.0f * (float) M_PI * u2;
        x = r * cos(phi);
        y = r * sin(phi);
        z = sqrt(std::max(0.0f, 1.0f - u1));

        pdfOut = z / (float) M_PI;
    }

    Vec3 dirW = t.scale(x).add(b.scale(y)).add(N.normalize().scale(z)).normalize();
    return dirW;
}

static Vec2 dirToUV_LatLong(const Vec3& d) {
    Vec3 dn = d.normalize();
    float u = (1.0f + std::atan2(dn.x, -dn.z) / (float) M_PI) * 0.5f;
    float v = std::acos(std::max(-1.0f, std::min(1.0f, dn.y))) / (float) M_PI;

    u = u - std::floor(u);
    v = std::max(0.0f, std::min(1.0f, v));
    return Vec2(u, v);
}

static Vec2 dirToUV_Probe(const Vec3& d) {
    Vec3 dn = d.normalize();

    float denom = std::sqrt(dn.x * dn.x + dn.y * dn.y);
    if (denom < 1e-8f) return Vec2(0.5f, 0.5f);

    float r = (1.0f / (float) M_PI) * std::acos(std::max(-1.0f, std::min(1.0f, -dn.z))) / denom;
    float u = (r * dn.x + 1.0f) * 0.5f;
    float v = (-r * dn.y + 1.0f) * 0.5f;

    u = u - std::floor(u);
    v = std::max(0.0f, std::min(1.0f, v));

    return Vec2(u, v);
}

static Vec3 sampleEnv(const SphericalDirectionalLight& el, const Vec3& dirW) {
    if (!el.image)
        return Vec3(0,0,0);

    Vec2 uv = (el.type == EnvMapType::LatLong) ? dirToUV_LatLong(dirW) : dirToUV_Probe(dirW);

    return el.image->sampleBilinear(uv);
}

static float misWeight(float pdfA, float pdfB, Camera::MISHeuristic h) {
    if (pdfA <= 0.0f) return 0.0f;
    if (pdfB <= 0.0f) return 1.0f;

    if (h == Camera::MISHeuristic::Balance) {
        return pdfA / (pdfA + pdfB);
    }
    else if (h == Camera::MISHeuristic::Power) {
        float a2 = pdfA * pdfA;
        float b2 = pdfB * pdfB;
        return a2 / (a2 + b2);
    }
    else {
        // 01 heuristic
        return (pdfA > pdfB) ? 1.0f : 0.0f;
    }
}

static Vec3 sampleUniformSphere(float u1, float u2) {
    float z = 1.0f - 2.0f * u1;
    float r = sqrt(std::max(0.0f, 1.0f - z * z));
    float phi = 2.0f * (float) M_PI * u2;

    return Vec3(r * cos(phi), r * sin(phi), z);
}


struct LightSample {
    Vec3 Li; // Emitted radiance
    Vec3 wi; // Direction from x to light
    float dist; // Distance to sampled point (or large for env)
    float pdfW; // pdf in solid angle measure at x (p_omega)
    bool isDelta = false;
    bool valid = false;
};

// Sample ONE emitter uniformly among: AreaLight + LightSphere + LightMesh
static LightSample sampleOneEmitter(const Scene &scene, const Vec3 &x, const Vec3 &n, Intersector &intersector, float rayTime, std::mt19937 &rng) {
    std::uniform_real_distribution<float> dist01(0.0f, 1.0f);

    int nArea = (int) scene.areaLights.size();
    int nLSph = (int) scene.objects.lightSpheres.size();
    int nLMesh = (int) scene.objects.lightMeshes.size();
    int nTotal = nArea + nLSph + nLMesh;

    LightSample out;
    if (nTotal == 0) return out;

    int which = (int) std::floor(dist01(rng) * nTotal);
    if (which >= nTotal)
        which = nTotal - 1;

    float pPick = 1.0f / (float) nTotal;

    // Area Light
    if (which < nArea) {
        const auto &L = scene.areaLights[which];

        Vec3 uL, vL;
        buildAreaLightBasis(L.normal, uL, vL);

        float half = L.size * 0.5f;
        float sx = (dist01(rng) * 2.0f - 1.0f) * half;
        float sy = (dist01(rng) * 2.0f - 1.0f) * half;

        Vec3 pL = L.position.add(uL.scale(sx)).add(vL.scale(sy));

        Vec3 toL = pL.subtract(x);
        float d2 = toL.length_squared();
        if (d2 <= 0.0f)
            return out;

        float d = std::sqrt(d2);
        Vec3 wi = toL.scale(1.0f / d);

        float cosX = std::max(0.0f, n.dot(wi));
        if (cosX <= 0.0f)
            return out;

        float cosL = std::max(0.0f, std::fabs(L.normal.normalize().dot(wi.scale(-1.0f))));
        if (cosL <= 0.0f)
            return out;

        Vec3 shadowOrigin = x.add(n.scale(scene.shadowRayEpsilon));

        if (isInShadow(scene, shadowOrigin, wi, d - scene.shadowRayEpsilon, intersector, rayTime)) return out;

        float area = L.size * L.size;
        float pdfA = 1.0f / area;
        float pdfW = (d2) / (cosL * area);

        out.valid = true;
        out.wi = wi;
        out.dist = d;
        out.pdfW = pdfW * pPick;
        out.Li = L.radiance;

        return out;
    }

    which -= nArea;

    // Light Sphere
    if (which < nLSph) {
        const auto &S = scene.objects.lightSpheres[which];

        Vec3 cLocal = scene.vertexData[S.center - 1];
        Vec3 rndDir = sampleUniformSphere(dist01(rng), dist01(rng));
        Vec3 localSurfacePoint = cLocal.add(rndDir.scale(S.radius));
        glm::vec4 worldPosGLM = S.modelMatrix * glm::vec4(localSurfacePoint.x, localSurfacePoint.y, localSurfacePoint.z,
                                                          1.0f);
        Vec3 pL(worldPosGLM.x, worldPosGLM.y, worldPosGLM.z);

        glm::mat3 normalMatrix = glm::transpose(glm::inverse(glm::mat3(S.modelMatrix)));
        glm::vec3 worldNormalGLM = glm::normalize(normalMatrix * glm::vec3(rndDir.x, rndDir.y, rndDir.z));
        Vec3 nL(worldNormalGLM.x, worldNormalGLM.y, worldNormalGLM.z);

        Vec3 toL = pL.subtract(x);
        float d2 = toL.length_squared();
        if (d2 <= 0.0f)
            return out;

        float d = std::sqrt(d2);
        Vec3 wi = toL.scale(1.0f / d);

        float cosX = std::max(0.0f, n.dot(wi));
        if (cosX <= 0.0f)
            return out;

        float cosL = std::max(0.0f, nL.dot(wi.scale(-1.0f)));
        if (cosL <= 0.0f)
            return out;

        Vec3 shadowOrigin = x.add(n.scale(scene.shadowRayEpsilon));
        if (isInShadow(scene, shadowOrigin, wi, d - scene.shadowRayEpsilon, intersector, rayTime)) return out;

        glm::vec3 col0 = glm::vec3(S.modelMatrix[0]);
        glm::vec3 col1 = glm::vec3(S.modelMatrix[1]);
        glm::vec3 col2 = glm::vec3(S.modelMatrix[2]);

        float sx = glm::length(col0);
        float sy = glm::length(col1);
        float sz = glm::length(col2);

        float a = S.radius * sx;
        float b = S.radius * sy;
        float c = S.radius * sz;

        float p = 1.6075f;
        float term = (std::pow(a * b, p) + std::pow(a * c, p) + std::pow(b * c, p)) / 3.0f;
        float area = 4.0f * (float) M_PI * std::pow(term, 1.0f / p);

        float pdfA = 1.0f / area;
        float pdfW = (d2) / (cosL * area);

        out.valid = true;
        out.wi = wi;
        out.dist = d;
        out.pdfW = pdfW * pPick;
        out.Li = S.radiance;
        return out;
    }

    which -= nLSph;

    // Light Mesh
    const auto &M = scene.objects.lightMeshes[which];

    double totalArea = 0.0;
    int triCount = (int) M.data.size() / 3;
    for (int t = 0; t < triCount; ++t) {
        int i0 = M.data[3 * t + 0] - 1;
        int i1 = M.data[3 * t + 1] - 1;
        int i2 = M.data[3 * t + 2] - 1;

        Vec3 v0 = scene.vertexData[i0];
        Vec3 v1 = scene.vertexData[i1];
        Vec3 v2 = scene.vertexData[i2];

        glm::vec3 w0 = glm::vec3(M.modelMatrix * glm::vec4(v0.x, v0.y, v0.z, 1));
        glm::vec3 w1 = glm::vec3(M.modelMatrix * glm::vec4(v1.x, v1.y, v1.z, 1));
        glm::vec3 w2 = glm::vec3(M.modelMatrix * glm::vec4(v2.x, v2.y, v2.z, 1));

        Vec3 a(w0.x, w0.y, w0.z), b(w1.x, w1.y, w1.z), c(w2.x, w2.y, w2.z);
        double A = 0.5 * (b.subtract(a).cross(c.subtract(a))).length();
        totalArea += A;
    }

    if (totalArea <= 0.0)
        return out;

    double r = dist01(rng) * totalArea;
    int chosen = 0;
    double acc = 0.0;

    for (int t = 0; t < triCount; ++t) {
        int i0 = M.data[3 * t + 0] - 1;
        int i1 = M.data[3 * t + 1] - 1;
        int i2 = M.data[3 * t + 2] - 1;

        Vec3 v0 = scene.vertexData[i0];
        Vec3 v1 = scene.vertexData[i1];
        Vec3 v2 = scene.vertexData[i2];

        glm::vec3 w0 = glm::vec3(M.modelMatrix * glm::vec4(v0.x, v0.y, v0.z, 1));
        glm::vec3 w1 = glm::vec3(M.modelMatrix * glm::vec4(v1.x, v1.y, v1.z, 1));
        glm::vec3 w2 = glm::vec3(M.modelMatrix * glm::vec4(v2.x, v2.y, v2.z, 1));

        Vec3 a(w0.x, w0.y, w0.z), b(w1.x, w1.y, w1.z), c(w2.x, w2.y, w2.z);
        double A = 0.5 * (b.subtract(a).cross(c.subtract(a))).length();
        acc += A;
        if (acc >= r) {
            chosen = t;
            break;
        }
    }

    int i0 = M.data[3 * chosen + 0] - 1;
    int i1 = M.data[3 * chosen + 1] - 1;
    int i2 = M.data[3 * chosen + 2] - 1;

    Vec3 v0 = scene.vertexData[i0];
    Vec3 v1 = scene.vertexData[i1];
    Vec3 v2 = scene.vertexData[i2];

    glm::vec3 w0 = glm::vec3(M.modelMatrix * glm::vec4(v0.x, v0.y, v0.z, 1));
    glm::vec3 w1 = glm::vec3(M.modelMatrix * glm::vec4(v1.x, v1.y, v1.z, 1));
    glm::vec3 w2 = glm::vec3(M.modelMatrix * glm::vec4(v2.x, v2.y, v2.z, 1));

    Vec3 a(w0.x, w0.y, w0.z), b(w1.x, w1.y, w1.z), c(w2.x, w2.y, w2.z);

    float u1 = dist01(rng);
    float u2 = dist01(rng);
    float su1 = std::sqrt(u1);
    float b0 = 1.0f - su1;
    float b1 = su1 * (1.0f - u2);
    float b2 = su1 * u2;

    Vec3 pL = a.scale(b0).add(b.scale(b1)).add(c.scale(b2));

    Vec3 nTri = (b.subtract(a).cross(c.subtract(a))).normalize();

    // For cornellbox
    if (nTri.z > 0.0f) {
        nTri = nTri.scale(-1.0f);
    }

    Vec3 toL = pL.subtract(x);
    float d2 = toL.length_squared();
    if (d2 <= 0.0f) return out;
    float d = std::sqrt(d2);
    Vec3 wi = toL.scale(1.0f / d);

    float cosX = std::max(0.0f, n.dot(wi));
    if (cosX <= 0.0f) return out;

    float cosL = nTri.dot(wi.scale(-1.0f));

    if (cosL <= 0.0f) return out;

    Vec3 shadowOrigin = x.add(n.scale(scene.shadowRayEpsilon));
    if (isInShadow(scene, shadowOrigin, wi, d - scene.shadowRayEpsilon, intersector, rayTime)) return out;

    float area = (float) totalArea;
    float pdfA = 1.0f / area;
    float pdfW = (d2) / (cosL * area);

    out.valid = true;
    out.wi = wi;
    out.dist = d;
    out.pdfW = pdfW * pPick;
    out.Li = M.radiance;

    return out;
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

#pragma region Precomputations
    // Load all images into memory
    for (auto& img : scene.images) {
        if (!img.load()) {
            std::cerr << "Failed to load texture image: " << img.path << std::endl;
        }
    }

    // Resolve ImageTextureMap.image pointer
    auto findImageById = [&](int id) -> const Image* {
        for (const auto& img : scene.images) {
            if (img.id == id) return &img;
        }
        return nullptr;
    };

    // Set image pointers for all ImageTextureMaps
    for (auto& tex : scene.textures.imageTextures)
        tex.image = findImageById(tex.imageId);

    // Precompute model matrices for all objects
    for (auto& s : scene.objects.spheres)
        s.modelMatrix = getTransformationMatrix(s.transformationOrder, scene.transformations);

    for (auto& ls : scene.objects.lightSpheres) {
        ls.modelMatrix = getTransformationMatrix(ls.transformationOrder, scene.transformations);
    }

    for (auto& lm : scene.objects.lightMeshes) {
        lm.modelMatrix = getTransformationMatrix(lm.transformationOrder, scene.transformations);
    }

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

    for (auto& el : scene.envLights) {
        el.image = findImageById(el.imageId); // Resolve image pointer
    }
#pragma endregion

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

#pragma region Smooth Shading
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
        // glm::mat3 normalMatrix = glm::transpose(glm::inverse(glm::mat3(mesh.modelMatrix)));

        // Normalize all vertex normals
        for (size_t vi = 0; vi < mesh.perVertexNormal.size(); ++vi) {
            if (mesh.perVertexNormal[vi].length_squared() > 0.0f) {
                // glm::vec3 nLocal = glm::vec3(mesh.perVertexNormal[vi].x,
                //                              mesh.perVertexNormal[vi].y,
                //                              mesh.perVertexNormal[vi].z);

                //glm::vec3 nWorld = glm::normalize(normalMatrix * nLocal);
                //mesh.perVertexNormal[vi] = Vec3(nWorld);
                mesh.perVertexNormal[vi] = mesh.perVertexNormal[vi].normalize();

            }
        }
    }

    for (auto &mesh: scene.objects.lightMeshes) {
        if (mesh.shadingMode != ShadingMode::Smooth) continue;

        mesh.perVertexNormal.assign(scene.vertexData.size(), Vec3(0, 0, 0));

        for (size_t i = 0; i + 2 < mesh.data.size(); i += 3) {
            int i0 = mesh.data[i] - 1;
            int i1 = mesh.data[i + 1] - 1;
            int i2 = mesh.data[i + 2] - 1;

            const Vec3 &v0 = scene.vertexData[i0];
            const Vec3 &v1 = scene.vertexData[i1];
            const Vec3 &v2 = scene.vertexData[i2];

            Vec3 faceN = v1.subtract(v0).cross(v2.subtract(v0));
            mesh.perVertexNormal[i0] = mesh.perVertexNormal[i0].add(faceN);
            mesh.perVertexNormal[i1] = mesh.perVertexNormal[i1].add(faceN);
            mesh.perVertexNormal[i2] = mesh.perVertexNormal[i2].add(faceN);
        }

        for (auto &n: mesh.perVertexNormal) {
            if (n.length() > 0) n = n.normalize();
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
#pragma endregion

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

        // vector<unsigned char> pixels(width * height * 3);
        std::vector<Vec3> hdr(width * height);

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

                    if (N == 1) {
                        // No jitter: sample pixel center
                        jitterSamples.push_back({0.5f, 0.5f});
                    } else {
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
                    }

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
                            rayOrigin = lensPoint;
                            rayDir = (p_focal.subtract(lensPoint)).normalize();
                        } else {
                            // Old pinhole behavior
                            rayOrigin = camPos;
                            rayDir = (P.subtract(camPos)).normalize();
                        }

                        // For motion blur (0.0 to 1.0)
                        float rayTime = dist(rng); // Constant for this ray

                        // For background sampling
                        float u01 = (x + jx) / width;
                        float v01 = (y + jy) / height;

                        Vec2 bgUV(u01, v01);

                        // Shading
                        // Vec3 sampleColor = traceRay(scene, rayOrigin, rayDir, 0, threadIntersector, rayTime, bgUV);

                        Vec3 sampleColor;
                        if (cam.renderer == Camera::RendererType::PathTracing) {
                            sampleColor = tracePath(scene, rayOrigin, rayDir, cam, threadIntersector, rayTime, bgUV, rng);
                        } else {
                            sampleColor = traceRay(scene, rayOrigin, rayDir, 0, threadIntersector, rayTime, bgUV, rng);
                        }

                        pixelColor = pixelColor.add(sampleColor);
                    }

                    // Average over samples
                    pixelColor = pixelColor.scale(1.0f / (float) N);

                    // Flip y coordinate for image origin at a bottom-left (In PNG, origin is at top-left)
                    int flippedY = height - 1 - y;
                    // int index = (flippedY * width + x) * 3;

                    // auto clamp255 = [](float v) {
                    //     return v < 0.0f ? 0.0f : (v > 255.0f ? 255.0f : v);
                    // };

                    // pixels[index + 0] = static_cast<unsigned char>(clamp255(pixelColor.x));
                    // pixels[index + 1] = static_cast<unsigned char>(clamp255(pixelColor.y));
                    // pixels[index + 2] = static_cast<unsigned char>(clamp255(pixelColor.z));

                    hdr[flippedY * width + x] = pixelColor;
                }
            }
        } // Parallel region end

        // If no tonemaps, still save EXR
        if (ImageIO::endsWith(cam.imageName, ".exr")) {
            ImageIO::writeEXR(cam.imageName, width, height, hdr);
        }

        for (const auto& tm : cam.tonemaps) {
            auto png = Tonemapper::toPNG(width, height, hdr, tm);
            std::string outName = ImageIO::replaceExrWithExtension(cam.imageName, tm.extension);
            ImageIO::writePNG(outName, width, height, png);
        }

        if (cam.tonemaps.empty() && ImageIO::endsWith(cam.imageName, ".png")) {
            std::vector<unsigned char> ldr(width * height * 3);

            auto clamp255 = [](float v) -> unsigned char {
                if (v < 0.0f) v = 0.0f;
                if (v > 255.0f) v = 255.0f;
                return (unsigned char) std::lround(v);
            };

            for (int i = 0; i < width * height; ++i) {
                ldr[i * 3 + 0] = clamp255(hdr[i].x);
                ldr[i * 3 + 1] = clamp255(hdr[i].y);
                ldr[i * 3 + 2] = clamp255(hdr[i].z);
            }

            ImageIO::writePNG(cam.imageName, width, height, ldr);
        }


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
        //if (info.isEmissive && std::abs(info.t - lightDistance) < scene.shadowRayEpsilon) {
        if (info.isEmissive) {
            return false;
        }

        if (info.t > scene.shadowRayEpsilon && info.t < lightDistance - scene.shadowRayEpsilon) {
            return true; // It is in shadow
        }
    }

    return false; // Not in shadow
}

Vec3 traceRay(const Scene& scene, const Vec3& rayOrigin, const Vec3& rayDir, int depth, Intersector& intersector, float rayTime, const Vec2& bgUV, std::mt19937& rng) {
    if (depth > scene.maxRecursionDepth) {
        return scene.backgroundColor; // limit reached
    }

    Vec3 rayDirN = rayDir.normalize();

    // To sample area lights (for each thread)
    std::uniform_real_distribution<float> dist01(0.0f, 1.0f); // [0,1)

    IntersectionInfo info = intersector.findClosestIntersection(rayOrigin, rayDirN, false, rayTime);

    if (!info.hit || info.materialId == -1) {

        // Environment Background
        if (!scene.envLights.empty()) {
            const auto& el = scene.envLights[0];
            if (el.image) {
                Vec3 d = rayDirN.normalize();

                Vec2 uv = (el.type == EnvMapType::LatLong) ? dirToUV_LatLong(d) : dirToUV_Probe(d);

                Vec3 L = el.image->sampleBilinear(uv); // EXR -> HDR float (linear)
                return L; // No scaling here, already in raw HDR
            }
        }

        // Background Texture
        if (scene.backgroundTextureMapId != -1) {
            const ImageTextureMap* tm = findImageTexMap(scene, scene.backgroundTextureMapId);
            if (tm && tm->image) {

                float u = std::max(0.0f, std::min(1.0f, bgUV.u));
                float v = std::max(0.0f, std::min(1.0f, bgUV.v));

                Vec3 c01 = tm->image->sampleNearest(Vec2(u, v)); // [0,1]
                return c01.scale(255.0f);
            }
        }

        return scene.backgroundColor;
    }

    if (info.isEmissive) {
        return info.emission;
    }

    int hitMaterialId = info.materialId;
    Vec3 hitNormal = info.hitNormal;
    Vec3 hitPoint = info.hitPoint;

    Material material;
    for (const auto &m : scene.materials) {
        if (m.id == hitMaterialId) { material = m; break; }
    }

    Vec3 kd = material.diffuseReflectance;
    Vec3 ks = material.specularReflectance;
    Vec3 ka = material.ambientReflectance;

    bool doReplaceAll = false;
    Vec3 replaceAllColor(0,0,0);

    // Apply image textures (replace_kd / replace_ks) if UV exists
    if (info.hasUV && info.textureMapIds) {
        const auto& texIds = *info.textureMapIds;
        for (int texMapId : texIds) {
            const ImageTextureMap* tm = findImageTexMap(scene, texMapId);
            if (!tm || !tm->image) continue;

            Vec3 texColor;
            if (tm->interpolation == InterpolationMode::Bilinear) {
                texColor = tm->image->sampleBilinear(info.hitUV);
            } else {
                texColor = tm->image->sampleNearest(info.hitUV);
            }

            // Only degamma for LDR texture
            // No degamma for normal maps or HDR textures
            bool affectsColor =
                (tm->decalMode == DecalMode::ReplaceKd) ||
                (tm->decalMode == DecalMode::BlendKd)   ||
                (tm->decalMode == DecalMode::ReplaceKs) ||
                (tm->decalMode == DecalMode::ReplaceAll);

            if (tm->degamma && affectsColor && tm->image && !tm->image->isHDR) {
                texColor = Vec3(std::pow(std::max(0.0f, std::min(1.0f, texColor.x)), 2.2f),
                                std::pow(std::max(0.0f, std::min(1.0f, texColor.y)), 2.2f),
                                std::pow(std::max(0.0f, std::min(1.0f, texColor.z)), 2.2f));
            }

            // Normalize if needed
            if (tm->normalizer > 0.0f) {
                if (tm->image->isHDR) {
                    texColor = texColor.scale(1.0f / tm->normalizer);
                } else {
                    texColor = texColor.scale(255.0f / tm->normalizer);
                }
            }

            if (tm->decalMode == DecalMode::ReplaceKd)
                kd = texColor;
            else if (tm->decalMode == DecalMode::BlendKd) {
                // Blend with existing kd
                kd = kd.add(texColor).scale(0.5f);
            }

            if (tm->decalMode == DecalMode::ReplaceKs) ks = texColor;

            if (tm->decalMode == DecalMode::ReplaceAll) {
                doReplaceAll = true;
                replaceAllColor = texColor;
            }
        }
    }

    // Return immediately if ReplaceAll was applied
    if (doReplaceAll)
        return replaceAllColor.scale(255.0f);

    // Normal Mapping (ReplaceNormal)
    if (info.hasUV && info.hasTBN && info.textureMapIds) {
        const ImageTextureMap* normalTM = nullptr;
        for (int texMapId : *info.textureMapIds) {
            const ImageTextureMap* tm = findImageTexMap(scene, texMapId);
            if (!tm || !tm->image) continue;
            if (tm->decalMode == DecalMode::ReplaceNormal) {
                normalTM = tm;
                break;
            }
        }

        if (normalTM) {
            Vec3 c = normalTM->image->sampleNearest(info.hitUV);

            // [0,1] -> [-1,1]
            Vec3 nTS(c.x * 2.0f - 1.0f, c.y * 2.0f - 1.0f, c.z * 2.0f - 1.0f);
            nTS = nTS.normalize();

            // Transform to world space
            Vec3 T = info.tangentW.normalize();
            Vec3 B = info.bitangentW.normalize();
            Vec3 N = hitNormal.normalize();

            Vec3 nW = T.scale(nTS.x).add(B.scale(nTS.y)).add(N.scale(nTS.z)).normalize();

            if (nW.dot(rayDirN) > 0.0f) nW = nW.scale(-1.0f);
            hitNormal = nW;
        }
    }

    // Bump Mapping (BumpNormal)
    if (info.hasUV && info.hasTBN && info.textureMapIds) {
        const ImageTextureMap *bumpTM = nullptr;
        bool hasReplaceNormal = false;

        for (int texMapId: *info.textureMapIds) {
            const ImageTextureMap *tm = findImageTexMap(scene, texMapId);
            if (!tm || !tm->image) continue;
            if (tm->decalMode == DecalMode::ReplaceNormal) hasReplaceNormal = true;
            if (tm->decalMode == DecalMode::BumpNormal) bumpTM = tm;
        }

        if (!hasReplaceNormal && bumpTM) {
            float du = 1.0f / (float) bumpTM->image->width;
            float dv = 1.0f / (float) bumpTM->image->height;

            // Height function H(u,v)
            auto H = [&](float u, float v) {
                Vec2 uv(u, v);
                Vec3 c = (bumpTM->interpolation == InterpolationMode::Bilinear)
                       ? bumpTM->image->sampleBilinear(uv)
                       : bumpTM->image->sampleNearest(uv);
                return (c.x + c.y + c.z) / 3.0f;
            };

            // Compute height and its derivatives
            float h = H(info.hitUV.u, info.hitUV.v);
            float hx = H(info.hitUV.u + du, info.hitUV.v) - h;
            float hy = H(info.hitUV.u, info.hitUV.v + dv) - h;

            // Apply bump factor
            hx *= bumpTM->bumpFactor;
            hy *= bumpTM->bumpFactor;

            // Tangent space normal
            Vec3 nTS = Vec3(-hx, -hy, 1.0f).normalize();

            // Transform to world space
            Vec3 T = info.tangentW.normalize();
            Vec3 B = info.bitangentW.normalize();
            Vec3 N = hitNormal.normalize();
            Vec3 nW = T.scale(nTS.x).add(B.scale(nTS.y)).add(N.scale(nTS.z)).normalize();

            if (nW.dot(rayDirN) > 0) nW = nW.scale(-1.0f);
            hitNormal = nW;
        }
    }

    // Procedural Checkerboard Texture
    if (info.textureMapIds) {
        Vec3 posForProc = hitPoint; // Default world space

        glm::mat4 invM = glm::inverse(info.modelMatrix);
        glm::vec4 pL = invM * glm::vec4(hitPoint.x, hitPoint.y, hitPoint.z, 1.0f);
        posForProc = Vec3(pL.x, pL.y, pL.z); // Local space

        for (int texMapId : *info.textureMapIds) {
            const CheckerTextureMap* tm = findCheckerTexMap(scene, texMapId);
            if (!tm) continue;

            Vec3 texColor = sampleCheckerboard(*tm, posForProc);

            // Apply decal modes
            if (tm->decalMode == DecalMode::ReplaceKd) {
                kd = texColor;
            } else if (tm->decalMode == DecalMode::BlendKd) {
                kd = kd.add(texColor).scale(0.5f);
            } else if (tm->decalMode == DecalMode::ReplaceKs) {
                ks = texColor;
            } else if (tm->decalMode == DecalMode::ReplaceAll) {
                doReplaceAll = true;
                replaceAllColor = texColor;
            }
        }
    }

    // Return immediately if ReplaceAll was applied
    if (doReplaceAll)
        return replaceAllColor.scale(255.0f);

    // Perlin
    if (info.textureMapIds) {
        Vec3 posForProc = hitPoint;

        glm::mat4 invM = glm::inverse(info.modelMatrix);
        glm::vec4 pL = invM * glm::vec4(hitPoint.x, hitPoint.y, hitPoint.z, 1.0f);
        posForProc = Vec3(pL.x, pL.y, pL.z); // Local space

        for (int texMapId : *info.textureMapIds) {
            const PerlinTextureMap* tm = findPerlinTexMap(scene, texMapId);
            if (!tm) continue;

            if (tm->decalMode == DecalMode::BumpNormal && info.hasTBN) { // Perlin Bump Mapping
                // For finite difference
                float eps = 0.001f;

                float h = perlinTextureValue(*tm, posForProc);

                Vec3 P_world_u = hitPoint.add(info.tangentW.scale(eps));
                glm::vec4 pLu_vec = invM * glm::vec4(P_world_u.x, P_world_u.y, P_world_u.z, 1.0f);
                Vec3 pos_u(pLu_vec.x, pLu_vec.y, pLu_vec.z);
                float h_u = perlinTextureValue(*tm, pos_u);

                Vec3 P_world_v = hitPoint.add(info.bitangentW.scale(eps));
                glm::vec4 pLv_vec = invM * glm::vec4(P_world_v.x, P_world_v.y, P_world_v.z, 1.0f);
                Vec3 pos_v(pLv_vec.x, pLv_vec.y, pLv_vec.z);
                float h_v = perlinTextureValue(*tm, pos_v);

                // Calculate derivatives
                float dh_du = (h_u - h) / eps;
                float dh_dv = (h_v - h) / eps;

                // Apply bump factor
                dh_du *= tm->bumpFactor;
                dh_dv *= tm->bumpFactor;

                Vec3 nTS(-dh_du, -dh_dv, 1.0f);
                nTS = nTS.normalize();

                // Go to world space
                Vec3 T = info.tangentW.normalize();
                Vec3 B = info.bitangentW.normalize();
                Vec3 N = hitNormal.normalize();

                Vec3 nW = T.scale(nTS.x).add(B.scale(nTS.y)).add(N.scale(nTS.z)).normalize();

                if (nW.dot(rayDirN) > 0.0f) nW = nW.scale(-1.0f); // Fix the normal direction if needed
                hitNormal = nW;
            }

            else { // Perlin
                Vec3 texColor = samplePerlinTex(*tm, posForProc);

                if (tm->decalMode == DecalMode::ReplaceKd) kd = texColor;
                else if (tm->decalMode == DecalMode::BlendKd) kd = kd.add(texColor).scale(0.5f);
                else if (tm->decalMode == DecalMode::ReplaceKs) ks = texColor;
                else if (tm->decalMode == DecalMode::ReplaceAll) {
                    doReplaceAll = true;
                    replaceAllColor = texColor;
                }
            }
        }
    }

    // BRDF
    Vec3 wo = rayOrigin.subtract(hitPoint).normalize();
    const BRDF *brdf = scene.getBRDFById(material.brdfId);
    float eta = (material.refractionIndex > 0.0f) ? material.refractionIndex : 1.5f;

    // Ambient
    Vec3 color = ka.multiply(scene.ambientLight);

    // Point Lights Shadow and Shading
    for (const auto &light : scene.pointLights) {
        Vec3 lightDir = light.position.subtract(hitPoint).normalize();
        Vec3 toLight = light.position.subtract(hitPoint);
        float lightDist = toLight.length();

        Vec3 shadowOrigin = hitPoint.add(hitNormal.scale(scene.shadowRayEpsilon));

        if (isInShadow(scene, shadowOrigin, lightDir, lightDist, intersector, rayTime)) continue;

        // Li = I / r^2
        Vec3 Li = light.intensity.scale(1.0f / (lightDist * lightDist));

        float cosI = std::max(0.0f, hitNormal.dot(lightDir));
        if (cosI <= 0.0f) continue;

        if (brdf) {
            // f(wi, wo)
            Vec3 f = brdf->eval(hitNormal, lightDir, wo, kd, ks, eta, material.absorptionIndex);

            // Lo += Li * f * cos(theta_i)
            color = color.add(Li.multiply(f).scale(cosI));
        }
        else {
            Vec3 diffuse = kd.multiply(Li).scale(cosI);

            Vec3 halfVec = lightDir.add(wo).normalize();
            float NdotH = std::max(0.0f, hitNormal.dot(halfVec));
            Vec3 specular = ks.multiply(Li).scale(pow(NdotH, material.phongExponent));

            color = color.add(diffuse).add(specular);
        }
    }

    // Area Lights Shadow and Shading
    for (const auto &light: scene.areaLights) {
        Vec3 uL, vL;
        buildAreaLightBasis(light.normal, uL, vL);

        int numSamples = max(1, scene.areaLightNumSamples);
        float A = light.size * light.size;

        Vec3 areaContribution(0, 0, 0);

        for (int s = 0; s < numSamples; ++s) {
            float xi1 = dist01(rng);
            float xi2 = dist01(rng);

            float half = light.size * 0.5f;
            // [-half, half] range
            float sx = (xi1 * 2.0f - 1.0f) * half;
            float sy = (xi2 * 2.0f - 1.0f) * half;

            // Sampled point
            Vec3 lightPoint = light.position.add(uL.scale(sx)).add(vL.scale(sy));

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

            float cosI = std::max(0.0f, hitNormal.dot(wi));
            if (cosI <= 0.0f) continue;

            if (brdf) {
                Vec3 f = brdf->eval(hitNormal, wi, wo, kd, ks, eta, material.absorptionIndex);
                areaContribution = areaContribution.add(eff.multiply(f).scale(cosI));
            }
            else {
                Vec3 diffuse = kd.multiply(eff).scale(cosI);

                Vec3 halfVec = wi.add(wo).normalize();
                float NdotH = std::max(0.0f, hitNormal.dot(halfVec));
                Vec3 specular = ks.multiply(eff).scale(pow(NdotH, material.phongExponent));

                areaContribution = areaContribution.add(diffuse).add(specular);
            }
        }

        color = color.add(areaContribution);
    }

    // Directional Lights Shadow and Shading
    for (const auto& dl : scene.directionalLights) {
        Vec3 wi = dl.direction.scale(-1.0f).normalize();
        Vec3 shadowOrigin = hitPoint.add(hitNormal.scale(scene.shadowRayEpsilon));

        if (isInShadow(scene, shadowOrigin, wi, 1e9f, intersector, rayTime)) continue;

        Vec3 Li = dl.radiance;

        float cosI = std::max(0.0f, hitNormal.dot(wi));
        if (cosI <= 0.0f) continue;

        if (brdf) {
            Vec3 f = brdf->eval(hitNormal, wi, wo, kd, ks, eta, material.absorptionIndex);
            color = color.add(Li.multiply(f).scale(cosI));
        }
        else {
            Vec3 diffuse = kd.multiply(Li).scale(cosI);

            Vec3 halfVec = wi.add(wo).normalize();
            float NdotH = std::max(0.0f, hitNormal.dot(halfVec));
            Vec3 specular = ks.multiply(Li).scale(std::pow(NdotH, material.phongExponent));

            color = color.add(diffuse).add(specular);
        }
    }

    // Spot Lights Shadow and Shading
    for (const auto &sl: scene.spotLights) {
        Vec3 L = sl.position.subtract(hitPoint);
        float d2 = L.length_squared();
        if (d2 <= 0.0f) continue;

        float d = std::sqrt(d2);
        L = L.scale(1.0f / d);

        Vec3 shadowOrigin = hitPoint.add(hitNormal.scale(scene.shadowRayEpsilon));
        if (isInShadow(scene, shadowOrigin, L, d, intersector, rayTime)) continue;

        Vec3 spotAxis = sl.direction.normalize();
        Vec3 lightToHit = L.scale(-1.0f);
        float cosAlpha = std::max(-1.0f, std::min(1.0f, spotAxis.dot(lightToHit)));

        float cov = glm::radians(sl.coverageAngle * 0.5f);
        float fall = glm::radians(sl.falloffAngle * 0.5f);

        float cosCov = std::cos(cov);
        float cosFall = std::cos(fall);

        float spotFactor = 0.0f;

        if (cosAlpha >= cosFall) {
            spotFactor = 1.0f;
        }
        else if (cosAlpha >= cosCov) {
            float denom = (cosFall - cosCov);
            float t = (cosAlpha - cosCov) / denom;
            if (t < 0.0f) t = 0.0f;
            if (t > 1.0f) t = 1.0f;
            spotFactor = std::pow(t, 4.0f);
        }
        else {
            continue;
        }

        Vec3 Li = sl.intensity.scale(spotFactor / d2);

        float cosI = std::max(0.0f, hitNormal.dot(L));
        if (cosI <= 0.0f) continue;

        if (brdf) {
            Vec3 f = brdf->eval(hitNormal, L, wo, kd, ks, eta, material.absorptionIndex);
            color = color.add(Li.multiply(f).scale(cosI));
        }
        else {
            Vec3 diffuse = kd.multiply(Li).scale(cosI);

            Vec3 halfVec = L.add(wo).normalize();
            float NdotH = std::max(0.0f, hitNormal.dot(halfVec));
            Vec3 specular = ks.multiply(Li).scale(std::pow(NdotH, material.phongExponent));

            color = color.add(diffuse).add(specular);
        }
    }

    // Environment Lights Shadow and Shading
    for (const auto &el: scene.envLights) {
        if (!el.image) continue;

        float u1 = dist01(rng);
        float u2 = dist01(rng);

        float pdf = 0.0f;
        Vec3 wi = sampleHemisphere(hitNormal, el.sampler, u1, u2, pdf);
        if (pdf <= 0.0f) continue;

        Vec3 shadowOrigin = hitPoint.add(hitNormal.scale(scene.shadowRayEpsilon));
        if (isInShadow(scene, shadowOrigin, wi, 1e9f, intersector, rayTime)) continue;

        Vec3 L = sampleEnv(el, wi);

        Vec3 eff = L.scale(1.0f / pdf);

        float cosI = std::max(0.0f, hitNormal.dot(wi));
        if (cosI <= 0.0f) continue;

        if (brdf) {
            Vec3 f = brdf->eval(hitNormal, wi, wo, kd, ks, eta, material.absorptionIndex);
            color = color.add(eff.multiply(f).scale(cosI));
        }
        else {
            Vec3 diffuse = kd.multiply(eff).scale(cosI);

            Vec3 halfVec = wi.add(wo).normalize();
            float NdotH = std::max(0.0f, hitNormal.dot(halfVec));
            Vec3 specular = ks.multiply(eff).scale(std::pow(NdotH, material.phongExponent));

            color = color.add(diffuse).add(specular);
        }
    }

    // Light Meshes
    for (const auto &lm: scene.objects.lightMeshes) {
        int numSamples = std::max(1, scene.areaLightNumSamples);
        int triCount = (int) lm.data.size() / 3;
        if (triCount <= 0) continue;

        double totalAreaD = 0.0;
        std::vector<double> triAreas;
        triAreas.resize(triCount);

        for (int t = 0; t < triCount; ++t) {
            int i0 = lm.data[3 * t + 0] - 1;
            int i1 = lm.data[3 * t + 1] - 1;
            int i2 = lm.data[3 * t + 2] - 1;

            Vec3 v0 = scene.vertexData[i0];
            Vec3 v1 = scene.vertexData[i1];
            Vec3 v2 = scene.vertexData[i2];

            glm::vec3 w0 = glm::vec3(lm.modelMatrix * glm::vec4(v0.x, v0.y, v0.z, 1));
            glm::vec3 w1 = glm::vec3(lm.modelMatrix * glm::vec4(v1.x, v1.y, v1.z, 1));
            glm::vec3 w2 = glm::vec3(lm.modelMatrix * glm::vec4(v2.x, v2.y, v2.z, 1));

            Vec3 a(w0.x, w0.y, w0.z), b(w1.x, w1.y, w1.z), c(w2.x, w2.y, w2.z);
            double A = 0.5 * (b.subtract(a).cross(c.subtract(a))).length();
            triAreas[t] = A;
            totalAreaD += A;
        }

        if (totalAreaD <= 0.0) continue;
        float totalArea = (float) totalAreaD;

        Vec3 meshContribution(0, 0, 0);

        for (int s = 0; s < numSamples; ++s) {
            double r = dist01(rng) * totalAreaD;
            int chosen = 0;
            double acc = 0.0;
            for (int t = 0; t < triCount; ++t) {
                acc += triAreas[t];
                if (acc >= r) {
                    chosen = t;
                    break;
                }
            }

            int i0 = lm.data[3 * chosen + 0] - 1;
            int i1 = lm.data[3 * chosen + 1] - 1;
            int i2 = lm.data[3 * chosen + 2] - 1;

            Vec3 v0 = scene.vertexData[i0];
            Vec3 v1 = scene.vertexData[i1];
            Vec3 v2 = scene.vertexData[i2];

            glm::vec3 w0 = glm::vec3(lm.modelMatrix * glm::vec4(v0.x, v0.y, v0.z, 1));
            glm::vec3 w1 = glm::vec3(lm.modelMatrix * glm::vec4(v1.x, v1.y, v1.z, 1));
            glm::vec3 w2 = glm::vec3(lm.modelMatrix * glm::vec4(v2.x, v2.y, v2.z, 1));

            Vec3 a(w0.x, w0.y, w0.z), b(w1.x, w1.y, w1.z), c(w2.x, w2.y, w2.z);

            float u1 = dist01(rng);
            float u2 = dist01(rng);
            float su1 = std::sqrt(u1);
            float b0 = 1.0f - su1;
            float b1 = su1 * (1.0f - u2);
            float b2 = su1 * u2;

            Vec3 lightPoint = a.scale(b0).add(b.scale(b1)).add(c.scale(b2));

            Vec3 toLight = lightPoint.subtract(hitPoint);
            float d2 = toLight.length_squared();
            if (d2 <= 0.0f) continue;

            float d = std::sqrt(d2);
            Vec3 wi = toLight.scale(1.0f / d);

            Vec3 nTri = (b.subtract(a).cross(c.subtract(a))).normalize();

            float cosLight = std::max(0.0f, std::fabs(nTri.dot(wi.scale(-1.0f))));
            if (cosLight <= 0.0f) continue;

            Vec3 shadowOrigin = hitPoint.add(hitNormal.scale(scene.shadowRayEpsilon));
            if (isInShadow(scene, shadowOrigin, wi, d - scene.shadowRayEpsilon, intersector, rayTime))
                continue;

            float geom = (totalArea * cosLight) / (d2 * (float) numSamples);

            Vec3 Li = lm.radiance;
            Vec3 eff = Li.scale(geom);

            float cosI = std::max(0.0f, hitNormal.dot(wi));
            if (cosI <= 0.0f) continue;

            if (brdf) {
                Vec3 f = brdf->eval(hitNormal, wi, wo, kd, ks, eta, material.absorptionIndex);
                meshContribution = meshContribution.add(eff.multiply(f).scale(cosI));
            } else {
                Vec3 diffuse = kd.multiply(eff).scale(cosI);

                Vec3 halfVec = wi.add(wo).normalize();
                float NdotH = std::max(0.0f, hitNormal.dot(halfVec));
                Vec3 specular = ks.multiply(eff).scale(std::pow(NdotH, material.phongExponent));

                meshContribution = meshContribution.add(diffuse).add(specular);
            }
        }

        color = color.add(meshContribution);
    }

    // Light Spheres
    for (const auto &ls: scene.objects.lightSpheres) {
        int numSamples = std::max(1, scene.areaLightNumSamples);

        Vec3 cLocal = scene.vertexData[ls.center - 1];

        glm::vec3 col0 = glm::vec3(ls.modelMatrix[0]);
        glm::vec3 col1 = glm::vec3(ls.modelMatrix[1]);
        glm::vec3 col2 = glm::vec3(ls.modelMatrix[2]);

        float sx = glm::length(col0);
        float sy = glm::length(col1);
        float sz = glm::length(col2);

        float a = ls.radius * sx;
        float b = ls.radius * sy;
        float c = ls.radius * sz;

        float p = 1.6075f;
        float term = (std::pow(a * b, p) + std::pow(a * c, p) + std::pow(b * c, p)) / 3.0f;
        float area = 4.0f * (float) M_PI * std::pow(term, 1.0f / p);

        if (area <= 0.0f) continue;

        glm::mat3 normalMatrix = glm::transpose(glm::inverse(glm::mat3(ls.modelMatrix)));

        Vec3 sphereContribution(0, 0, 0);

        for (int s = 0; s < numSamples; ++s) {
            float u1 = dist01(rng);
            float u2 = dist01(rng);

            float z = 1.0f - 2.0f * u1;
            float r = std::sqrt(std::max(0.0f, 1.0f - z * z));
            float phi = 2.0f * (float) M_PI * u2;

            Vec3 rndDir(r * std::cos(phi), r * std::sin(phi), z);

            Vec3 localSurfacePoint = cLocal.add(rndDir.scale(ls.radius));

            glm::vec4 worldPosGLM = ls.modelMatrix * glm::vec4(localSurfacePoint.x, localSurfacePoint.y,
                                                               localSurfacePoint.z, 1.0f);
            Vec3 lightPoint(worldPosGLM.x, worldPosGLM.y, worldPosGLM.z);

            glm::vec3 worldNormalGLM = glm::normalize(normalMatrix * glm::vec3(rndDir.x, rndDir.y, rndDir.z));
            Vec3 nLight(worldNormalGLM.x, worldNormalGLM.y, worldNormalGLM.z);

            Vec3 toLight = lightPoint.subtract(hitPoint);
            float d2 = toLight.length_squared();
            if (d2 <= 0.0f) continue;

            float d = std::sqrt(d2);
            Vec3 wi = toLight.scale(1.0f / d);

            float cosLight = std::max(0.0f, std::fabs(nLight.dot(wi.scale(-1.0f))));
            if (cosLight <= 0.0f) continue;

            Vec3 shadowOrigin = hitPoint.add(hitNormal.scale(scene.shadowRayEpsilon));
            if (isInShadow(scene, shadowOrigin, wi, d, intersector, rayTime))
                continue;

            float geom = (area * cosLight) / (d2 * (float) numSamples);

            Vec3 Li = ls.radiance;
            Vec3 eff = Li.scale(geom);

            float cosI = std::max(0.0f, hitNormal.dot(wi));
            if (cosI <= 0.0f) continue;

            if (brdf) {
                Vec3 f = brdf->eval(hitNormal, wi, wo, kd, ks, eta, material.absorptionIndex);
                sphereContribution = sphereContribution.add(eff.multiply(f).scale(cosI));
            }
            else {
                Vec3 diffuse = kd.multiply(eff).scale(cosI);

                Vec3 halfVec = wi.add(wo).normalize();
                float NdotH = std::max(0.0f, hitNormal.dot(halfVec));
                Vec3 specular = ks.multiply(eff).scale(std::pow(NdotH, material.phongExponent));

                sphereContribution = sphereContribution.add(diffuse).add(specular);
            }
        }

        color = color.add(sphereContribution);
    }


    // Mirror Reflectance
    if (material.type == "mirror") {
        // Reflection Ray
        Vec3 idealReflectDir = reflect(rayDirN, hitNormal).normalize();

        // Perturb reflection direction based on roughness
        Vec3 reflectDir = perturbDirection(idealReflectDir, material.roughness, rng);
        Vec3 reflectOrigin = hitPoint.add(hitNormal.scale(scene.shadowRayEpsilon));
        Vec3 reflectedColor = traceRay(scene, reflectOrigin, reflectDir, depth + 1, intersector, rayTime, bgUV, rng);

        // Combine
        color = color.add(reflectedColor.multiply(material.mirrorReflectance));
    }

    // Dielectrics
    else if (material.type == "dielectric") {
        float ni = 1.0f; // Incident index (air)
        float nt = material.refractionIndex; // Transmitted index (material)

        Vec3 N = hitNormal;
        float cosI = -rayDirN.dot(N);
        bool isInside = false;

        // Is the ray inside the material?
        if (cosI < 0.0f) {
            // Going from inside to outside
            isInside = true;
            std::swap(ni, nt);
            N = N.scale(-1.0f); // Flip normal
            cosI = -rayDirN.dot(N);
        }

        float eta = ni / nt;
        float R = fresnelDielectric(cosI, eta);
        float T = 1.0f - R;

        // Reflection
        Vec3 idealReflectDir = reflect(rayDirN, N).normalize();
        Vec3 reflectDir = perturbDirection(idealReflectDir, material.roughness, rng);

        Vec3 reflOrigin = hitPoint.add(N.scale(scene.shadowRayEpsilon));

        Vec3 reflectedColor = traceRay(scene, reflOrigin, reflectDir, depth + 1, intersector, rayTime, bgUV, rng);

        if (material.mirrorReflectance.x > 0 || material.mirrorReflectance.y > 0 || material.mirrorReflectance.z > 0) {
            reflectedColor = reflectedColor.multiply(material.mirrorReflectance);
        }

        // Refraction
        Vec3 refractedColor(0, 0, 0);

        // If T > 0, calculate refraction (otherwise total internal reflection)
        if (T > 0.0f) {
            Vec3 idealRefractDir;
            if (refract(rayDirN, N, eta, idealRefractDir)) {
                Vec3 refractDir = perturbDirection(idealRefractDir, material.roughness, rng);

                Vec3 refractOrigin = hitPoint.add(refractDir.scale(scene.shadowRayEpsilon)); // Offset along refracted direction

                refractedColor = traceRay(scene, refractOrigin, refractDir, depth + 1, intersector, rayTime, bgUV, rng);

                // Beer's Law Absorption
                if (isInside) {
                    Vec3 absorbance = material.absorptionCoefficient;
                    float distance = info.t; // Distance traveled inside the material

                    Vec3 beerFactor(std::exp(-absorbance.x * distance),
                                    std::exp(-absorbance.y * distance),
                                    std::exp(-absorbance.z * distance));

                    refractedColor = refractedColor.multiply(beerFactor);
                }
            } else {
                R = 1.0f;
                T = 0.0f;
            }
        } else {
            // Total Internal Reflection
        }

        color = color.add(reflectedColor.scale(R));
        color = color.add(refractedColor.scale(T));
    }

    // Conductors
    else if (material.type == "conductor") {
        Vec3 idealReflectDir = reflect(rayDirN, hitNormal).normalize();
        Vec3 reflectDir = perturbDirection(idealReflectDir, material.roughness, rng);

        Vec3 reflectOrigin = hitPoint.add(hitNormal.scale(scene.shadowRayEpsilon));
        Vec3 reflectedColor = traceRay(scene, reflectOrigin, reflectDir,
                                       depth + 1, intersector, rayTime, bgUV, rng);

        Vec3 F = fresnelConductor(rayDirN, hitNormal.normalize(),
                                  material.refractionIndex, material.absorptionIndex);

        color = color.add(reflectedColor.multiply(material.mirrorReflectance).multiply(F));
    }

    return color;
}

Vec3 tracePath(const Scene &scene, const Vec3 &rayOrigin, const Vec3 &rayDir, const Camera &cam, Intersector &intersector, float rayTime, const Vec2 &bgUV, std::mt19937 &rng) {
    std::uniform_real_distribution<float> dist01(0.0f, 1.0f);

    Vec3 L(0, 0, 0);
    Vec3 pathThroughput(1, 1, 1);
    Vec3 ro = rayOrigin;
    Vec3 rd = rayDir.normalize();

    bool lastSpecular = true;

    for (int depth = 0; depth <= cam.maxRecursionDepth; ++depth) {
        IntersectionInfo info = intersector.findClosestIntersection(ro, rd, false, rayTime);

        if (!info.hit || info.materialId == -1) {
            Vec3 env(0, 0, 0);
            if (!scene.envLights.empty()) {
                const auto &el = scene.envLights[0];
                if (el.image)
                    env = sampleEnv(el, rd);
            }
            else if (scene.backgroundTextureMapId != -1) {
                const ImageTextureMap *tm = findImageTexMap(scene, scene.backgroundTextureMapId);
                if (tm && tm->image) {
                    float u = std::max(0.0f, std::min(1.0f, bgUV.u));
                    float v = std::max(0.0f, std::min(1.0f, bgUV.v));
                    Vec3 c01 = tm->image->sampleNearest(Vec2(u, v));
                    env = c01.scale(255.0f);
                }
            } else {
                env = scene.backgroundColor;
            }

            if (lastSpecular || !cam.pathTracingOptions.nextEventEstimation) {
                L = L.add(pathThroughput.multiply(env));
            }
            break;
        }

        if (info.isEmissive) {
            if (lastSpecular || !cam.pathTracingOptions.nextEventEstimation) {
                L = L.add(pathThroughput.multiply(info.emission));
            }

            break;
        }

        Material material;
        for (const auto &m: scene.materials) {
            if (m.id == info.materialId) {
                material = m;
                break;
            }
        }

        Vec3 hitPoint = info.hitPoint;
        Vec3 hitNormal = info.hitNormal.normalize();
        Vec3 wo = rd.scale(-1.0f).normalize();

        // Texture mapping as in traceRay
        Vec3 kd = material.diffuseReflectance;
        Vec3 ks = material.specularReflectance;

        // Image Textures
        if (info.hasUV && info.textureMapIds) {
            for (int texMapId: *info.textureMapIds) {
                const ImageTextureMap *tm = findImageTexMap(scene, texMapId);
                if (!tm || !tm->image) continue;

                Vec3 texColor;
                if (tm->interpolation == InterpolationMode::Bilinear) {
                    texColor = tm->image->sampleBilinear(info.hitUV);
                } else {
                    texColor = tm->image->sampleNearest(info.hitUV);
                }

                // Only degamma for LDR texture
                // No degamma for normal maps or HDR textures
                bool affectsColor =
                    (tm->decalMode == DecalMode::ReplaceKd) ||
                    (tm->decalMode == DecalMode::BlendKd)   ||
                    (tm->decalMode == DecalMode::ReplaceKs) ||
                    (tm->decalMode == DecalMode::ReplaceAll);

                if (tm->degamma && affectsColor && tm->image && !tm->image->isHDR) {
                    texColor = Vec3(std::pow(std::max(0.0f, std::min(1.0f, texColor.x)), 2.2f),
                                    std::pow(std::max(0.0f, std::min(1.0f, texColor.y)), 2.2f),
                                    std::pow(std::max(0.0f, std::min(1.0f, texColor.z)), 2.2f));
                }

                if (tm->normalizer > 0.0f) {
                    if (tm->image->isHDR)
                        texColor = texColor.scale(1.0f / tm->normalizer);
                    else
                        texColor = texColor.scale(255.0f / tm->normalizer);
                }

                if (tm->decalMode == DecalMode::ReplaceKd)
                    kd = texColor;
                else if (tm->decalMode == DecalMode::BlendKd) {
                    // Blend with existing kd
                    kd = kd.add(texColor).scale(0.5f);
                }

                else if (tm->decalMode == DecalMode::ReplaceKs)
                    ks = texColor;

                else if (tm->decalMode == DecalMode::ReplaceAll) {
                    kd = texColor;
                    ks = Vec3(0, 0, 0);
                }
            }
        }

        // Normal Mapping (ReplaceNormal)
        if (info.hasUV && info.hasTBN && info.textureMapIds) {
            const ImageTextureMap *normalTM = nullptr;
            for (int texMapId: *info.textureMapIds) {
                const ImageTextureMap *tm = findImageTexMap(scene, texMapId);
                if (!tm || !tm->image) continue;
                if (tm->decalMode == DecalMode::ReplaceNormal) {
                    normalTM = tm;
                    break;
                }
            }
            if (normalTM) {
                Vec3 c = normalTM->image->sampleNearest(info.hitUV);

                Vec3 nTS(c.x * 2.0f - 1.0f, c.y * 2.0f - 1.0f, c.z * 2.0f - 1.0f);
                nTS = nTS.normalize();

                Vec3 T = info.tangentW.normalize();
                Vec3 B = info.bitangentW.normalize();
                Vec3 N = hitNormal; // current shading normal

                Vec3 nW = T.scale(nTS.x).add(B.scale(nTS.y)).add(N.scale(nTS.z)).normalize();

                hitNormal = nW;
            }
        }

        // Bump Mapping (BumpNormal)
        if (info.hasUV && info.hasTBN && info.textureMapIds) {
            const ImageTextureMap *bumpTM = nullptr;
            bool hasReplaceNormal = false;

            for (int texMapId: *info.textureMapIds) {
                const ImageTextureMap *tm = findImageTexMap(scene, texMapId);
                if (!tm || !tm->image) continue;
                if (tm->decalMode == DecalMode::ReplaceNormal) hasReplaceNormal = true;
                if (tm->decalMode == DecalMode::BumpNormal) bumpTM = tm;
            }

            if (!hasReplaceNormal && bumpTM) {
                float du = 1.0f / (float) bumpTM->image->width;
                float dv = 1.0f / (float) bumpTM->image->height;

                // Height function H(u,v)
                auto H = [&](float u, float v) {
                    Vec2 uv(u, v);
                    Vec3 c = (bumpTM->interpolation == InterpolationMode::Bilinear)
                                 ? bumpTM->image->sampleBilinear(uv)
                                 : bumpTM->image->sampleNearest(uv);
                    return (c.x + c.y + c.z) / 3.0f;
                };

                // Compute height and its derivatives
                float h = H(info.hitUV.u, info.hitUV.v);
                float hx = (H(info.hitUV.u + du, info.hitUV.v) - h) * bumpTM->bumpFactor;
                float hy = (H(info.hitUV.u, info.hitUV.v + dv) - h) * bumpTM->bumpFactor;

                Vec3 T = info.tangentW.normalize();
                Vec3 B = info.bitangentW.normalize();
                Vec3 N = hitNormal;

                Vec3 nTS = Vec3(-hx, -hy, 1.0f).normalize();
                Vec3 nW = T.scale(nTS.x).add(B.scale(nTS.y)).add(N.scale(nTS.z)).normalize();
                hitNormal = nW;
            }
        }

        // Procedural Textures (Checkerboard & Perlin)
        if (info.textureMapIds) {
            Vec3 posForProc = hitPoint; // Default world space

            glm::mat4 invM = glm::inverse(info.modelMatrix);
            glm::vec4 pL = invM * glm::vec4(hitPoint.x, hitPoint.y, hitPoint.z, 1.0f);
            posForProc = Vec3(pL.x, pL.y, pL.z); // Local space

            for (int texMapId: *info.textureMapIds) {
                const CheckerTextureMap *tm = findCheckerTexMap(scene, texMapId);

                if (tm) {
                    Vec3 texColor = sampleCheckerboard(*tm, posForProc);

                    if (tm->decalMode == DecalMode::ReplaceKd)
                        kd = texColor;
                    else if (tm->decalMode == DecalMode::BlendKd)
                        kd = kd.add(texColor).scale(0.5f);
                    else if (tm->decalMode == DecalMode::ReplaceKs)
                        ks = texColor;
                }

                // Perlin
                const PerlinTextureMap *ptm = findPerlinTexMap(scene, texMapId);

                if (ptm) {
                    if (ptm->decalMode == DecalMode::BumpNormal && info.hasTBN) { // Perlin Bump Mapping
                        // For finite difference
                        float eps = 0.001f;

                        float h = perlinTextureValue(*ptm, posForProc);

                        Vec3 P_world_u = hitPoint.add(info.tangentW.scale(eps));
                        glm::vec4 pLu_vec = invM * glm::vec4(P_world_u.x, P_world_u.y, P_world_u.z, 1.0f);
                        float h_u = perlinTextureValue(*ptm, Vec3(pLu_vec.x, pLu_vec.y, pLu_vec.z));

                        Vec3 P_world_v = hitPoint.add(info.bitangentW.scale(eps));
                        glm::vec4 pLv_vec = invM * glm::vec4(P_world_v.x, P_world_v.y, P_world_v.z, 1.0f);
                        float h_v = perlinTextureValue(*ptm, Vec3(pLv_vec.x, pLv_vec.y, pLv_vec.z));

                        // Calculate derivatives
                        float dh_du = ((h_u - h) / eps) * ptm->bumpFactor;
                        float dh_dv = ((h_v - h) / eps) * ptm->bumpFactor;

                        Vec3 nTS = Vec3(-dh_du, -dh_dv, 1.0f).normalize();

                        // Go to world space
                        Vec3 T = info.tangentW.normalize();
                        Vec3 B = info.bitangentW.normalize();
                        Vec3 N = hitNormal;

                        hitNormal = T.scale(nTS.x).add(B.scale(nTS.y)).add(N.scale(nTS.z)).normalize();
                    }
                    else {
                        Vec3 texColor = samplePerlinTex(*ptm, posForProc);

                        if (ptm->decalMode == DecalMode::ReplaceKd) kd = texColor;
                        else if (ptm->decalMode == DecalMode::BlendKd) kd = kd.add(texColor).scale(0.5f);
                        else if (ptm->decalMode == DecalMode::ReplaceKs) ks = texColor;
                    }
                }
            }
        }

        bool hasTexture = (info.textureMapIds && !info.textureMapIds->empty());
        bool hasDiffuse = (material.diffuseReflectance.x > 1e-4f ||
                           material.diffuseReflectance.y > 1e-4f ||
                           material.diffuseReflectance.z > 1e-4f);

        if (material.type == "mirror" && !hasTexture && !hasDiffuse) {
            Vec3 reflDir = reflect(rd, hitNormal).normalize();

            if (material.roughness > 0.0f) {
                reflDir = perturbDirection(reflDir, material.roughness, rng);
            }

            pathThroughput = pathThroughput.multiply(material.mirrorReflectance);

            ro = hitPoint.add(hitNormal.scale(scene.shadowRayEpsilon));
            rd = reflDir;
            lastSpecular = true;
        }

        else if (material.type == "conductor") {
            Vec3 reflDir = reflect(rd, hitNormal).normalize();
            if (material.roughness > 0.0f) {
                reflDir = perturbDirection(reflDir, material.roughness, rng);
            }

            // Fresnel
            Vec3 F = fresnelConductor(rd, hitNormal, material.refractionIndex, material.absorptionIndex);

            pathThroughput = pathThroughput.multiply(material.mirrorReflectance).multiply(F);

            ro = hitPoint.add(hitNormal.scale(scene.shadowRayEpsilon));
            rd = reflDir;
            lastSpecular = true;
        }

        else if (material.type == "dielectric") {
            float ni = 1.0f;
            float nt = material.refractionIndex;
            Vec3 N = hitNormal;
            float cosI = -rd.dot(N);
            bool isInside = false;

            if (cosI < 0.0f) {
                isInside = true;
                std::swap(ni, nt);
                N = N.scale(-1.0f);
                cosI = -rd.dot(N);
            }

            float eta = ni / nt;
            float R = fresnelDielectric(cosI, eta);

            float rnd = dist01(rng);

            if (rnd < R) {
                Vec3 reflDir = reflect(rd, N).normalize();
                if (material.roughness > 0.0f) reflDir = perturbDirection(reflDir, material.roughness, rng);

                pathThroughput = pathThroughput.multiply(material.mirrorReflectance);

                ro = hitPoint.add(N.scale(scene.shadowRayEpsilon));
                rd = reflDir;
            } else {
                Vec3 refrDir;
                if (refract(rd, N, eta, refrDir)) {
                    if (material.roughness > 0.0f) refrDir = perturbDirection(refrDir, material.roughness, rng);

                    if (isInside) {
                        Vec3 absorbance = material.absorptionCoefficient;
                        float dist = info.t;
                        Vec3 beerFactor(std::exp(-absorbance.x * dist),
                                        std::exp(-absorbance.y * dist),
                                        std::exp(-absorbance.z * dist));
                        pathThroughput = pathThroughput.multiply(beerFactor);
                    }

                    ro = hitPoint.add(refrDir.scale(scene.shadowRayEpsilon));
                    rd = refrDir;
                } else {
                    Vec3 reflDir = reflect(rd, N).normalize();
                    ro = hitPoint.add(N.scale(scene.shadowRayEpsilon));
                    rd = reflDir;
                }
            }
            lastSpecular = true;
        }

        else {
            const BRDF *brdf = scene.getBRDFById(material.brdfId);
            float eta = (material.refractionIndex > 0.0f) ? material.refractionIndex : 1.5f;

            Vec3 directPL(0, 0, 0);

            for (const auto &pl: scene.pointLights) {
                Vec3 toL = pl.position.subtract(hitPoint);
                float d2 = toL.length_squared();
                if (d2 <= 0.0f) continue;

                float d = std::sqrt(d2);
                Vec3 wi = toL.scale(1.0f / d);

                float cosI = std::max(0.0f, hitNormal.dot(wi));
                if (cosI <= 0.0f) continue;

                Vec3 shadowOrigin = hitPoint.add(hitNormal.scale(scene.shadowRayEpsilon));
                if (isInShadow(scene, shadowOrigin, wi, d - scene.shadowRayEpsilon, intersector, rayTime)) continue;

                Vec3 Li = pl.intensity.scale(1.0f / d2);

                Vec3 f;
                if (brdf) f = brdf->eval(hitNormal, wi, wo, kd, ks, eta, material.absorptionIndex);
                else f = kd.scale(1.0f / (float) M_PI);

                directPL = directPL.add(Li.multiply(f).scale(cosI));
            }

            L = L.add(pathThroughput.multiply(directPL));

            if (cam.pathTracingOptions.nextEventEstimation) {
                int numLightSamples = 1; // Default for simple lights

                Vec3 directLight(0, 0, 0);

                for (int ls = 0; ls < numLightSamples; ++ls) {
                    LightSample lightSample = sampleOneEmitter(scene, hitPoint, hitNormal, intersector, rayTime, rng);

                    if (lightSample.valid && lightSample.pdfW > 0.0f) {
                        float cosI_light = std::max(0.0f, hitNormal.dot(lightSample.wi));
                        if (cosI_light > 0.0f) {
                            Vec3 f;

                            if (brdf)
                                f = brdf->eval(hitNormal, lightSample.wi, wo, kd, ks, eta, material.absorptionIndex);
                            else
                                f = kd.scale(1.0f / (float) M_PI);

                            // MIS Weight
                            float pdfBsdf = 0.0f;

                            if (cam.pathTracingOptions.importanceSampling)
                                pdfBsdf = cosI_light / (float) M_PI;
                            else
                                pdfBsdf = 1.0f / (2.0f * (float) M_PI);

                            float w = lightSample.isDelta
                                          ? 1.0f
                                          : misWeight(lightSample.pdfW, pdfBsdf, cam.pathTracingOptions.mis);

                            Vec3 contrib = lightSample.Li.multiply(f).scale(cosI_light * w / lightSample.pdfW);
                            directLight = directLight.add(contrib);
                        }
                    }
                }

                if (numLightSamples > 1) {
                    directLight = directLight.scale(1.0f / (float) numLightSamples);
                }

                L = L.add(pathThroughput.multiply(directLight));
            }

            float pdf = 0.0f;
            Vec3 wi = sampleHemisphere(hitNormal,
                                       cam.pathTracingOptions.importanceSampling
                                           ? EnvSampler::Cosine
                                           : EnvSampler::Uniform,
                                       dist01(rng), dist01(rng),
                                       pdf);

            if (pdf <= 0.0f) break;

            float cosI = std::max(0.0f, hitNormal.dot(wi));
            if (cosI <= 0.0f) break;

            Vec3 f;
            if (brdf) f = brdf->eval(hitNormal, wi, wo, kd, ks, eta, material.absorptionIndex);
            else f = kd.scale(1.0f / (float) M_PI);

            pathThroughput = pathThroughput.multiply(f).scale(cosI / pdf);

            ro = hitPoint.add(hitNormal.scale(scene.shadowRayEpsilon));
            rd = wi.normalize();

            lastSpecular = false;

            if (cam.pathTracingOptions.russianRoulette && depth >= cam.minRecursionDepth) {
                float p = std::max(pathThroughput.x, std::max(pathThroughput.y, pathThroughput.z));
                p = std::min(0.99f, std::max(0.05f, p));

                if (dist01(rng) > p)
                    break;

                pathThroughput = pathThroughput.scale(1.0f / p);
            }
        }
    }

    return L;
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
static Vec3 randomInUnitSphere(std::mt19937& rng) {
    std::uniform_real_distribution<float> dist(-1.0f, 1.0f);

    while (true) { //
        Vec3 p(dist(rng), dist(rng), dist(rng));
        if (p.length_squared() > 0.0f && p.length_squared() <= 1.0f)
            return p;
    }
}

// Perturb the ideal direction based on roughness
static Vec3 perturbDirection(const Vec3& idealDir, float roughness, std::mt19937& rng) {
    if (roughness <= 0.0f)
        return idealDir;

    float r = std::min(roughness, 1.0f);

    Vec3 randVec = randomInUnitSphere(rng);
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