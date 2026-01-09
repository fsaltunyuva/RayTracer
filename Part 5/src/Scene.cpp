#include <iostream>
#include <fstream>
#include "../include/Scene.h"
#include "../include/Transformation.h"

#include "../ply/happly.h"

using json = nlohmann::json;
using namespace std;

static Vec3 parseVec3(const string &str) {
    istringstream iss(str);
    Vec3 vec;
    // >> automatically casts to float
    if (!(iss >> vec.x >> vec.y >> vec.z)) {
        // Error handling
        return Vec3(0.0f, 0.0f, 0.0f);
    }
    return vec;
}

static bool readPLY(const std::string &filename, std::vector<Vec3> &outVertices, std::vector<int> &outIndices,
                    std::vector<Vec2> &outUVs) {
    try {
        happly::PLYData plyIn(filename);

        // Vertex positions
        std::vector<float> x = plyIn.getElement("vertex").getProperty<float>("x");
        std::vector<float> y = plyIn.getElement("vertex").getProperty<float>("y");
        std::vector<float> z = plyIn.getElement("vertex").getProperty<float>("z");

        outVertices.reserve(x.size());
        for (size_t i = 0; i < x.size(); ++i)
            outVertices.emplace_back(x[i], y[i], z[i]);

        // UV coordinates (optional)
        try {
            std::vector<float> u, v;
            // Check for s,t or u,v
            if (plyIn.getElement("vertex").hasProperty("s")) {
                u = plyIn.getElement("vertex").getProperty<float>("s");
                v = plyIn.getElement("vertex").getProperty<float>("t");
            } else if (plyIn.getElement("vertex").hasProperty("u")) {
                u = plyIn.getElement("vertex").getProperty<float>("u");
                v = plyIn.getElement("vertex").getProperty<float>("v");
            }

            if (!u.empty() && u.size() == x.size()) {
                outUVs.reserve(u.size());
                for (size_t i = 0; i < u.size(); ++i) {
                    outUVs.emplace_back(u[i], v[i]);
                }
            }
        } catch (...) {
            // No UVs found, continue without them
        }

        auto &faceElement = plyIn.getElement("face");
        std::vector<std::vector<int> > faces;

        if (faceElement.hasProperty("vertex_indices")) {
            faces = faceElement.getListProperty<int>("vertex_indices");
        } else if (faceElement.hasProperty("vertex_index")) {
            faces = faceElement.getListProperty<int>("vertex_index");
        } else {
            return false;
        }

        // Convert faces to triangle indices
        for (const auto &f: faces) {
            if (f.size() == 3) {
                outIndices.push_back(f[0] + 1);
                outIndices.push_back(f[1] + 1);
                outIndices.push_back(f[2] + 1);
            } else if (f.size() == 4) {
                // Quad to two triangles
                outIndices.push_back(f[0] + 1);
                outIndices.push_back(f[1] + 1);
                outIndices.push_back(f[2] + 1);
                outIndices.push_back(f[0] + 1);
                outIndices.push_back(f[2] + 1);
                outIndices.push_back(f[3] + 1);
            }
        }

        return true;
    } catch (const std::exception &e) {
        std::cerr << "PLY Read Error: " << e.what() << std::endl;
        return false;
    }
}

static void parseTransformations(const nlohmann::json &j, std::vector<std::string> &out) {
    if (!j.contains("Transformations")) return;
    std::istringstream iss(j["Transformations"].get<std::string>());
    std::string token;
    while (iss >> token)
        out.push_back(token);
}

static void parseTextureIds(const nlohmann::json &j, std::vector<int> &out) {
    // For Material or Object
    if (!j.contains("Textures")) return;

    if (j["Textures"].is_string()) {
        std::istringstream iss(j["Textures"].get<std::string>());
        int id;
        while (iss >> id) out.push_back(id);
    } else if (j["Textures"].is_array()) {
        for (const auto &v: j["Textures"]) out.push_back(v.get<int>());
    }
}

static float clamp01(float x) {
    return std::max(0.0f, std::min(1.0f, x));
}

static float srgbToLinear(float x) {
    if (x <= 0.0f) return 0.0f;
    if (x >= 1.0f) return x;
    return std::pow(x, 2.2f);
}

static Vec3 srgbToLinear3(const Vec3& c) {
    return Vec3(srgbToLinear(c.x), srgbToLinear(c.y), srgbToLinear(c.z));
}

static bool parseBool(const json& j) {
    if (j.is_boolean()) return j.get<bool>();
    if (j.is_string()) {
        std::string s = j.get<std::string>();
        for (auto& ch : s) ch = (char)std::tolower((unsigned char)ch);
        return s == "true";
    }
    return false;
}


bool Scene::loadScene(const string &filePath) {
#pragma region File Read - Error Handling
    ifstream file(filePath);
    if (!file.is_open()) {
        cerr << "Error: File could not be opened. " << filePath << endl;
        return false;
    }

    json doc;
    try {
        file >> doc;
    } catch (const json::parse_error &e) {
        cerr << "Error: JSON Parse Error: " << e.what() << endl;
        return false;
    }
#pragma endregion

#pragma region Scene Parameters
    // TODO: Set all default values here or in their respective structs/classes
    if (doc["Scene"].contains("MaxRecursionDepth"))
        this->maxRecursionDepth = stoi(doc["Scene"]["MaxRecursionDepth"].get<string>());
    else
        this->maxRecursionDepth = 5; // Default value

    if (doc["Scene"].contains("BackgroundColor")) // replace_background decal mode
        this->backgroundColor = parseVec3(doc["Scene"]["BackgroundColor"].get<string>());
    else
        this->backgroundColor = Vec3(0, 0, 0); // Default value

    if (doc["Scene"].contains("ShadowRayEpsilon"))
        this->shadowRayEpsilon = stof(doc["Scene"]["ShadowRayEpsilon"].get<string>());
    else
        this->shadowRayEpsilon = 0.001f; // Default value

    if (doc["Scene"].contains("IntersectionTestEpsilon"))
        this->intersectionTestEpsilon = stof(doc["Scene"]["IntersectionTestEpsilon"].get<string>());
    else
        this->intersectionTestEpsilon = 1e-10f; // Default value
#pragma endregion

#pragma region Camera
    auto camNode = doc["Scene"]["Cameras"]["Camera"];

    auto parseOneCamera = [&](const json &j) -> Camera {
        Camera c;

        if (j["_id"].is_string()) c.id = stoi(j["_id"].get<std::string>());
        else c.id = j["_id"].get<int>();

        c.position = parseVec3(j["Position"].get<std::string>());

        if (j.contains("Gaze"))
            c.gaze = parseVec3(j["Gaze"].get<std::string>());
        else
            c.gaze = parseVec3(j["GazePoint"].get<std::string>());

        c.up = parseVec3(j["Up"].get<std::string>());

        if (j.contains("_type") && j["_type"] == "lookAt") {
            c.hasFovY = true;
            c.fovY = std::stof(j["FovY"].get<std::string>());
            c.gazePoint = parseVec3(j["GazePoint"].get<std::string>());
            c.gaze = c.gazePoint.subtract(c.position).normalize();
        }

        if (j.contains("NearPlane")) {
            std::istringstream issNP(j["NearPlane"].get<std::string>());
            issNP >> c.nearPlane[0] >> c.nearPlane[1] >> c.nearPlane[2] >> c.nearPlane[3];
        } else {
            // If NearPlane is not provided, use default values
            c.nearPlane[0] = -1.0f;
            c.nearPlane[1] = 1.0f;
            c.nearPlane[2] = -1.0f;
            c.nearPlane[3] = 1.0f;
        }

        c.nearDistance = std::stof(j["NearDistance"].get<std::string>());

        if (j.contains("FocusDistance"))
            c.focusDistance = stof(j["FocusDistance"].get<std::string>());

        if (j.contains("ApertureSize"))
            c.apertureSize = stof(j["ApertureSize"].get<std::string>());

        std::istringstream issRes(j["ImageResolution"].get<std::string>());
        issRes >> c.imageResolution.x >> c.imageResolution.y;

        if (j.contains("NumSamples"))
            c.numSamples = stoi(j["NumSamples"].get<std::string>());
        // else default is 1

        c.imageName = j["ImageName"].get<std::string>();

        if (j.contains("Transformations")) {
            std::istringstream iss(j["Transformations"].get<std::string>());
            std::string token;
            while (iss >> token)
                c.transformationOrder.push_back(token);
        }

        if (j.contains("Tonemap")) {
            const auto &tmNode = j["Tonemap"];

            if (tmNode.is_object()) {
                Camera::ToneMapSettings t;

                if (tmNode.contains("TMO"))
                    t.tmo = tmNode["TMO"].get<string>();

                if (tmNode.contains("TMOOptions")) {
                    const auto &opt = tmNode["TMOOptions"];
                    if (opt.is_string()) {
                        std::istringstream iss(opt.get<string>());
                        iss >> t.key >> t.burnOut;
                    } else if (opt.is_array()) {
                        if (opt.size() >= 1) {
                            if (opt[0].is_string()) t.key = std::stof(opt[0].get<string>());
                            else t.key = opt[0].get<float>();
                        }
                        if (opt.size() >= 2) {
                            if (opt[1].is_string()) t.burnOut = std::stof(opt[1].get<string>());
                            else t.burnOut = opt[1].get<float>();
                        }
                    }
                }

                if (tmNode.contains("Saturation")) {
                    const auto &s = tmNode["Saturation"];
                    t.saturation = s.is_string() ? std::stof(s.get<string>()) : s.get<float>();
                }

                if (tmNode.contains("Gamma")) {
                    const auto &g = tmNode["Gamma"];
                    t.gamma = g.is_string() ? std::stof(g.get<string>()) : g.get<float>();
                }

                if (tmNode.contains("Extension"))
                    t.extension = tmNode["Extension"].get<string>();

                c.tonemaps.push_back(t);
            }
            else if (tmNode.is_array()) {
                for (const auto &tm: tmNode) {
                    if (!tm.is_object()) continue;

                    Camera::ToneMapSettings t;

                    if (tm.contains("TMO"))
                        t.tmo = tm["TMO"].get<string>();

                    if (tm.contains("TMOOptions")) {
                        const auto &opt = tm["TMOOptions"];
                        if (opt.is_string()) {
                            std::istringstream iss(opt.get<string>());
                            iss >> t.key >> t.burnOut;
                        }
                        else if (opt.is_array()) {
                            if (opt.size() >= 1) {
                                if (opt[0].is_string()) t.key = std::stof(opt[0].get<string>());
                                else t.key = opt[0].get<float>();
                            }
                            if (opt.size() >= 2) {
                                if (opt[1].is_string()) t.burnOut = std::stof(opt[1].get<string>());
                                else t.burnOut = opt[1].get<float>();
                            }
                        }
                    }

                    if (tm.contains("Saturation")) {
                        const auto &s = tm["Saturation"];
                        t.saturation = s.is_string() ? std::stof(s.get<string>()) : s.get<float>();
                    }

                    if (tm.contains("Gamma")) {
                        const auto &g = tm["Gamma"];
                        t.gamma = g.is_string() ? std::stof(g.get<string>()) : g.get<float>();
                    }

                    if (tm.contains("Extension"))
                        t.extension = tm["Extension"].get<string>();

                    c.tonemaps.push_back(t);
                }
            }
        }

        return c;
    };

    if (camNode.is_array()) {
        for (size_t i = 0; i < camNode.size(); ++i) {
            Camera c = parseOneCamera(camNode[i]);
            this->cameras.push_back(c);
        }
    }
    else if (camNode.is_object()) {
        this->cameras.push_back(parseOneCamera(camNode));
    }
    else {
        cerr << "Error: Cameras.Camera must be object or array." << std::endl;
        return false;
    }
#pragma endregion

#pragma region Lights
    if (doc["Scene"]["Lights"].contains("AmbientLight"))
        this->ambientLight = parseVec3(doc["Scene"]["Lights"]["AmbientLight"].get<string>());

    auto pointLightData = doc["Scene"]["Lights"]["PointLight"];

    if (pointLightData.is_array()) {
        for (const json &lightJson: pointLightData) {
            // Reference to avoid copying
            PointLight light;
            light.id = stoi(lightJson["_id"].get<string>());
            light.position = parseVec3(lightJson["Position"].get<string>());
            light.intensity = parseVec3(lightJson["Intensity"].get<string>());

            if (lightJson.contains("Transformations")) {
                std::istringstream iss(lightJson["Transformations"].get<std::string>());
                std::string token;
                while (iss >> token)
                    light.transformationOrder.push_back(token);
            }

            this->pointLights.emplace_back(light); // Upcasting PointLight to Light
        }
    } else if (pointLightData.is_object()) {
        PointLight light;
        light.id = stoi(pointLightData["_id"].get<string>());
        light.position = parseVec3(pointLightData["Position"].get<string>());
        light.intensity = parseVec3(pointLightData["Intensity"].get<string>());

        if (pointLightData.contains("Transformations")) {
            std::istringstream iss(pointLightData["Transformations"].get<std::string>());
            std::string token;
            while (iss >> token)
                light.transformationOrder.push_back(token);
        }

        this->pointLights.emplace_back(light);
    }

    if (doc["Scene"]["Lights"].contains("AreaLight")) {
        auto areaLightData = doc["Scene"]["Lights"]["AreaLight"];

        auto parseOneAreaLight = [&](const json &j) {
            AreaLight light;

            if (j["_id"].is_string())
                light.id = stoi(j["_id"].get<std::string>());
            else
                light.id = j["_id"].get<int>();

            light.position = parseVec3(j["Position"].get<std::string>());
            light.normal = parseVec3(j["Normal"].get<std::string>());

            if (j["Size"].is_string())
                light.size = std::stof(j["Size"].get<std::string>());
            else
                light.size = j["Size"].get<float>();

            light.radiance = parseVec3(j["Radiance"].get<std::string>());
            light.intensity = light.radiance;

            this->areaLights.emplace_back(light);
        };

        if (areaLightData.is_array()) {
            for (const json &j: areaLightData)
                parseOneAreaLight(j);
        } else if (areaLightData.is_object()) {
            parseOneAreaLight(areaLightData);
        }
    }

    if (doc["Scene"]["Lights"].contains("DirectionalLight")) {
        auto dirData = doc["Scene"]["Lights"]["DirectionalLight"];

        auto parseOneDir = [&](const json& j) {
            DirectionalLight L;

            if (j["_id"].is_string())
                L.id = stoi(j["_id"].get<string>());
            else
                L.id = j["_id"].get<int>();

            L.direction = parseVec3(j["Direction"].get<string>());
            L.radiance = parseVec3(j["Radiance"].get<string>());

            L.intensity = L.radiance;

            this->directionalLights.push_back(L);
        };

        if (dirData.is_array()) {
            for (const auto& j : dirData)
                parseOneDir(j);
        }
        else if (dirData.is_object()) {
            parseOneDir(dirData);
        }
    }

    if (doc["Scene"]["Lights"].contains("SpotLight")) {
        auto spotData = doc["Scene"]["Lights"]["SpotLight"];

        auto parseOneSpot = [&](const json& j) {
            SpotLight L;

            if (j["_id"].is_string())
                L.id = stoi(j["_id"].get<string>());
            else
                L.id = j["_id"].get<int>();

            L.position = parseVec3(j["Position"].get<string>());
            L.direction = parseVec3(j["Direction"].get<string>());

            L.intensity = parseVec3(j["Intensity"].get<string>());

            if (j["CoverageAngle"].is_string())
                L.coverageAngle = stof(j["CoverageAngle"].get<string>());
            else
                L.coverageAngle = j["CoverageAngle"].get<float>();

            if (j["FalloffAngle"].is_string())
                L.falloffAngle = stof(j["FalloffAngle"].get<string>());
            else
                L.falloffAngle = j["FalloffAngle"].get<float>();

            if (j.contains("Transformations")) {
                istringstream iss(j["Transformations"].get<string>());
                string token;
                while (iss >> token) L.transformationOrder.push_back(token);
            }

            this->spotLights.push_back(L);
        };

        if (spotData.is_array()) {
            for (const auto& j : spotData)
                parseOneSpot(j);
        } else if (spotData.is_object()) {
            parseOneSpot(spotData);
        }
    }

    if (doc["Scene"]["Lights"].contains("SphericalDirectionalLight")) {
        auto envData = doc["Scene"]["Lights"]["SphericalDirectionalLight"];

        auto parseOneEnv = [&](const json& j) {
            SphericalDirectionalLight L;

            if (j["_id"].is_string())
                L.id = stoi(j["_id"].get<string>());
            else
                L.id = j["_id"].get<int>();

            if (j.contains("_type")) {
                string t = j["_type"].get<string>();
                if (t == "probe") L.type = EnvMapType::Probe;
                else L.type = EnvMapType::LatLong;
            }

            if (j["ImageId"].is_string())
                L.imageId = stoi(j["ImageId"].get<string>());
            else
                L.imageId = j["ImageId"].get<int>();

            if (j.contains("Sampler")) {
                string s = j["Sampler"].get<string>();
                if (s == "uniform")
                    L.sampler = EnvSampler::Uniform;
                else
                    L.sampler = EnvSampler::Cosine; // default
            }
            else {
                L.sampler = EnvSampler::Cosine;
            }

            this->envLights.push_back(L);
        };

        if (envData.is_array()) {
            for (const auto& j : envData)
                parseOneEnv(j);
        }
        else if (envData.is_object()) {
            parseOneEnv(envData);
        }
    }


#pragma endregion

#pragma region Materials
    auto materialData = doc["Scene"]["Materials"]["Material"];
    auto parseMaterial = [&](const json &materialJson) {
        Material material;
        material.id = stoi(materialJson["_id"].get<string>());

        if (materialJson.contains("_type"))
            material.type = materialJson["_type"].get<string>();

        material.degamma = false; // Default value
        if (materialJson.contains("_degamma")) {
            material.degamma = parseBool(materialJson["_degamma"]);
        }

        material.ambientReflectance = parseVec3(materialJson["AmbientReflectance"].get<string>());
        material.diffuseReflectance = parseVec3(materialJson["DiffuseReflectance"].get<string>());
        material.specularReflectance = parseVec3(materialJson["SpecularReflectance"].get<string>());

        if (material.degamma) {
            material.ambientReflectance = srgbToLinear3(material.ambientReflectance);
            material.diffuseReflectance = srgbToLinear3(material.diffuseReflectance);
            material.specularReflectance = srgbToLinear3(material.specularReflectance);

            // TODO: If mirror is also given as sRGB, convert it too
            // material.mirrorReflectance = srgbToLinear3(material.mirrorReflectance);
        }

        // Optional fields with default values
        material.phongExponent = materialJson.contains("PhongExponent")
                                     ? stoi(materialJson["PhongExponent"].get<string>())
                                     : 0;
        material.refractionIndex = materialJson.contains("RefractionIndex")
                                       ? stof(materialJson["RefractionIndex"].get<string>())
                                       : 0.0f;
        material.absorptionIndex = materialJson.contains("AbsorptionIndex")
                                       ? stof(materialJson["AbsorptionIndex"].get<string>())
                                       : 0.0f;

        if (materialJson.contains("AbsorptionCoefficient"))
            material.absorptionCoefficient = parseVec3(materialJson["AbsorptionCoefficient"].get<string>());
        else
            material.absorptionCoefficient = Vec3(0.0f, 0.0f, 0.0f); // Default value

        if (materialJson.contains("MirrorReflectance"))
            material.mirrorReflectance = parseVec3(materialJson["MirrorReflectance"].get<string>());
        else
            material.mirrorReflectance = Vec3(0.0f, 0.0f, 0.0f); // Default value

        if (materialJson.contains("Roughness"))
            material.roughness = stof(materialJson["Roughness"].get<string>());
        else
            material.roughness = 0.0f; // Default value

        this->materials.push_back(material);
    };

    if (materialData.is_array()) {
        for (const json &materialJson: materialData) {
            parseMaterial(materialJson);
        }
    } else if (materialData.is_object()) {
        parseMaterial(materialData);
    }
#pragma endregion

#pragma region Transformations
    if (doc["Scene"].contains("Transformations")) {
        const auto &T = doc["Scene"]["Transformations"];

        // Scaling
        if (T.contains("Scaling")) {
            const auto &arr = T["Scaling"];
            if (arr.is_array()) {
                for (const auto &j: arr) {
                    int id = stoi(j["_id"].get<std::string>());
                    Vec3 s = parseVec3(j["_data"].get<std::string>());
                    this->transformations.push_back(Transformation::fromScaling(id, s));
                }
            } else if (arr.is_object()) {
                int id = stoi(arr["_id"].get<std::string>());
                Vec3 s = parseVec3(arr["_data"].get<std::string>());
                this->transformations.push_back(Transformation::fromScaling(id, s));
            }
        }

        // Translation
        if (T.contains("Translation")) {
            const auto &arr = T["Translation"];
            if (arr.is_array()) {
                for (const auto &j: arr) {
                    int id = stoi(j["_id"].get<std::string>());
                    Vec3 t = parseVec3(j["_data"].get<std::string>());
                    this->transformations.push_back(Transformation::fromTranslation(id, t));
                }
            } else if (arr.is_object()) {
                int id = stoi(arr["_id"].get<std::string>());
                Vec3 t = parseVec3(arr["_data"].get<std::string>());
                this->transformations.push_back(Transformation::fromTranslation(id, t));
            }
        }

        // Rotation
        if (T.contains("Rotation")) {
            const auto &node = T["Rotation"];
            if (node.is_array()) {
                for (const auto &j: node) {
                    int id = stoi(j["_id"].get<std::string>());
                    float angle;
                    Vec3 axis;
                    std::istringstream iss(j["_data"].get<std::string>());
                    iss >> angle >> axis.x >> axis.y >> axis.z;
                    this->transformations.push_back(Transformation::fromRotation(id, angle, axis));
                }
            } else if (node.is_object()) {
                int id = stoi(node["_id"].get<std::string>());
                float angle;
                Vec3 axis;
                std::istringstream iss(node["_data"].get<std::string>());
                iss >> angle >> axis.x >> axis.y >> axis.z;
                this->transformations.push_back(Transformation::fromRotation(id, angle, axis));
            }
        }

        // Composite
        if (T.contains("Composite")) {
            const auto &node = T["Composite"];

            auto parseComposite = [&](const json &j) {
                int id = stoi(j["_id"].get<string>());
                istringstream iss(j["_data"].get<string>());

                glm::mat4 mat(1.0f);
                float value;
                for (int row = 0; row < 4; ++row) {
                    for (int col = 0; col < 4; ++col) {
                        if ((iss >> value)) {
                            mat[col][row] = value;
                        }
                    }
                }

                this->transformations.push_back(Transformation::fromComposite(id, mat));
            };

            if (node.is_array()) {
                for (const auto &j: node)
                    parseComposite(j);
            } else if (node.is_object()) {
                parseComposite(node);
            }
        }
    }
#pragma endregion

#pragma region Vertex Data
    this->vertexData.clear();

    if (doc["Scene"].contains("VertexData")) {
        std::string vertexDataStr;

        if (doc["Scene"]["VertexData"].is_object()) {
            if (doc["Scene"]["VertexData"].contains("_data"))
                vertexDataStr = doc["Scene"]["VertexData"]["_data"].get<std::string>();
            else
                vertexDataStr = "";
        } else {
            vertexDataStr = doc["Scene"]["VertexData"].get<std::string>();
        }

        std::istringstream issVertex(vertexDataStr);
        float x, y, z;
        while (issVertex >> x >> y >> z) {
            this->vertexData.emplace_back(x, y, z);
        }
    } else {
        // No VertexData present (possible for procedural scenes)
    }
#pragma endregion

#pragma region TexCoordData
    if (doc["Scene"].contains("TexCoordData")) {
        std::string texStr;
        if (doc["Scene"]["TexCoordData"].is_object()) {
            texStr = doc["Scene"]["TexCoordData"]["_data"].get<std::string>();
        } else {
            texStr = doc["Scene"]["TexCoordData"].get<std::string>();
        }

        std::istringstream issTex(texStr);
        float u, v;
        while (issTex >> u >> v) {
            this->texCoordData.emplace_back(u, v);
        }
    }
#pragma endregion

#pragma region Textures
    if (doc["Scene"].contains("Textures")) {
        const auto &T = doc["Scene"]["Textures"];

        // Images
        if (T.contains("Images")) {
            const auto &imgsNode = T["Images"]["Image"];

            auto parseImage = [&](const json &j) {
                Image img;
                if (j["_id"].is_string()) img.id = std::stoi(j["_id"].get<std::string>());
                else img.id = j["_id"].get<int>();

                img.path = j["_data"].get<std::string>();
                this->images.push_back(img);
            };

            if (imgsNode.is_array()) {
                for (const auto &j: imgsNode)
                    parseImage(j);
            } else if (imgsNode.is_object()) {
                parseImage(imgsNode);
            }
        }

        // TextureMaps
        if (T.contains("TextureMap")) {
            const auto &texMaps = T["TextureMap"];

            auto parseDecalMode = [](const std::string &s) -> DecalMode {
                if (s == "replace_kd") return DecalMode::ReplaceKd;
                if (s == "blend_kd") return DecalMode::BlendKd;
                if (s == "replace_ks") return DecalMode::ReplaceKs;
                if (s == "replace_background") return DecalMode::ReplaceBackground;
                if (s == "replace_normal") return DecalMode::ReplaceNormal;
                if (s == "bump_normal") return DecalMode::BumpNormal;
                if (s == "replace_all") return DecalMode::ReplaceAll;
                return DecalMode::ReplaceKd; // Default value
            };

            auto parseNoiseConv = [](const std::string &s) -> NoiseConversion {
                if (s == "absval") return NoiseConversion::AbsVal;
                if (s == "linear") return NoiseConversion::Linear;
                return NoiseConversion::Linear; // Default value
            };

            auto parseInterp = [](const std::string &s) -> InterpolationMode {
                if (s == "bilinear") return InterpolationMode::Bilinear;
                return InterpolationMode::Nearest;
                // No trilinear for now
            };

            auto parseOneTextureMap = [&](const json &j) {
                std::string typeStr = j["_type"].get<std::string>();

                if (typeStr == "image") {
                    ImageTextureMap tex;
                    tex.type = TextureType::Image;

                    tex.id = j["_id"].is_string() ? std::stoi(j["_id"].get<std::string>()) : j["_id"].get<int>();

                    if (j.contains("_degamma")) tex.degamma = parseBool(j["_degamma"]);
                    else tex.degamma = false;

                    tex.decalMode = parseDecalMode(j["DecalMode"].get<std::string>());
                    tex.imageId = std::stoi(j["ImageId"].get<std::string>());

                    if (j.contains("BumpFactor"))
                        tex.bumpFactor = std::stof(j["BumpFactor"].get<std::string>());

                    if (tex.decalMode == DecalMode::ReplaceBackground) {
                        this->backgroundTextureMapId = tex.id; // TextureMap _id
                    }

                    if (j.contains("Interpolation")) {
                        tex.interpolation = parseInterp(j["Interpolation"].get<std::string>());
                    }

                    if (j.contains("Normalizer")) {
                        tex.normalizer = std::stof(j["Normalizer"].get<std::string>());
                    }

                    this->textures.imageTextures.push_back(tex);
                } else if (typeStr == "perlin") {
                    PerlinTextureMap tex;
                    tex.type = TextureType::Perlin;

                    tex.id = j["_id"].is_string() ? std::stoi(j["_id"].get<std::string>()) : j["_id"].get<int>();
                    tex.decalMode = parseDecalMode(j["DecalMode"].get<std::string>());

                    if (j.contains("NoiseScale")) tex.noiseScale = std::stof(j["NoiseScale"].get<std::string>());
                    if (j.contains("NoiseConversion")) tex.conversion = parseNoiseConv(
                                                           j["NoiseConversion"].get<std::string>());
                    if (j.contains("NumOctaves")) tex.numOctaves = std::stoi(j["NumOctaves"].get<std::string>());
                    if (j.contains("BumpFactor")) tex.bumpFactor = std::stof(j["BumpFactor"].get<std::string>());

                    if (tex.decalMode == DecalMode::ReplaceBackground) {
                        this->backgroundTextureMapId = tex.id;
                    }

                    this->textures.perlinTextures.push_back(tex);
                } else if (typeStr == "checkerboard") {
                    CheckerTextureMap tex;
                    tex.type = TextureType::Checkerboard;

                    tex.id = j["_id"].is_string() ? std::stoi(j["_id"].get<std::string>()) : j["_id"].get<int>();
                    tex.decalMode = parseDecalMode(j["DecalMode"].get<std::string>());

                    if (j.contains("Scale")) tex.scale = std::stof(j["Scale"].get<std::string>());
                    if (j.contains("Offset")) tex.offset = std::stof(j["Offset"].get<std::string>());
                    if (j.contains("BlackColor")) tex.blackColor = parseVec3(j["BlackColor"].get<std::string>());
                    if (j.contains("WhiteColor")) tex.whiteColor = parseVec3(j["WhiteColor"].get<std::string>());

                    if (tex.decalMode == DecalMode::ReplaceBackground) {
                        this->backgroundTextureMapId = tex.id;
                    }

                    this->textures.checkerTextures.push_back(tex);
                }
            };

            if (texMaps.is_array()) {
                for (const auto &j: texMaps) parseOneTextureMap(j);
            } else if (texMaps.is_object()) {
                parseOneTextureMap(texMaps);
            }
        }
    }
#pragma endregion

#pragma region Objects
    auto objectData = doc["Scene"]["Objects"];

    // Triangles
    if (objectData.contains("Triangle")) {
        auto triangleData = objectData["Triangle"];
        auto processTriangle = [&](const json &triangleJson) {
            Triangle triangle;
            triangle.id = stoi(triangleJson["_id"].get<string>());
            triangle.materialId = stoi(triangleJson["Material"].get<string>());

            std::istringstream issIndices(triangleJson["Indices"].get<string>());
            issIndices >> triangle.indices[0] >> triangle.indices[1] >> triangle.indices[2];
            parseTransformations(triangleJson, triangle.transformationOrder);
            parseTextureIds(triangleJson, triangle.textureMapIds);

            if (triangleJson.contains("MotionBlur"))
                triangle.motionBlur = parseVec3(triangleJson["MotionBlur"].get<string>());
            else
                triangle.motionBlur = Vec3(0.0f, 0.0f, 0.0f); // Default no motion blur

            this->objects.triangles.push_back(triangle);
        };

        if (triangleData.is_array()) {
            for (const json &triangleJson: triangleData) processTriangle(triangleJson);
        } else if (triangleData.is_object()) {
            processTriangle(triangleData);
        }
    }

    // Spheres
    if (objectData.contains("Sphere")) {
        auto sphereData = objectData["Sphere"];
        auto processSphere = [&](const json &sphereJson) {
            Sphere sphere;
            sphere.id = stoi(sphereJson["_id"].get<string>());
            sphere.materialId = stoi(sphereJson["Material"].get<string>());
            sphere.center = stoi(sphereJson["Center"].get<string>());
            sphere.radius = stof(sphereJson["Radius"].get<string>());
            parseTransformations(sphereJson, sphere.transformationOrder);
            parseTextureIds(sphereJson, sphere.textureMapIds);

            if (sphereJson.contains("MotionBlur"))
                sphere.motionBlur = parseVec3(sphereJson["MotionBlur"].get<string>());
            else
                sphere.motionBlur = Vec3(0.0f, 0.0f, 0.0f); // Default no motion blur

            this->objects.spheres.push_back(sphere);
        };

        if (sphereData.is_array()) {
            for (const json &sphereJson: sphereData) processSphere(sphereJson);
        } else if (sphereData.is_object()) {
            processSphere(sphereData);
        }
    }

    // Meshes
    if (objectData.contains("Mesh")) {
        auto meshData = objectData["Mesh"];
        auto processMesh = [&](const json &meshJson) {
            Mesh mesh;
            mesh.id = stoi(meshJson["_id"].get<string>());

            if (meshJson.contains("_shadingMode")) {
                if (meshJson["_shadingMode"].get<string>() == "smooth")
                    mesh.shadingMode = ShadingMode::Smooth;
                else
                    mesh.shadingMode = ShadingMode::Flat;
            } else {
                mesh.shadingMode = ShadingMode::Flat; // Default value
            }

            mesh.materialId = stoi(meshJson["Material"].get<string>());

            if (meshJson["Faces"].contains("_vertexOffset")) {
                mesh.vertexOffset = std::stoi(meshJson["Faces"]["_vertexOffset"].get<std::string>());
            }

            if (meshJson["Faces"].contains("_textureOffset")) {
                mesh.textureOffset = std::stoi(meshJson["Faces"]["_textureOffset"].get<std::string>());
            }

            if (meshJson["Faces"].contains("_data")) {
                std::stringstream issFaces(meshJson["Faces"]["_data"].get<std::string>());

                int vertexOffset = 0;
                if (meshJson["Faces"].contains("_vertexOffset")) {
                    // If there is a vertex offset, read it
                    vertexOffset = std::stoi(meshJson["Faces"]["_vertexOffset"].get<std::string>());
                }

                int idx;
                // Read face indices (if vertexOffset > 0, add it to each index)
                while (issFaces >> idx) {
                    if (vertexOffset > 0) {
                        mesh.data.push_back(idx + vertexOffset);
                    } else {
                        mesh.data.push_back(idx);
                    }
                }
            } else if (meshJson["Faces"].contains("_plyFile")) {
                // PLY file
                std::string plyPath = meshJson["Faces"]["_plyFile"].get<std::string>();

                std::vector<Vec3> vertices;
                std::vector<int> indices;
                std::vector<Vec2> plyUVs;

                if (readPLY(plyPath, vertices, indices, plyUVs)) {
                    size_t baseIndex = this->vertexData.size();

                    this->vertexData.insert(this->vertexData.end(), vertices.begin(), vertices.end());

                    if (!plyUVs.empty()) {
                        this->texCoordData.insert(this->texCoordData.end(), plyUVs.begin(), plyUVs.end());
                    } else {
                        for (size_t k = 0; k < vertices.size(); ++k)
                            this->texCoordData.emplace_back(0.0f, 0.0f);
                    }

                    for (int idx: indices)
                        mesh.data.push_back(static_cast<int>(baseIndex + idx));
                } else {
                    cerr << "Failed to read PLY file: " << plyPath << std::endl;
                }
            }

            // TODO: Can be replaced with parseTransformations
            if (meshJson.contains("Transformations")) {
                std::istringstream iss(meshJson["Transformations"].get<std::string>());
                std::string transformationOrder;
                while (iss >> transformationOrder)
                    mesh.transformationOrder.push_back(transformationOrder);
            }

            parseTextureIds(meshJson, mesh.textureMapIds);

            if (meshJson.contains("MotionBlur"))
                mesh.motionBlur = parseVec3(meshJson["MotionBlur"].get<string>());
            else
                mesh.motionBlur = Vec3(0.0f, 0.0f, 0.0f); // Default no motion blur

            this->objects.meshes.push_back(mesh);
        };

        if (meshData.is_array()) {
            for (const json &meshJson: meshData) processMesh(meshJson);
        } else if (meshData.is_object()) {
            processMesh(meshData);
        }
    }

    if (objectData.contains("MeshInstance")) {
        auto instanceData = objectData["MeshInstance"];

        auto processInstance = [&](const json &j) {
            MeshInstance inst;
            inst.id = stoi(j["_id"].get<std::string>());
            inst.baseMeshId = stoi(j["_baseMeshId"].get<std::string>());
            if (j.contains("Material"))
                inst.materialId = stoi(j["Material"].get<std::string>());
            else
                inst.materialId = -1; // Inherit from base mesh

            if (j.contains("_resetTransform"))
                inst.resetTransform = (j["_resetTransform"].get<std::string>() == "true");
            else
                inst.resetTransform = false;

            if (j.contains("MotionBlur"))
                inst.motionBlur = parseVec3(j["MotionBlur"].get<string>());
            else
                inst.motionBlur = Vec3(0.0f, 0.0f, 0.0f); // Default no motion blur

            if (j.contains("Transformations")) {
                istringstream iss(j["Transformations"].get<std::string>());
                string transformationOrder;
                while (iss >> transformationOrder)
                    inst.transformationOrder.push_back(transformationOrder);
            }

            parseTextureIds(j, inst.textureMapIds);

            this->objects.meshInstances.push_back(inst);
        };

        if (instanceData.is_array()) {
            for (const json &j: instanceData)
                processInstance(j);
        } else if (instanceData.is_object()) {
            processInstance(instanceData);
        }
    }

    // Planes
    if (objectData.contains("Plane")) {
        auto planeData = objectData["Plane"];
        auto processPlane = [&](const json &planeJson) {
            Plane plane;
            plane.id = stoi(planeJson["_id"].get<string>());
            plane.materialId = stoi(planeJson["Material"].get<string>());

            plane.point = stoi(planeJson["Point"].get<string>());

            plane.normal = parseVec3(planeJson["Normal"].get<string>());

            if (planeJson.contains("MotionBlur"))
                plane.motionBlur = parseVec3(planeJson["MotionBlur"].get<string>());
            else
                plane.motionBlur = Vec3(0.0f, 0.0f, 0.0f); // Default no motion blur

            parseTransformations(planeJson, plane.transformationOrder);
            parseTextureIds(planeJson, plane.textureMapIds);
            this->objects.planes.push_back(plane);
        };

        if (planeData.is_array()) {
            for (const json &planeJson: planeData) processPlane(planeJson);
        } else if (planeData.is_object()) {
            processPlane(planeData);
        }
    }
#pragma endregion

    return true;
}
