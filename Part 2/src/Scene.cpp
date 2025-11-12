#include <iostream>
#include <fstream>
#include "../include/Scene.h"
#include "../include/Transformation.h"

#include "../ply/happly.h"

using json = nlohmann::json;
using namespace std;

static Vec3 parseVec3(const string& str) {
    istringstream iss(str);
    Vec3 vec;
    // >> automatically casts to float
    if (!(iss >> vec.x >> vec.y >> vec.z)) { // Error handling
        return Vec3(0.0f, 0.0f, 0.0f);
    }
    return vec;
}

static bool readPLY(const std::string &filename, std::vector<Vec3> &outVertices, std::vector<int> &outIndices) {
   try{
       happly::PLYData plyIn(filename);

       // Vertex positions
       std::vector<float> x = plyIn.getElement("vertex").getProperty<float>("x");
       std::vector<float> y = plyIn.getElement("vertex").getProperty<float>("y");
       std::vector<float> z = plyIn.getElement("vertex").getProperty<float>("z");

       outVertices.reserve(x.size());
       for (size_t i = 0; i < x.size(); ++i)
           outVertices.emplace_back(x[i], y[i], z[i]);

       // Faces (triangles)
       std::vector<std::vector<int>> faces = plyIn.getElement("face").getListProperty<int>("vertex_indices");

       for (const auto &f : faces) {
           if (f.size() == 3) {
               outIndices.push_back(f[0] + 1);
               outIndices.push_back(f[1] + 1);
               outIndices.push_back(f[2] + 1);
           } else if (f.size() == 4) {
               outIndices.push_back(f[0] + 1);
               outIndices.push_back(f[1] + 1);
               outIndices.push_back(f[2] + 1);
               outIndices.push_back(f[0] + 1);
               outIndices.push_back(f[2] + 1);
               outIndices.push_back(f[3] + 1);
           }
       }

       return true;
   }
   catch (const std::exception &e) {
       return false;
   }
}

static void parseTransformations(const nlohmann::json& j, std::vector<std::string>& out) {
    if (!j.contains("Transformations")) return;
    std::istringstream iss(j["Transformations"].get<std::string>());
    std::string token;
    while (iss >> token)
        out.push_back(token);
}

bool Scene::loadScene(const string& filePath) {
#pragma region File Read - Error Handling
    ifstream file(filePath);
    if (!file.is_open()) {
        cerr << "Error: File could not be opened. " << filePath << endl;
        return false;
    }

    json doc;
    try {
        file >> doc;
    } catch (const json::parse_error& e) {
        cerr << "Error: JSON Parse Error: " << e.what() << endl;
        return false;
    }
#pragma endregion

#pragma region Scene Parameters
    if (doc["Scene"].contains("MaxRecursionDepth"))
        this->maxRecursionDepth = stoi(doc["Scene"]["MaxRecursionDepth"].get<string>());
    else
        this->maxRecursionDepth = 5; // Default value

    this->backgroundColor = parseVec3(doc["Scene"]["BackgroundColor"].get<string>());

    if (doc["Scene"].contains("ShadowRayEpsilon"))
        this->shadowRayEpsilon = stof(doc["Scene"]["ShadowRayEpsilon"].get<string>());
    else
        this->shadowRayEpsilon = 0.001f; // Default value

    if (doc["Scene"].contains("IntersectionTestEpsilon"))
        this->intersectionTestEpsilon = stof(doc["Scene"]["IntersectionTestEpsilon"].get<string>());
    else
        this->intersectionTestEpsilon = 0.001f; // Default value
#pragma endregion

#pragma region Camera
    Camera camera;
    auto camNode  = doc["Scene"]["Cameras"]["Camera"];

    auto parseOneCamera = [&](const json& j) -> Camera {
        Camera c;

        if (j["_id"].is_string()) c.id = stoi(j["_id"].get<std::string>());
        else                      c.id = j["_id"].get<int>();

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
        } else { // If NearPlane is not provided, use default values
            c.nearPlane[0] = -1.0f; c.nearPlane[1] = 1.0f;
            c.nearPlane[2] = -1.0f; c.nearPlane[3] = 1.0f;
        }

        c.nearDistance = std::stof(j["NearDistance"].get<std::string>());

        std::istringstream issRes(j["ImageResolution"].get<std::string>());
        issRes >> c.imageResolution.x >> c.imageResolution.y;

        c.imageName = j["ImageName"].get<std::string>();

        if (j.contains("Transformations")) {
            std::istringstream iss(j["Transformations"].get<std::string>());
            std::string token;
            while (iss >> token)
                c.transformationOrder.push_back(token);
        }

        return c;
    };

    if (camNode.is_array()) {
        for (size_t i = 0; i < camNode.size(); ++i) {
            Camera c = parseOneCamera(camNode[i]);
            this->cameras.push_back(c);
        }
    } else if (camNode.is_object()) {
        this->cameras.push_back(parseOneCamera(camNode));
    } else {
        cerr << "Error: Cameras.Camera must be object or array." << std::endl;
        return false;
    }
#pragma endregion

#pragma region Lights
    this->ambientLight = parseVec3(doc["Scene"]["Lights"]["AmbientLight"].get<string>());

    auto pointLightData = doc["Scene"]["Lights"]["PointLight"];
    if (pointLightData.is_array()) {
        for (const json &lightJson: pointLightData) { // Reference to avoid copying
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

    //TODO: There may be other light types
#pragma endregion

#pragma region Materials
    auto materialData = doc["Scene"]["Materials"]["Material"];
    auto parseMaterial = [&](const json& materialJson) {
        Material material;
        material.id = stoi(materialJson["_id"].get<string>());

        if (materialJson.contains("_type"))
            material.type = materialJson["_type"].get<string>();

        material.ambientReflectance = parseVec3(materialJson["AmbientReflectance"].get<string>());
        material.diffuseReflectance = parseVec3(materialJson["DiffuseReflectance"].get<string>());
        material.specularReflectance = parseVec3(materialJson["SpecularReflectance"].get<string>());

        // Optional fields with default values
        material.phongExponent = materialJson.contains("PhongExponent") ? stoi(materialJson["PhongExponent"].get<string>()) : 0;
        material.refractionIndex = materialJson.contains("RefractionIndex") ? stof(materialJson["RefractionIndex"].get<string>()) : 0.0f;
        material.absorptionIndex = materialJson.contains("AbsorptionIndex") ? stof(materialJson["AbsorptionIndex"].get<string>()) : 0.0f;

        if (materialJson.contains("AbsorptionCoefficient"))
            material.absorptionCoefficient = parseVec3(materialJson["AbsorptionCoefficient"].get<string>());
        else
            material.absorptionCoefficient = Vec3(0.0f, 0.0f, 0.0f); // Default value

        if (materialJson.contains("MirrorReflectance"))
            material.mirrorReflectance = parseVec3(materialJson["MirrorReflectance"].get<string>());
        else
            material.mirrorReflectance = Vec3(0.0f, 0.0f, 0.0f); // Default value

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
        const auto& T = doc["Scene"]["Transformations"];

        // Scaling
        if (T.contains("Scaling")) {
            const auto& arr = T["Scaling"];
            if (arr.is_array()) {
                for (const auto& j : arr) {
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
            const auto& arr = T["Translation"];
            if (arr.is_array()) {
                for (const auto& j : arr) {
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
            const auto& node = T["Rotation"];
            if (node.is_array()) {
                for (const auto& j : node) {
                    int id = stoi(j["_id"].get<std::string>());
                    float angle; Vec3 axis;
                    std::istringstream iss(j["_data"].get<std::string>());
                    iss >> angle >> axis.x >> axis.y >> axis.z;
                    this->transformations.push_back(Transformation::fromRotation(id, angle, axis));
                }
            } else if (node.is_object()) {
                int id = stoi(node["_id"].get<std::string>());
                float angle; Vec3 axis;
                std::istringstream iss(node["_data"].get<std::string>());
                iss >> angle >> axis.x >> axis.y >> axis.z;
                this->transformations.push_back(Transformation::fromRotation(id, angle, axis));
            }
        }

        // Composite
        if (T.contains("Composite")) {
            const auto& node = T["Composite"];

            auto parseComposite = [&](const json& j) {
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
                for (const auto& j : node)
                    parseComposite(j);
            }
            else if (node.is_object()) {
                parseComposite(node);
            }
        }

    }
#pragma endregion

#pragma region Vertex Data
    string vertexDataStr;
    if (doc["Scene"]["VertexData"].is_object()){
        vertexDataStr = doc["Scene"]["VertexData"]["_data"].get<string>();
    } else {
        vertexDataStr = doc["Scene"]["VertexData"].get<string>();
    }

    istringstream issVertex(vertexDataStr);
    float x, y, z;
    while (issVertex >> x >> y >> z) {
        this->vertexData.emplace_back(x, y, z); // Using emplace_back to construct in place (instead of push_back which copies)
    }
#pragma endregion

#pragma region Objects
    auto objectData = doc["Scene"]["Objects"];

    // Triangles
    if(objectData.contains("Triangle")){
        auto triangleData = objectData["Triangle"];
        auto processTriangle = [&](const json &triangleJson){
            Triangle triangle;
            triangle.id = stoi(triangleJson["_id"].get<string>());
            triangle.materialId = stoi(triangleJson["Material"].get<string>());

            std::istringstream issIndices(triangleJson["Indices"].get<string>());
            issIndices >> triangle.indices[0] >> triangle.indices[1] >> triangle.indices[2];
            parseTransformations(triangleJson, triangle.transformationOrder);
            this->objects.triangles.push_back(triangle);
        };

        if (triangleData.is_array()){
            for (const json &triangleJson: triangleData) processTriangle(triangleJson);
        } else if (triangleData.is_object()) {
            processTriangle(triangleData);
        }
    }

    // Spheres
    if(objectData.contains("Sphere")){
        auto sphereData = objectData["Sphere"];
        auto processSphere = [&](const json &sphereJson){
            Sphere sphere;
            sphere.id = stoi(sphereJson["_id"].get<string>());
            sphere.materialId = stoi(sphereJson["Material"].get<string>());
            sphere.center = stoi(sphereJson["Center"].get<string>());
            sphere.radius = stof(sphereJson["Radius"].get<string>());
            parseTransformations(sphereJson, sphere.transformationOrder);
            this->objects.spheres.push_back(sphere);
        };

        if (sphereData.is_array()){
            for (const json &sphereJson: sphereData) processSphere(sphereJson);
        } else if (sphereData.is_object()) {
            processSphere(sphereData);
        }
    }

    // Meshes
    if(objectData.contains("Mesh")){
        auto meshData = objectData["Mesh"];
        auto processMesh = [&](const json &meshJson){
            Mesh mesh;
            mesh.id = stoi(meshJson["_id"].get<string>());

            if (meshJson.contains("_shadingMode")){
                if(meshJson["_shadingMode"].get<string>() == "smooth")
                    mesh.shadingMode = ShadingMode::Smooth;
                else
                    mesh.shadingMode = ShadingMode::Flat;
            }
            else {
                mesh.shadingMode = ShadingMode::Flat; // Default value
            }

            mesh.materialId = stoi(meshJson["Material"].get<string>());

            if (meshJson["Faces"].contains("_data")) {
                std::stringstream issFaces(meshJson["Faces"]["_data"].get<std::string>());
                int idx;
                while (issFaces >> idx)
                    mesh.data.push_back(idx);
            }
            else if (meshJson["Faces"].contains("_plyFile")) {
                // PLY file
                std::string plyPath = meshJson["Faces"]["_plyFile"].get<std::string>();

                std::vector<Vec3> vertices;
                std::vector<int> indices;

                if (readPLY(plyPath, vertices, indices)) {
                    size_t baseIndex = this->vertexData.size();
                    this->vertexData.insert(this->vertexData.end(), vertices.begin(), vertices.end());
                    for (int idx : indices)
                        mesh.data.push_back(static_cast<int>(baseIndex + idx));
                } else {
                    cerr << "Failed to read PLY file: " << plyPath << std::endl;
                }
            }

            if (meshJson.contains("Transformations")) {
                std::istringstream iss(meshJson["Transformations"].get<std::string>());
                std::string transformationOrder;
                while (iss >> transformationOrder)
                    mesh.transformationOrder.push_back(transformationOrder);
            }
            this->objects.meshes.push_back(mesh);
        };

        if (meshData.is_array()){
            for (const json &meshJson: meshData) processMesh(meshJson);
        } else if (meshData.is_object()) {
            processMesh(meshData);
        }
    }

    if (objectData.contains("MeshInstance")) {
        auto instanceData = objectData["MeshInstance"];

        auto processInstance = [&](const json& j) {
            MeshInstance inst;
            inst.id = stoi(j["_id"].get<std::string>());
            inst.baseMeshId = stoi(j["_baseMeshId"].get<std::string>());
            if (j.contains("Material"))
                inst.materialId = stoi(j["Material"].get<std::string>());
            else
                inst.materialId = -1; // means "inherit from base mesh"

            if (j.contains("_resetTransform"))
                inst.resetTransform = (j["_resetTransform"].get<std::string>() == "true");
            else
                inst.resetTransform = false;

            if (j.contains("Transformations")) {
                istringstream iss(j["Transformations"].get<std::string>());
                string transformationOrder;
                while (iss >> transformationOrder)
                    inst.transformationOrder.push_back(transformationOrder);
            }

            this->objects.meshInstances.push_back(inst);
        };

        if (instanceData.is_array()) {
            for (const json& j : instanceData)
                processInstance(j);
        } else if (instanceData.is_object()) {
            processInstance(instanceData);
        }
    }

    // Planes
    if(objectData.contains("Plane")){
        auto planeData = objectData["Plane"];
        auto processPlane = [&](const json &planeJson){
            Plane plane;
            plane.id = stoi(planeJson["_id"].get<string>());
            plane.materialId = stoi(planeJson["Material"].get<string>());

            plane.point = stoi(planeJson["Point"].get<string>());

            plane.normal = parseVec3(planeJson["Normal"].get<string>());

            parseTransformations(planeJson, plane.transformationOrder);
            this->objects.planes.push_back(plane);
        };

        if (planeData.is_array()){
            for (const json &planeJson: planeData) processPlane(planeJson);
        } else if (planeData.is_object()) {
            processPlane(planeData);
        }
    }
#pragma endregion

    return true;
}