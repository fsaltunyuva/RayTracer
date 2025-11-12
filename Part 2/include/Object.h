#pragma once
#include <vector>
#include "Vec3.h"
#include "BVH.h"
#include "../glm/glm.hpp"

using namespace std;

class Object {
public:
    int id;
    int materialId;
    vector<string> transformationOrder; // Order of transformations (r1 t1 s2 r2 means first rotate with id 1, then translate with id 1, then scale with id 2, then rotate with id 2)
    glm::mat4 modelMatrix = glm::mat4(1.0f); // Identity matrix by default
};

class Triangle: public Object {
public:
    int indices[3];
};

class Sphere: public Object {
public:
    int center; // linked with vertex data
    float radius;
};

class Plane: public Object {
public:
    int point; // linked with vertex data
    Vec3 normal;
};

enum class ShadingMode { Flat, Smooth };

class Mesh: public Object {
public:
    vector<int> data; // or int?
    ShadingMode shadingMode;
    vector<Vec3> perVertexNormal; // for smooth shading
    BVH* bvh = nullptr; // pointer to BVH
    //  Also contains _type: "triangle"
};

class MeshInstance : public Object {
public:
    int baseMeshId;
    bool resetTransform;
    vector<Vec3> perVertexNormal; // for smooth shading
    BVH* bvh = nullptr;
    const Mesh* resolvedBaseMesh = nullptr; // pointer to base mesh
    //  Also contains _type: "triangle"
};

class Objects {
public:
    vector<Triangle> triangles;
    vector<Sphere> spheres;
    vector<Mesh> meshes;
    vector<Plane> planes;
    vector<MeshInstance> meshInstances;
};
