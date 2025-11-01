#pragma once
#include <vector>
#include "Vec3.h"

using namespace std;

class Object {
public:
    int id;
    int materialId;
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
    vector<float> data; // or int?
    ShadingMode shadingMode;
    vector<Vec3> perVertexNormal; // for smooth shading
    //  Also contains _type: "triangle"
};

class Objects {
public:
    vector<Triangle> triangles;
    vector<Sphere> spheres;
    vector<Mesh> meshes;
    vector<Plane> planes;
};
