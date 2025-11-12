# Ray Tracer

Development journey blog can be found [in my github.io page](https://fsaltunyuva.github.io/). To build and run the project, make sure you have a C++ compiler installed. Then, follow these steps:

```bash
 make
 ./raytracer [input_json_file]
```

Example input files are provided in the [`Scenes`](https://github.com/fsaltunyuva/RayTracer/tree/main/Scenes) directory. You can specify any of these files as the input to see different rendered scenes.

> [!WARNING]
> Rendering times may vary depending on the complexity of the scene and your hardware capabilities.

### Part 1 - Recursive Ray Tracing
This part implements basic ray tracing features, including:
- Ray Intersection Tests
- Blinn-Phong shading model
- Shadows
- Fresnel Reflections for Conductors, Dielectrics, and Simple Mirrors
- Beer's law for attenuation for dielectrics
- Point and Ambient light sources
- Back-face culling
- Degenerate triangle handling
- Shaded mesh rendering

This part's development blog can be found [here](https://fsaltunyuva.github.io/ray-tracing/graphics/adventure/2025/10/12/Ray-Tracing-Adventure.html).

### Part 1 - Acceleration Structures, Transformations, and Mesh Instances
- Transformations (translation, scaling, rotation)
- Mesh Instancing
- Bounding Volume Hierarchies (BVH)
- Look At Camera

This part's development blog can be found [here](https://fsaltunyuva.github.io/ray-tracing/graphics/adventure/2025/11/11/Ray-Tracing-Adventure.html).

> [!NOTE]
> Used libraries are: [json library](https://github.com/nlohmann/json?tab=readme-ov-file#license), [stb](https://github.com/nothings/stb), [Happly](https://github.com/nmwsharp/happly), [glm](https://github.com/g-truc/glm), and [FFmpeg](https://ffmpeg.org/).