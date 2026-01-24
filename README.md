# Ray Tracer

<table>
  <tr>
    <td align="center"><img src="https://github.com/user-attachments/assets/f5a1ca1a-90d0-45ac-8f46-0a26bf83c8fe" width="260"/><br/></td>
    <td align="center"><img src="https://github.com/user-attachments/assets/6534b866-01ce-4508-ba82-7480040c201a" width="260"/><br/></td>
    <td align="center"><img src="https://github.com/user-attachments/assets/e83f04af-b7da-4a57-9cdd-3b9918aa6647" width="260"/><br/></td>
  </tr>
  <tr>
    <td align="center"><img src="https://github.com/user-attachments/assets/f0f53686-fc76-4fc8-9190-f4aeb737f4b9" width="260"/><br/></td>
    <td align="center"><img src="https://github.com/user-attachments/assets/11e1058c-bfc5-4361-bc03-fdda2947380a" width="260"/><br/></td>
    <td align="center"><img src="https://github.com/user-attachments/assets/9f0a01f4-0620-479f-a2bd-8c742c1e967a" width="260"/><br/></td>
  </tr>
</table>

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

### Part 2 - Acceleration Structures, Transformations, and Mesh Instances
- Transformations (translation, scaling, rotation)
- Mesh Instancing
- Bounding Volume Hierarchies (BVH)
- Look At Camera

This part's development blog can be found [here](https://fsaltunyuva.github.io/ray-tracing/graphics/adventure/2025/11/11/Ray-Tracing-Adventure.html).

### Part 3 - Multisampling and Distribution Ray Tracing
- Multisampling
- Depth of Field
- Area Lights
- Motion Blur
- Material Roughness

This part's development blog can be found [here](https://fsaltunyuva.github.io/ray-tracing/graphics/adventure/2025/11/24/Ray-Tracing-Adventure.html).

### Part 4 - Texture Mapping and Procedural Textures
- Texture Mapping
- Normal and Bump Mapping
- Diffuse and Specular Maps
- Perlin Noise
- Checkerboard Patterns

This part's development blog can be found [here](https://fsaltunyuva.github.io/ray-tracing/graphics/adventure/2025/12/15/Ray-Tracing-Adventure.html).

### Part 5 - Advanced Lighting and HDR Rendering
- Tonemapping
- .exr and .hdr Images
- Directional Lights
- Spot Lights
- Environment Lights

This part's development blog can be found [here](https://fsaltunyuva.github.io/ray-tracing/graphics/adventure/2025/12/27/Ray-Tracing-Adventure.html).

### Part 6 - BRDFs and Path Tracing
- Bidirectional Reflectance Distribution Function (BRDF)
- Object Lights
- Path Tracing

This part's development blog can be found [here](https://fsaltunyuva.github.io/ray-tracing/graphics/adventure/2026/01/17/Ray-Tracing-Adventure.html).

### Foveated Rendering in Ray Tracing
- Foveated Rendering
- Falloff Method Comparisons

This part's development blog can be found [here](https://fsaltunyuva.github.io/ray-tracing/graphics/adventure/2026/01/20/Ray-Tracing-Adventure.html).

> [!NOTE]
> Used libraries are: [json library](https://github.com/nlohmann/json?tab=readme-ov-file#license), [stb](https://github.com/nothings/stb), [Happly](https://github.com/nmwsharp/happly), [glm](https://github.com/g-truc/glm), [FFmpeg](https://ffmpeg.org/), and [TinyEXR](https://github.com/syoyo/tinyexr). Also I used [this Python script](https://github.com/fsaltunyuva/FramesToVideo) to generate videos from renders.
