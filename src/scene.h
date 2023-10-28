#pragma once
// Suppress warnings in third-party code.
#include <framework/disable_all_warnings.h>
DISABLE_WARNINGS_PUSH()
#include <glm/vec3.hpp>
DISABLE_WARNINGS_POP()
#include <filesystem>
#include <framework/mesh.h>
#include <framework/ray.h>
#include <optional>
#include <variant>
#include <vector>
#include "common.h"

enum SceneType {
    SingleTriangle,
    Cube,
    CubeTextured,
    CornellBox,
    CornellBoxTransparency,
    CornellBoxParallelogramLight,
    Monkey,
    Teapot,
    Dragon,
    Spheres,
    Custom,
};

struct Scene {
    using SceneLight = std::variant<PointLight, SegmentLight, ParallelogramLight>;

    SceneType type;
    std::vector<Mesh> meshes;
    std::vector<Sphere> spheres;
    std::vector<SceneLight> lights;

    // You can add your own objects (e.g. environment maps) here
    // ...
    std::shared_ptr<Image> cubeMap[6];

    std::filesystem::path dataDir = DATA_DIR "CubeMap";

    Scene()
    {
        cubeMap[0] = std::make_shared<Image>(dataDir / "posx.jpg");
        cubeMap[1] = std::make_shared<Image>(dataDir / "negx.jpg");
        cubeMap[2] = std::make_shared<Image>(dataDir / "posy.jpg");
        cubeMap[3] = std::make_shared<Image>(dataDir / "negy.jpg");
        cubeMap[4] = std::make_shared<Image>(dataDir / "posz.jpg");
        cubeMap[5] = std::make_shared<Image>(dataDir / "negz.jpg");
   
    }
};

// Load a prebuilt scene.
Scene loadScenePrebuilt(SceneType type, const std::filesystem::path& dataDir);

// Load a scene from a file.
Scene loadSceneFromFile(const std::filesystem::path& path, const std::vector<std::variant<PointLight, SegmentLight, ParallelogramLight>>& lights);
