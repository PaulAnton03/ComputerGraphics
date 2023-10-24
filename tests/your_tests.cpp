// Put your includes here
#include "bvh.h"
#include "render.h"
#include "sampler.h"
#include "scene.h"
#include "shading.h"
#include <limits>

// Suppress warnings in third-party code.
#include <framework/disable_all_warnings.h>
DISABLE_WARNINGS_PUSH()
#include <catch2/catch_all.hpp>
#include <glm/glm.hpp>
DISABLE_WARNINGS_POP()

// In this file you can add your own unit tests using the Catch2 library.
// You can find the documentation of Catch2 at the following link:
// https://github.com/catchorg/Catch2/blob/devel/docs/assertions.md
//
// These tests are only to help you verify that your code is correct.
// You don't have to hand them in; we will not consider them when grading.
//

// Add your tests here, if you want :D
TEST_CASE("ElementWiseMinTest")
{
        glm::vec3 a = { 1, 2, 3 };
        glm::vec3 b = { 3, 2, 1 };
        glm::vec3 c = elementWiseMin(a, b);
        CHECK(c == glm::vec3(1, 2, 1));
        CHECK(c.x <= a.x);
        CHECK(c.x <= b.x);
        CHECK(c.y <= a.y);
        CHECK(c.y <= b.y);
        CHECK(c.z <= a.z);
        CHECK(c.z <= b.z);
}

TEST_CASE("ElementWiseMaxTest")
{
        glm::vec3 a = { 1, 2, 3 };
        glm::vec3 b = { 3, 2, 1 };
        glm::vec3 c = elementWiseMax(a, b);
        CHECK(c == glm::vec3(3, 2, 3));
        CHECK(c.x >= a.x);
        CHECK(c.x >= b.x);
        CHECK(c.y >= a.y);
        CHECK(c.y >= b.y);
        CHECK(c.z >= a.z);
        CHECK(c.z >= b.z);
}

TEST_CASE("MinOfThreeTest")
{
        glm::vec3 a = { 1, 2, 3 };
        glm::vec3 b = { 3, -1, 1 };
        glm::vec3 c = { 2, 1, 3 };
        glm::vec3 d = minOfThree(a, b, c);
        CHECK(d == glm::vec3(1, -1, 1));
        CHECK(d.x <= a.x);
        CHECK(d.x <= b.x);
        CHECK(d.x <= c.x);
        CHECK(d.y <= a.y);
        CHECK(d.y <= b.y);
        CHECK(d.y <= c.y);
        CHECK(d.z <= a.z);
        CHECK(d.z <= b.z);
        CHECK(d.z <= c.z);
}

TEST_CASE("MaxOFThreeTest")
{
        glm::vec3 a = { 1, 2, 3 };
        glm::vec3 b = { 3, 2, 1 };
        glm::vec3 c = { -1, 1, 3 };
        glm::vec3 d = maxOfThree(a, b, c);
        CHECK(d == glm::vec3(3, 2, 3));
        CHECK(d.x >= a.x);
        CHECK(d.x >= b.x);
        CHECK(d.x >= c.x);
        CHECK(d.y >= a.y);
        CHECK(d.y >= b.y);
        CHECK(d.y >= c.y);
        CHECK(d.z >= a.z);
        CHECK(d.z >= b.z);
        CHECK(d.z >= c.z);
}

// The below tests are not "good" unit tests. They don't actually test correctness.
// They simply exist for demonstrative purposes. As they interact with the interfaces
// (scene, bvh_interface, etc), they allow you to verify that you haven't broken
// our grading interface. They should compile without changes. If they do
// not compile, neither will our grading tests!
TEST_CASE("InterfaceTest")
{
    // Setup a RenderState object with some defaults
    Features features = {
        .enableShading = true,
        .enableAccelStructure = false, // BVH is not actually active r.n.
        .shadingModel = ShadingModel::Lambertian
    };
    Scene scene = loadScenePrebuilt(SceneType::CornellBox, DATA_DIR);
    BVH bvh(scene, features);
    RenderState state = { .scene = scene, .features = features, .bvh = bvh, .sampler = {} };

    SECTION("BVH generation")
    {
        // There's something in here?
        CHECK(!state.bvh.primitives().empty());
    }

    SECTION("BVH traversal")
    {
        Ray ray = { .origin = glm::vec3(0), .direction = glm::vec3(1) };
        HitInfo hitInfo;

        // Hit something?
        CHECK(state.bvh.intersect(state, ray, hitInfo));
        CHECK(ray.t != std::numeric_limits<float>::max());
    }

    SECTION("Hit shading")
    {
        Ray ray = { .origin = glm::vec3(0), .direction = glm::vec3(1) };
        HitInfo hitInfo;
        state.bvh.intersect(state, ray, hitInfo);

        // Shaded something?
        glm::vec3 Lo = computeShading(state, ray.direction, -ray.direction, glm::vec3(1), hitInfo);
        CHECK(glm::any(glm::notEqual(Lo, glm::vec3(0))));
    }
}
