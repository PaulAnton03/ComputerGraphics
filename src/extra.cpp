#include "extra.h"
#include "bvh.h"
#include "light.h"
#include "intersect.h"
#include "recursive.h"
#include "texture.h"
#include "shading.h"
#include <framework/trackball.h>

// TODO; Extra feature
// Given the same input as for `renderImage()`, instead render an image with your own implementation
// of Depth of Field. Here, you generate camera rays s.t. a focus point and a thin lens camera model
// are in play, allowing objects to be in and out of focus.
// This method is not unit-tested, but we do expect to find it **exactly here**, and we'd rather
// not go on a hunting expedition for your implementation, so please keep it here!
void renderImageWithDepthOfField(const Scene& scene, const BVHInterface& bvh, const Features& features, const Trackball& camera, Screen& screen)
{
    if (!features.extra.enableDepthOfField)
        return;

    float F = features.extra.focusDistance;
    if(features.extra.DOFCalculateFocusDistance)
        F = camera.distanceFromLookAt();


    glm::vec3 cPos = camera.position();
    auto cDir = glm::quat(camera.rotationEulerAngles());

#ifdef NDEBUG // Enable multi threading in Release mode
#pragma omp parallel for schedule(guided)
#endif

    for (int y = 0; y < screen.resolution().y; y++) {
        for (int x = 0; x != screen.resolution().x; x++) {
            // Assemble useful objects on a per-pixel basis; e.g. a per-thread sampler
            // Note; we seed the sampler for consistent behavior across frames
            RenderState state = {
                .scene = scene,
                .features = features,
                .bvh = bvh,
                .sampler = { static_cast<uint32_t>(screen.resolution().y * x + y) }
            };
            auto rays = generatePixelRays(state, camera, { x, y }, screen.resolution());
            std::vector<Ray> extraRays = {};
            for(Ray & ray : rays)
            {
                Sampler sampler = Sampler(static_cast<uint32_t>(screen.resolution().y * x + y));
                extraRays.push_back(ray);

                for(uint32_t i = 0; i < state.features.extra.DOFnumSamples - 1; i ++) {
                    glm::vec3 focusPoint = ray.origin + ray.direction * F;
                    glm::vec2 sample = sampler.next_2d();
                    sample.x = (sample.x - 0.5f) * (1.f/ (1/F + 1.f)) / state.features.extra.aperture;
                    sample.y *= glm::pi<float>();
                    glm::vec3 newOrigin = cPos + cDir * glm::vec3(sample.x * glm::cos(sample.y), sample.x * glm::sin(sample.y), 0.f);
                    extraRays.push_back({newOrigin, glm::normalize(focusPoint - newOrigin)});
                }
            }

            auto L = renderRays(state, extraRays);
            screen.setPixel(x, y, L);
        }
    }
}

// TODO; Extra feature
// Given the same input as for `renderImage()`, instead render an image with your own implementation
// of motion blur. Here, you integrate over a time domain, and not just the pixel's image domain,
// to give objects the appearance of "fast movement".
// This method is not unit-tested, but we do expect to find it **exactly here**, and we'd rather
// not go on a hunting expedition for your implementation, so please keep it here!
void renderImageWithMotionBlur(const Scene& scene, const BVHInterface& bvh, const Features& features, const Trackball& camera, Screen& screen)
{
    if (!features.extra.enableMotionBlur) {
        return;
    }
}

// Bloom helper functions
double factorial(int n)
{
    double result = 1.0;
    while (n > 1) {
        result *= (double)n;
        n--;
    }
    return result;
}

double partialFactorial(int n, int m)
{
    if (m <= 0)
        m = 1;
    if (n <= 0)
        n = 1;

    bool bottomHeavy = m > n;
    int low = bottomHeavy ? n : m;
    int high = bottomHeavy ? m : n;

    double result = 1.0;

    for (int i = low; i <= high; i++)
        result *= (double)i;

    result = (bottomHeavy) ? 1.0 / result : result;

    return result;
}

uint32_t getIndex(int x, int y, const int& width, const int& height)
{
    uint32_t yc = (y < 0) ? 0 : (y >= height) ? height - 1
                                              : y;
    uint32_t xc = (x < 0) ? 0 : (x >= width) ? width - 1
                                             : x;

    return yc * ((uint32_t)width) + xc;
}

// TODO; Extra feature
// Given a rendered image, compute and apply a bloom post-processing effect to increase bright areas.
// This method is not unit-tested, but we do expect to find it **exactly here**, and we'd rather
// not go on a hunting expedition for your implementation, so please keep it here!
void postprocessImageWithBloom(const Scene& scene, const Features& features, const Trackball& camera, Screen& image)
{
    if (!features.extra.enableBloomEffect)
        return;

    std::vector<glm::vec3>& pixels = image.pixels();
    std::vector<glm::vec3> thresholdPixels = {};

    thresholdPixels.reserve(pixels.size());
    for (glm::vec3 P : pixels) {
        if (P.x > features.extra.bloomThreshold || P.y > features.extra.bloomThreshold || P.z > features.extra.bloomThreshold)
            thresholdPixels.push_back(P - features.extra.bloomThreshold);
        else
            thresholdPixels.push_back(glm::vec3(0.f));
    }

    std::vector<double> filter = {};

    int n = (int)features.extra.filterSize;
    n = (n % 2 == 0) ? n : n + 1;

    // build filter
    double sum = 0.0;
    for (int i = 0; i <= n; i++) {
        double P = 1.0;
        if (n - i > i)
            P = partialFactorial(n, n - i) / factorial(i);
        else
            P = partialFactorial(n, i) / factorial(n - i);
        filter.emplace_back(P);
        sum += P;
    }
    for (double& P : filter)
        P /= sum;

    std::vector<glm::vec3> filteredPixels = { pixels.size(), glm::vec3(0.f) };

    // apply filter
    for (int y = 0; y < image.resolution().y; y++) {
        for (int x = 0; x < image.resolution().x; x++) {
            auto P = glm::vec3(0.f);
            for (int i = 0; i <= n; i++) {
                P += (float)filter[i] * thresholdPixels[getIndex(x + i - n / 2, y, image.resolution().x, image.resolution().y)];
            }
            filteredPixels[getIndex(x, y, image.resolution().x, image.resolution().y)] = P;
        }
    }

    for (int x = 0; x < image.resolution().x; x++) {
        for (int y = 0; y < image.resolution().y; y++) {
            auto P = glm::vec3(0.f);
            for (int i = 0; i <= n; i++) {
                P += (float)filter[i] * filteredPixels[getIndex(x, y + i - n / 2, image.resolution().x, image.resolution().y)];
            }
            pixels[getIndex(x, y, image.resolution().x, image.resolution().y)] = glm::clamp(pixels[getIndex(x, y, image.resolution().x, image.resolution().y)] + features.extra.bloomFactor * P, 0.f, 1.f);
        }
    }
}

// TODO; Extra feature
// Given a camera ray (or reflected camera ray) and an intersection, evaluates the contribution of a set of
// glossy reflective rays, recursively evaluating renderRay(..., depth + 1) along each ray, and adding the
// results times material.ks to the current intersection's hit color.
// - state;    the active scene, feature config, bvh, and sampler
// - ray;      camera ray
// - hitInfo;  intersection object
// - hitColor; current color at the current intersection, which this function modifies
// - rayDepth; current recursive ray depth
// This method is not unit-tested, but we do expect to find it **exactly here**, and we'd rather
// not go on a hunting expedition for your implementation, so please keep it here!
void renderRayGlossyComponent(RenderState& state, Ray ray, const HitInfo& hitInfo, glm::vec3& hitColor, int rayDepth)
{
    // Generate an initial specular ray, and base secondary glossies on this ray
    // auto numSamples = state.features.extra.numGlossySamples;
    // ...
    if (!state.features.extra.enableGlossyReflection) {
        return;
    }
    glm::vec3 intersectionPoint = ray.origin + ray.t * ray.direction;
    float s = hitInfo.material.shininess;
    glm::vec3 n = glm::normalize(hitInfo.normal);
    glm::vec3 d = glm::normalize(-ray.direction);
    glm::vec3 r = glm::normalize(2.0f * glm::dot(n, d) * n - d);
    uint32_t numSamples = state.features.extra.numGlossySamples;
    glm::vec3 u = glm::vec3(0);
    if (r.x == 0) {
        u = { 1, 0, 0 };
    } else if (r.y == 0) {
        u = { 0, 1, 0 };
    } else if (r.z == 0) {
        u = { 0, 0, 1 };
    } else {
        u = glm::normalize(glm::vec3 { r.y, -r.x, 0 });
    }
    glm::vec3 v = glm::normalize(glm::cross(r, u));
    float diskRadius = s / 64.0f;
    glm::vec3 ac = glm::vec3(0);
    for (uint32_t i = 0; i < numSamples; i++) {
        glm::vec2 sample = state.sampler.next_2d();
        glm::vec3 rs = glm::normalize(r + u * diskRadius * (2 * sample.x - 1) + v * diskRadius * (2 * sample.y - 1));
        if (glm::dot(n, rs) > 0.0f) {
            Ray glossyRay = Ray(intersectionPoint + FLT_EPSILON * rs, rs);
            ac += renderRay(state, glossyRay, rayDepth + 1);
        }
    }
    hitColor += ac * hitInfo.material.ks;
}

// TODO; Extra feature
// Given a camera ray (or reflected camera ray) that does not intersect the scene, evaluates the contribution
// along the ray, originating from an environment map. You will have to add support for environment textures
// to the Scene object, and provide a scene with the right data to supply this.
// - state; the active scene, feature config, bvh, and sampler
// - ray;   ray object
// This method is not unit-tested, but we do expect to find it **exactly here**, and we'd rather
// not go on a hunting expedition for your implementation, so please keep it here!
// Code for finding the intersection of a ray with a cube map is taken from:https://en.wikipedia.org/wiki/Cube_mapping
glm::vec3 sampleEnvironmentMap(RenderState& state, Ray ray)
{
    if (state.features.extra.enableEnvironmentMap) {
        const std::shared_ptr<Image>(&cubeMap)[6] = state.scene.cubeMap;
        glm::vec3 direction = glm::normalize(ray.direction);
        Ray intersect;
        intersect.direction = direction;
        AxisAlignedBox cube = AxisAlignedBox { glm::vec3(-1.0f), glm::vec3(1.0f) };
        intersectRayWithShape(cube, intersect);
        glm::vec3 intersectionPoint = intersect.origin + intersect.t * intersect.direction;
        float absX = std::fabs(intersectionPoint.x);
        float absY = std::fabs(intersectionPoint.y);
        float absZ = std::fabs(intersectionPoint.z);

        int isXPositive = intersectionPoint.x > 0 ? 1 : 0;
        int isYPositive = intersectionPoint.y > 0 ? 1 : 0;
        int isZPositive = intersectionPoint.z > 0 ? 1 : 0;

        float maxAxis, uc, vc;

        int index;
        float x = intersectionPoint.x;
        float y = intersectionPoint.y;
        float z = intersectionPoint.z;

        // POSITIVE X
        if (isXPositive && absX >= absY && absX >= absZ) {
            // u (0 to 1) goes from +z to -z
            // v (0 to 1) goes from -y to +y
            maxAxis = absX;
            uc = -z;
            vc = y;
            index = 0;
        }
        // NEGATIVE X
        if (!isXPositive && absX >= absY && absX >= absZ) {
            // u (0 to 1) goes from -z to +z
            // v (0 to 1) goes from -y to +y
            maxAxis = absX;
            uc = z;
            vc = y;
            index = 1;
        }
        // POSITIVE Y
        if (isYPositive && absY >= absX && absY >= absZ) {
            // u (0 to 1) goes from -x to +x
            // v (0 to 1) goes from +z to -z
            maxAxis = absY;
            uc = x;
            vc = -z;
            index = 2;
        }
        // NEGATIVE Y
        if (!isYPositive && absY >= absX && absY >= absZ) {
            // u (0 to 1) goes from -x to +x
            // v (0 to 1) goes from -z to +z
            maxAxis = absY;
            uc = x;
            vc = z;
            index = 3;
        }
        // POSITIVE Z
        if (isZPositive && absZ >= absX && absZ >= absY) {
            // u (0 to 1) goes from -x to +x
            // v (0 to 1) goes from -y to +y
            maxAxis = absZ;
            uc = x;
            vc = y;
            index = 4;
        }
        // NEGATIVE Z
        if (!isZPositive && absZ >= absX && absZ >= absY) {
            // u (0 to 1) goes from +x to -x
            // v (0 to 1) goes from -y to +y
            maxAxis = absZ;
            uc = -x;
            vc = y;
            index = 5;
        }
        // Convert range from -1 to 1 to 0 to 1
        float u = 0.5f * (uc / maxAxis + 1.0f);
        float v = 0.5f * (vc / maxAxis + 1.0f);
        return sampleTextureNearest(*cubeMap[index], glm::vec2(u, v));
    } else {
        return glm::vec3(0.f);
    }
}

// TODO: Extra feature
// As an alternative to `splitPrimitivesByMedian`, use a SAH+binning splitting criterion. Refer to
// the `Data Structures` lecture for details on this metric.
// - aabb;       the axis-aligned bounding box around the given triangle set
// - axis;       0, 1, or 2, determining on which axis (x, y, or z) the split must happen
// - primitives; the modifiable range of triangles that requires splitting
// - return;     the split position of the modified range of triangles
// This method is unit-tested, so do not change the function signature.
size_t splitPrimitivesBySAHBin(const AxisAlignedBox& aabb, uint32_t axis, std::span<BVH::Primitive> primitives)
{
    using Primitive = BVH::Primitive;

    return 0; // This is clearly not the solution
}