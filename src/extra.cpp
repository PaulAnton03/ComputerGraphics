#include "extra.h"
#include "bvh.h"
#include "light.h"
#include "recursive.h"
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
    if (!features.extra.enableDepthOfField) {
        return;
    }

    // ...
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

//Bloom helper functions
double factorial(int n)
{
    double result = 1.0;
    while (n > 1)
    {
        result *= (double) n;
        n--;
    }
    return result;
}

double partialFactorial(int n, int m)
{
    if(m <= 0)
        m = 1;
    if(n <= 0)
        n = 1;

    bool bottomHeavy = m > n;
    int low = bottomHeavy ? n : m;
    int high = bottomHeavy ? m : n;

    double result = 1.0;

    for(int i = low; i <= high; i++)
        result *= (double) i;

    result = (bottomHeavy) ? 1.0 / result : result;

    return result;

}

uint32_t getIndex(int x, int y, const int & width, const int & height)
{
    uint32_t yc = (y < 0) ? 0 : (y >= height) ? height - 1 : y;
    uint32_t xc = (x < 0) ? 0 : (x >= width) ? width - 1 : x;

    return yc * ((uint32_t) width) + xc;
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
    for(glm::vec3  P : pixels)
    {
        if(P.x > features.extra.bloomThreshold || P.y > features.extra.bloomThreshold || P.z > features.extra.bloomThreshold)
            thresholdPixels.push_back(P - features.extra.bloomThreshold);
        else
            thresholdPixels.push_back(glm::vec3(0.f));
    }

    std::vector<double> filter = {};

    int n = (int) features.extra.filterSize;
    n = (n % 2 == 0) ? n : n + 1;

    //build filter
    double sum = 0.0;
    for (int i = 0; i <= n; i++) {
        double P = 1.0;
        if(n - i > i)
            P = partialFactorial(n, n - i) / factorial(i);
        else
            P = partialFactorial(n, i) / factorial(n - i);
        filter.emplace_back(P);
        sum += P;
    }
    for (double & P : filter)
        P /= sum;

    std::vector<glm::vec3> filteredPixels = {pixels.size(), glm::vec3(0.f)};

    //apply filter
    for(int y = 0; y < image.resolution().y; y++) {
        for(int x = 0; x < image.resolution().x; x++) {
            auto P = glm::vec3(0.f);
            for (int i = 0; i <= n; i++) {
                P += (float) filter[i] * thresholdPixels[getIndex(x + i - n/2, y, image.resolution().x, image.resolution().y)];
            }
            filteredPixels [getIndex(x, y, image.resolution().x, image.resolution().y)] = P;
        }
    }

    for(int x = 0; x < image.resolution().x; x++) {
        for(int y = 0; y < image.resolution().y; y++) {
            auto P = glm::vec3(0.f);
            for (int i = 0; i <= n; i++) {
                P += (float) filter[i] * filteredPixels[getIndex(x, y + i - n/2, image.resolution().x, image.resolution().y)];
            }
            pixels [getIndex(x, y, image.resolution().x, image.resolution().y)] = glm::clamp(pixels [getIndex(x, y, image.resolution().x, image.resolution().y)] + features.extra.bloomFactor *  P, 0.f, 1.f);
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
}

// TODO; Extra feature
// Given a camera ray (or reflected camera ray) that does not intersect the scene, evaluates the contribution
// along the ray, originating from an environment map. You will have to add support for environment textures
// to the Scene object, and provide a scene with the right data to supply this.
// - state; the active scene, feature config, bvh, and sampler
// - ray;   ray object
// This method is not unit-tested, but we do expect to find it **exactly here**, and we'd rather
// not go on a hunting expedition for your implementation, so please keep it here!
glm::vec3 sampleEnvironmentMap(RenderState& state, Ray ray)
{
    if (state.features.extra.enableEnvironmentMap) {
        // Part of your implementation should go here
        return glm::vec3(0.f);
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