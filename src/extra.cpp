#include "extra.h"
#include "bvh.h"
#include "light.h"
#include "intersect.h"
#include "recursive.h"
#include "texture.h"
#include "shading.h"
#include <framework/trackball.h>
#include "draw.h"
#include <vector>

// DONE; Extra feature
// Given the same input as for `renderImage()`, instead render an image with your own implementation
// of Depth of Field. Here, you generate camera rays s.t. a focus point and a thin lens camera model
// are in play, allowing objects to be in and out of focus.
// This method is not unit-tested, but we do expect to find it **exactly here**, and we'd rather
// not go on a hunting expedition for your implementation, so please keep it here!


//DOF helper function
float calculateFocusDistance(const Scene& scene, const BVHInterface& bvh, const Features& features, const Trackball& camera)
{
    Ray ray = camera.generateRay({0.f, 0.f});
    RenderState state = {
        .scene = scene,
        .features = features,
        .bvh = bvh,
        .sampler = { static_cast<uint32_t>(0)}
    };
    ray.t = std::numeric_limits<float>::max();
    HitInfo info;
    if(bvh.intersect(state, ray, info))
        return ray.t;
    return features.extra.focusDistance;
}

Ray generateSecondaryRay(const Ray & ray, const float &  focusDistance, Sampler &  sampler, const RenderState &  state, const glm::vec3 &  cameraPos, const glm::qua<float> &  cameraDir)
{
    glm::vec3 focusPoint = ray.origin + ray.direction * focusDistance;
    glm::vec2 sample = sampler.next_2d();
    sample.x = (sample.x - 0.5f) * (1.f/ (1/ focusDistance + 1.f)) / state.features.extra.aperture;
    sample.y *= glm::pi<float>();
    glm::vec3 newOrigin = cameraPos + cameraDir * glm::vec3(sample.x * glm::cos(sample.y), sample.x * glm::sin(sample.y), 0.f);
    return {newOrigin, glm::normalize(focusPoint - newOrigin)};
}

//Show autofocus focus point in debug mode
void DOFDebugDrawFocusPoint(const Scene& scene, const BVHInterface& bvh, const Features& features, const Trackball& camera)
{
    float focusDistance = features.extra.focusDistance;
    if(features.extra.AutoFocus)
        focusDistance = calculateFocusDistance(scene, bvh, features, camera);

    Ray ray = camera.generateRay({0.f, 0.f});
    ray.t = focusDistance;
    drawSphere(glm::vec3(ray.origin + ray.direction * ray.t), 0.015f, glm::vec3(1.f, 1.f, 1.f));
}

void renderImageWithDepthOfField(const Scene& scene, const BVHInterface& bvh, const Features& features, const Trackball& camera, Screen& screen)
{
    if (!features.extra.enableDepthOfField)
        return;

    float focusDistance = features.extra.focusDistance;
    if(features.extra.AutoFocus)
        focusDistance = calculateFocusDistance(scene, bvh, features, camera);

    glm::vec3 cameraPos = camera.position();
    auto cameraDir = glm::quat(camera.rotationEulerAngles());

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
            std::vector<Ray> secondaryRays = {};

            for(Ray & ray : rays)
            {
                Sampler sampler = Sampler(static_cast<uint32_t>(screen.resolution().y * x + y));
//                secondaryRays.push_back(ray);

                for(uint32_t i = 0; i < state.features.extra.DOFSamples; i ++)
                    secondaryRays.emplace_back(generateSecondaryRay(ray, focusDistance, sampler, state, cameraPos, cameraDir));
            }

            auto L = renderRays(state, secondaryRays);
            screen.setPixel(x, y, L);
        }
    }
}

Scene updateScene(const Scene& scene, const Features& features)
{
    Scene scene2 = scene;
    for (Mesh& m : scene2.meshes) {
        m.p1 = { features.extra.bezierOffset1x, features.extra.bezierOffset1y, features.extra.bezierOffset1z };
        m.p2 = { features.extra.bezierOffset2x, features.extra.bezierOffset2y, features.extra.bezierOffset2z };
        m.moveable = true;
    }
    for (Sphere& s : scene2.spheres) {
        s.p1 = { features.extra.bezierOffset1x, features.extra.bezierOffset1y, features.extra.bezierOffset1z };
        s.p2 = { features.extra.bezierOffset2x, features.extra.bezierOffset2y, features.extra.bezierOffset2z };
        s.moveable = true;
    }
    return scene2;
}

void drawMovementLine(glm::vec3 p, const Features& features) {
    std::vector<glm::vec3> points = {};
    glm::vec3 p1 = { features.extra.bezierOffset1x, features.extra.bezierOffset1y, features.extra.bezierOffset1z };
    glm::vec3 p2 = { features.extra.bezierOffset2x, features.extra.bezierOffset2y, features.extra.bezierOffset2z };
    for (float t = 0; t < 1.f; t += 0.001f) {
        points.push_back(((1.f - t) * (1.f - t)) * p + 2 * t * (1 - t) * (p + p1) + t * t * (p + p2));
    }
    drawLine(points);
}

void drawMotionMeshAtTime(Scene scene, const Features& features)
{
    const double period = 5.0;
    auto currentTime = std::chrono::high_resolution_clock::now();
    auto duration = currentTime.time_since_epoch();
    double seconds = std::chrono::duration_cast<std::chrono::duration<double>>(duration).count();
    float t = (float)fmod(seconds / period, 1.0);
    glm::vec3 p1 = { features.extra.bezierOffset1x, features.extra.bezierOffset1y, features.extra.bezierOffset1z };
    glm::vec3 p2 = { features.extra.bezierOffset2x, features.extra.bezierOffset2y, features.extra.bezierOffset2z };
    for (Mesh& m : scene.meshes) {
        for (Vertex& v : m.vertices) {
            v.position += 2 * t * (1 - t) * (p1) + t * t * (p2);
        }
        for (glm::uvec3 triangle : m.triangles) {
            drawTriangle(m.vertices[triangle.x], m.vertices[triangle.y], m.vertices[triangle.z]);
        }
        for (Vertex& v : m.vertices) {
            v.position -= 2 * t * (1 - t) * (p1) + t * t * (p2);
        }
    }
    for (Sphere& s : scene.spheres) {
        s.center += 2 * t * (1 - t) * (p1) + t * t * (p2);
        drawSphere(s);
        s.center -= 2 * t * (1 - t) * (p1) + t * t * (p2);
    }
    for (float i = 0; i < features.extra.motionblurSamples; i++) {
        t = i / (float)(features.extra.motionblurSamples - 1);
        for (Mesh& m : scene.meshes) {
            for (Vertex& v : m.vertices) {
                v.position += 2 * t * (1 - t) * (p1) + t * t * (p2);
            }
            for (glm::uvec3 triangle : m.triangles) {
                drawTriangleMotion(m.vertices[triangle.x], m.vertices[triangle.y], m.vertices[triangle.z]);
            }
            for (Vertex& v : m.vertices) {
                v.position -= 2 * t * (1 - t) * (p1) + t * t * (p2);
            }
        }
        for (Sphere& s : scene.spheres) {
            s.center += 2 * t * (1 - t) * (p1) + t * t * (p2);
            drawSphere(s);
            s.center -= 2 * t * (1 - t) * (p1) + t * t * (p2);
        }
    }
}

void drawMotionblurPath(Scene& scene, const BVHInterface& bvh, const Features& features, const Trackball& camera, Screen& screen)
{
    drawMotionMeshAtTime(scene, features);
    for (Mesh& m : scene.meshes) {
        for (Vertex& v : m.vertices) {
            drawMovementLine(v.position, features);
        }
    }
    for (Sphere& s : scene.spheres) {
        drawMovementLine(s.center, features);
    }
    // drawLine()
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
    Scene scene2 = updateScene(scene, features);
    BVH bvh2 = BVH(scene2, features);
    Features features2 = features;
    features2.numPixelSamples = features.extra.motionblurSamples;
    for (int y = 0; y < screen.resolution().y; y++) {
        for (int x = 0; x != screen.resolution().x; x++) {
            // Assemble useful objects on a per-pixel basis; e.g. a per-thread sampler
            // Note; we seed the sampler for consistenct behavior across frames
            Sampler sampler = { static_cast<uint32_t>(screen.resolution().y * x + y) };
            RenderState state = {
                .scene = scene2,
                .features = features2,
                .bvh = bvh,
                .sampler = sampler
            };
            auto rays = generatePixelRays(state, camera, { x, y }, screen.resolution());
            float count = 0;
            for (Ray& r : rays) {
                r.time = sampler.next_1d();
                count++;
            }
            auto L = renderRays(state, rays);
            screen.setPixel(x, y, L);
        }
    }
}

//void drawMotionMeshAtTime(Scene scene, const Features& features) {
//    const double period = 5.0;
//    auto currentTime = std::chrono::high_resolution_clock::now();
//    auto duration = currentTime.time_since_epoch();
//    double seconds = std::chrono::duration_cast<std::chrono::duration<double>>(duration).count();
//    float t = (float)fmod(seconds / period, 1.0);
//    glm::vec3 p1 = { features.extra.bezierOffset1x, features.extra.bezierOffset1y, features.extra.bezierOffset1z };
//    glm::vec3 p2 = { features.extra.bezierOffset2x, features.extra.bezierOffset2y, features.extra.bezierOffset2z };
//    for (Mesh& m : scene.meshes) {
//        for (Vertex& v : m.vertices) {
//            v.position += 2 * t * (1 - t) * (p1) + t * t * (p2);
//        }
//        for (glm::uvec3 triangle : m.triangles) {
//            drawTriangle(m.vertices[triangle.x], m.vertices[triangle.y], m.vertices[triangle.z]);
//        }
//        for (Vertex& v : m.vertices) {
//            v.position -= 2 * t * (1 - t) * (p1) + t * t * (p2);
//        }
//    }
//    for (Sphere& s : scene.spheres) {
//        s.center += 2 * t * (1 - t) * (p1) + t * t * (p2);
//        drawSphere(s);
//        s.center -= 2 * t * (1 - t) * (p1) + t * t * (p2);
//    }
//    for (int i = 0; i < features.extra.keyFrames;i++) {
//        t = 1.f / features.extra.keyFrames * (i+.5); 
//        for (Mesh& m : scene.meshes) {
//            for (Vertex& v : m.vertices) {
//                v.position += 2 * t * (1 - t) * (p1) + t * t * (p2);
//            }
//            for (glm::uvec3 triangle : m.triangles) {
//                drawTriangleMotion(m.vertices[triangle.x], m.vertices[triangle.y], m.vertices[triangle.z]);
//            }
//            for (Vertex& v : m.vertices) {
//                v.position -= 2 * t * (1 - t) * (p1) + t * t * (p2);
//            }
//        }
//        for (Sphere& s : scene.spheres) {
//            s.center += 2 * t * (1 - t) * (p1) + t * t * (p2);
//            drawSphere(s);
//            s.center -= 2 * t * (1 - t) * (p1) + t * t * (p2);
//        }
//    }
//}
//
//void drawMotionblurPath(Scene& scene, const BVHInterface& bvh, const Features& features, const Trackball& camera, Screen& screen)
//{
//    drawMotionMeshAtTime(scene, features);
//    for (Mesh& m : scene.meshes) {
//        for (Vertex& v : m.vertices) {
//            drawMovementLine(v.position, features);
//        }
//    }
//    for (Sphere& s : scene.spheres) {
//        drawMovementLine(s.center, features);
//    }
//    //drawLine()
//}
//
//
//// TODO; Extra feature
//// Given the same input as for `renderImage()`, instead render an image with your own implementation
//// of motion blur. Here, you integrate over a time domain, and not just the pixel's image domain,
//// to give objects the appearance of "fast movement".
//// This method is not unit-tested, but we do expect to find it **exactly here**, and we'd rather
//// not go on a hunting expedition for your implementation, so please keep it here!
//void renderImageWithMotionBlur(const Scene& scene, const BVHInterface& bvh, const Features& features, const Trackball& camera, Screen& screen)
//{
//    if (!features.extra.enableMotionBlur) {
//        return;
//    }
//    Scene scene2 = updateScene(scene,features);
//    BVH bvh2 = BVH(scene2, features);
//    Features features2 = features;
//    int numSamples = features2.numPixelSamples;
//    features2.numPixelSamples *= features.extra.keyFrames;
//    for (int y = 0; y < screen.resolution().y; y++) {
//        for (int x = 0; x != screen.resolution().x; x++) {
//            // Assemble useful objects on a per-pixel basis; e.g. a per-thread sampler
//            // Note; we seed the sampler for consistenct behavior across frames
//            Sampler sampler = {static_cast<uint32_t>(screen.resolution().y * x + y)};
//            RenderState state = {
//                .scene = scene2,
//                .features = features2,
//                .bvh = bvh,
//                .sampler = sampler
//            };
//            auto rays = generatePixelRays(state, camera, { x, y }, screen.resolution());
//            int count = 0;
//            for (Ray& r : rays) {
//                if (count % numSamples == 0) {
//                    r.time = 1.f / (features.extra.keyFrames) * (count / numSamples + .5); 
//                }
//                r.time = 1.f / (features.extra.keyFrames) * (count / numSamples + .5 + (sampler.next_1d() * 2.f - 1.f)); 
//                count++;
//            }
//            auto L = renderRays(state, rays);
//            screen.setPixel(x, y, L);
//        }
//    }
//}

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

    int n = (int)features.extra.bloomFilterSize;
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
        float radius = diskRadius * std::sqrt(sample.y);
        float angle = 2.0f * glm::pi<float>() * sample.x;
        glm::vec3 rs = glm::normalize(r + u* radius* std::cos(angle) + v* radius* std::sin(angle));
        if (glm::dot(n, rs) > 0.0f) {
            Ray glossyRay = Ray(intersectionPoint + 0.001f * rs, rs);
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
        float x = intersectionPoint.x;
        float y = intersectionPoint.y;
        float z = intersectionPoint.z;
        bool posX = true;
        bool posY = true;
        bool posZ = true;
        if (x <= 0)
            posX = false;
        if (y <= 0)
            posY = false;
        if (z <= 0)
            posZ = false;
        float absX = std::fabs(x);
        float absY = std::fabs(y);
        float absZ = std::fabs(z);
        int index;
        float axis, u, v;
        if (posX && absX >= absY && absX >= absZ) {
            u = -z;
            v = y;
            index = 0;
            axis = absX;
        }
        else if (!posX && absX >= absY && absX >= absZ) {
            u = z;
            v = y;
            index = 1;
            axis = absX;
        }
        else if (posY && absY >= absX && absY >= absZ) {
            u = x;
            v = -z;
            index = 2;
            axis = absY;
        }
        else if (!posY && absY >= absX && absY >= absZ) {
            u = x;
            v = z;
            index = 3;
            axis = absY;
        }
        else if (posZ && absZ >= absX && absZ >= absY) {
            u = x;
            v = y;
            index = 4;
            axis = absZ;
        }
        else {
            u = -x;
            v = y;
            index = 5;
            axis = absZ;
        }
        u = 0.5f * (u / axis + 1.0f);
        v = 0.5f * (v / axis + 1.0f);
        return sampleTextureNearest(*cubeMap[index], glm::vec2(u, v));
    } else {
        return glm::vec3(0.f);
    }
}

// Helper method for SAH binning
float calculateAreaOfAABB(const AxisAlignedBox& aabb)
{
    glm::vec3 dimensions = aabb.upper - aabb.lower;
    float area1 = dimensions.x * dimensions.y;
    float area2 = dimensions.x * dimensions.z;
    float area3 = dimensions.y * dimensions.z;
    return 2 * (area1 + area2 + area3);
}

// Struct for SAH binning
struct SAHBin {
    AxisAlignedBox aabb = { .lower = glm::vec3(FLT_MAX), .upper = glm::vec3(-FLT_MAX) };
    AxisAlignedBox leftAABB;
    AxisAlignedBox rightAABB;
	size_t count=0;
    size_t leftCount;
    size_t rightCount;
};

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
    const size_t numBins = 16;
    SAHBin bins[numBins];
    std::sort(primitives.begin(), primitives.end(), [&](const Primitive& a, const Primitive& b) {
        return computePrimitiveCentroid(a)[axis] < computePrimitiveCentroid(b)[axis];
    });
    for (const Primitive& primitive : primitives) {
        glm::vec3 centroid = computePrimitiveCentroid(primitive);
        int binIndex = static_cast<int>((centroid[axis] - aabb.lower[axis]) / (aabb.upper[axis] - aabb.lower[axis]) * numBins);
        binIndex = std::clamp(binIndex, 0, static_cast<int> (numBins) - 1);
        AxisAlignedBox paabb = computePrimitiveAABB(primitive);
        bins[binIndex].aabb.lower = elementWiseMin(bins[binIndex].aabb.lower, paabb.lower);
        bins[binIndex].aabb.upper = elementWiseMax(bins[binIndex].aabb.upper, paabb.upper);
        bins[binIndex].count++;
    }

    bins[0].leftAABB = bins[0].aabb;
    bins[0].leftCount = bins[0].count;
    for (size_t i = 1; i < numBins-1; i++) {
        
        bins[i].leftAABB.lower = elementWiseMin(bins[i - 1].leftAABB.lower, bins[i].aabb.lower);
        bins[i].leftAABB.upper = elementWiseMax(bins[i - 1].leftAABB.upper, bins[i].aabb.upper);
        bins[i].leftCount = bins[i - 1].leftCount + bins[i].count;
    }

    bins[numBins - 1].rightAABB = bins[numBins - 1].aabb;
    bins[numBins - 1].rightCount = bins[numBins - 1].count;
    for (int i = numBins - 2; i >= 1; i--) {
        
        bins[i].rightAABB.lower = elementWiseMin(bins[i + 1].rightAABB.lower, bins[i].aabb.lower);
        bins[i].rightAABB.upper = elementWiseMax(bins[i + 1].rightAABB.upper, bins[i].aabb.upper);
        bins[i].rightCount = bins[i + 1].rightCount + bins[i].count;
    }

    size_t bestSplitIndex = 0;
    float bestSplitCost = FLT_MAX;
    for (size_t i = 0; i < numBins-1; i++) {

        float cost = bins[i].leftCount * calculateAreaOfAABB(bins[i].leftAABB) + bins[i + 1].rightCount * calculateAreaOfAABB(bins[i+1].rightAABB);

        if (bins[i].leftCount!=0 && bins[i+1].rightCount!=0 && cost < bestSplitCost) {
            bestSplitCost = cost;
            bestSplitIndex = i;
        }
    }

    if (bestSplitIndex == 0 || bestSplitIndex == 15)
		return splitPrimitivesByMedian(aabb,axis,primitives);
    return bins[bestSplitIndex].leftCount;
}
