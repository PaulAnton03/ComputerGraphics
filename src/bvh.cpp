#include "bvh.h"
#include "draw.h"
#include "extra.h"
#include "interpolate.h"
#include "intersect.h"
#include "render.h"
#include "scene.h"
#include "texture.h"
#include <algorithm>
#include <bit>
#include <chrono>
#include <framework/opengl_includes.h>
#include <iostream>
#include <queue>
#include <stack>

// Helper method to fill in hitInfo object. This can be safely ignored (or extended).
// Note: many of the functions in this helper tie in to standard/extra features you will have
// to implement separately, see interpolate.h/.cpp for these parts of the project
void updateHitInfo(RenderState& state, const BVHInterface::Primitive& primitive, const Ray& ray, HitInfo& hitInfo)
{
    const auto& [v0, v1, v2] = std::tie(primitive.v0, primitive.v1, primitive.v2);
    const auto& mesh = state.scene.meshes[primitive.meshID];
    const auto n = glm::normalize(glm::cross(v1.position - v0.position, v2.position - v0.position));
    const auto p = ray.origin + ray.t * ray.direction;

    // First, fill in default data, unrelated to separate features
    hitInfo.material = mesh.material;
    hitInfo.normal = n;
    hitInfo.barycentricCoord = computeBarycentricCoord(v0.position, v1.position, v2.position, p);

    // Next, if `features.enableNormalMapping` is true, generate smoothly interpolated vertex normals
    if (state.features.enableNormalInterp) {
        hitInfo.normal = interpolateNormal(v0.normal, v1.normal, v2.normal, hitInfo.barycentricCoord);
    }

    // Next, if `features.enableTextureMapping` is true, generate smoothly interpolated vertex uvs
    if (state.features.enableTextureMapping) {
        hitInfo.texCoord = interpolateTexCoord(v0.texCoord, v1.texCoord, v2.texCoord, hitInfo.barycentricCoord);
    }

    // Finally, catch flipped normals
    if (glm::dot(ray.direction, n) > 0.0f) {
        hitInfo.normal = -hitInfo.normal;
    }
}

// BVH constructor; can be safely ignored. You should not have to touch this
// NOTE: this constructor is tested, so do not change the function signature.
BVH::BVH(const Scene& scene, const Features& features)
{
#ifndef NDEBUG
    // Store start of bvh build for timing
    using clock = std::chrono::high_resolution_clock;
    const auto start = clock::now();
#endif

    // Count the total nr. of triangles in the scene
    size_t numTriangles = 0;
    for (const auto& mesh : scene.meshes)
        numTriangles += mesh.triangles.size();

    // Given the input scene, gather all triangles over which to build the BVH as a list of Primitives
    std::vector<Primitive> primitives;
    primitives.reserve(numTriangles);
    for (uint32_t meshID = 0; meshID < scene.meshes.size(); meshID++) {
        const auto& mesh = scene.meshes[meshID];
        for (const auto& triangle : mesh.triangles) {
            primitives.push_back(Primitive {
                .meshID = meshID,
                .v0 = mesh.vertices[triangle.x],
                .v1 = mesh.vertices[triangle.y],
                .v2 = mesh.vertices[triangle.z] });
        }
    }

    // Tell underlying vectors how large they should approximately be
    m_primitives.reserve(numTriangles);
    m_nodes.reserve(numTriangles + 1);

    // Recursively build BVH structure; this is where your implementation comes in
    m_nodes.emplace_back(); // Create root node
    m_nodes.emplace_back(); // Create dummy node s.t. children are allocated on the same cache line
    buildRecursive(scene, features, primitives, RootIndex);

    // Fill in boilerplate data
    buildNumLevels();
    buildNumLeaves();

#ifndef NDEBUG
    // Output end of bvh build for timing
    const auto end = clock::now();
    std::cout << "BVH construction time: " << std::chrono::duration<double, std::milli>(end - start).count() << "ms" << std::endl;
#endif
}

// BVH helper method; allocates a new node and returns its index
// You should not have to touch this
uint32_t BVH::nextNodeIdx()
{
    const auto idx = static_cast<uint32_t>(m_nodes.size());
    m_nodes.emplace_back();
    return idx;
}

glm::vec3 elementWiseMin(const glm::vec3 a, const glm::vec3 b)
{
    return { glm::min(a.x, b.x), glm::min(a.y, b.y), glm::min(a.z, b.z) };
}

glm::vec3 elementWiseMax(const glm::vec3 a, const glm::vec3 b)
{
    return { glm::max(a.x, b.x), glm::max(a.y, b.y), glm::max(a.z, b.z) };
}

glm::vec3 minOfThree(const glm::vec3 a, const glm::vec3 b, const glm::vec3 c)
{
    return elementWiseMin(a, elementWiseMin(b, c));
}

glm::vec3 maxOfThree(const glm::vec3 a, const glm::vec3 b, const glm::vec3 c)
{
    return elementWiseMax(a, elementWiseMax(b, c));
    }

glm::vec3 minOfList(const std::vector<const glm::vec3>& list)
{
    return std::reduce(list.begin(), list.end(), glm::vec3(std::numeric_limits<float>::max()),
        [&](const glm::vec3& a, const glm::vec3& b) {
            return elementWiseMin(a, b);
        });
}

glm::vec3 maxOfList(const std::vector<const glm::vec3>& list)
{
    return std::reduce(list.begin(), list.end(), glm::vec3(std::numeric_limits<float>::min()),
        [&](const glm::vec3& a, const glm::vec3& b) {
            return elementWiseMax(a, b);
        });
}

// DONE: Standard feature
// Given a BVH triangle, compute an axis-aligned bounding box around the primitive
// - primitive; a single triangle to be stored in the BVH
// - return;    an axis-aligned bounding box around the triangle
// This method is unit-tested, so do not change the function signature.
AxisAlignedBox computePrimitiveAABB(const BVHInterface::Primitive primitive)
{
    return { .lower = minOfThree(primitive.v0.position, primitive.v1.position, primitive.v2.position),
        .upper = maxOfThree(primitive.v0.position, primitive.v1.position, primitive.v2.position) };
}

// DONE: Standard feature
// Given a range of BVH triangles, compute an axis-aligned bounding box around the range.
// - primitive; a contiguous range of triangles to be stored in the BVH
// - return;    a single axis-aligned bounding box around the entire set of triangles
// This method is unit-tested, so do not change the function signature.
AxisAlignedBox computeSpanAABB(std::span<const BVHInterface::Primitive> primitives)
{
    glm::vec3 min = glm::vec3(std::numeric_limits<float>::max());
    glm::vec3 max = glm::vec3(std::numeric_limits<float>::min());
    for (BVHInterface::Primitive p : primitives) {
        min = elementWiseMin(minOfThree(p.v0.position, p.v1.position, p.v2.position), min);
        max = elementWiseMax(maxOfThree(p.v0.position, p.v1.position, p.v2.position), max);
    }

    return { .lower = min, .upper = max };
}

// DONE: Standard feature
// Given a BVH triangle, compute the geometric centroid of the triangle
// - primitive; a single triangle to be stored in the BVH
// - return;    the geometric centroid of the triangle's vertices
// This method is unit-tested, so do not change the function signature.
glm::vec3 computePrimitiveCentroid(const BVHInterface::Primitive primitive)
{
    return (primitive.v1.position + primitive.v2.position + primitive.v0.position) / 3.0f;
}

// DONE: Standard feature
// Given an axis-aligned bounding box, compute the longest axis; x = 0, y = 1, z = 2.
// - aabb;   the input axis-aligned bounding box
// - return; 0 for the x-axis, 1 for the y-axis, 2 for the z-axis
//           if several axes are equal in length, simply return the first of these
// This method is unit-tested, so do not change the function signature.
uint32_t computeAABBLongestAxis(const AxisAlignedBox& aabb)
{
    glm::vec3 diff = aabb.upper - aabb.lower;

    diff.x = glm::abs(diff.x);
    diff.y = glm::abs(diff.y);
    diff.z = glm::abs(diff.z);

    if (diff.x > diff.y && diff.x > diff.z)
        return 0;
    else if (diff.y > diff.x && diff.y > diff.z)
        return 1;
    else
        return 2;
}

// DONE: Standard feature
// Given a range of BVH triangles, sort these along a specified axis based on their geometric centroid.
// Then, find and return the split index in the range, such that the subrange containing the first element
// of the list is at least as big as the other, and both differ at most by one element in size.
// Hint: you should probably reuse `computePrimitiveCentroid()`
// - aabb;       the axis-aligned bounding box around the given triangle range
// - axis;       0, 1, or 2, determining on which axis (x, y, or z) the split must happen
// - primitives; the modifiable range of triangles that requires sorting/splitting along an axis
// - return;     the split position of the modified range of triangles
// This method is unit-tested, so do not change the function signature.
size_t splitPrimitivesByMedian(const AxisAlignedBox& aabb, uint32_t axis, std::span<BVHInterface::Primitive> primitives)
{
    using Primitive = BVHInterface::Primitive;

    if(primitives.empty())
        return 0;
//    std::vector<float> centroidsAlongAxis;
//    for (const Primitive &p : primitives) {
//         centroidsAlongAxis.push_back((axis == 0) ? computePrimitiveCentroid(p).x :
//                (axis == 1) ? computePrimitiveCentroid(p).y : computePrimitiveCentroid(p).z);
//    }
    sort(primitives.begin(), primitives.end(), [&](const Primitive &a, const Primitive &b) {
        return (axis == 0) ? computePrimitiveCentroid(a).x < computePrimitiveCentroid(b).x :
               (axis == 1) ? computePrimitiveCentroid(a).y < computePrimitiveCentroid(b).y :
               computePrimitiveCentroid(a).z < computePrimitiveCentroid(b).z; });
    return (primitives.size() + 1) / 2;
}

// DONE: Standard feature
// Hierarchy traversal routine; called by the BVH's intersect(),
// you must implement this method and implement it carefully!
//
// If `features.enableAccelStructure` is not enabled, the method should just iterate the BVH's
// underlying primitives (or the scene's geometry). The default imlpementation already does this.
// You will have to implement the part which actually traverses the BVH for a faster intersect,
// given that `features.enableAccelStructure` is enabled.
//
// This method returns `true` if geometry was hit, and `false` otherwise. On first/closest hit, the
// distance `t` in the `ray` object is updated, and information is updated in the `hitInfo` object.
//
// - state;    the active scene, and a user-specified feature config object, encapsulated
// - bvh;      the actual bvh which should be traversed for faster intersection
// - ray;      the ray intersecting the scene's geometry
// - hitInfo;  the return object, with info regarding the hit geometry
// - return;   boolean, if geometry was hit or not
//
// This method is unit-tested, so do not change the function signature.
bool intersectRayWithBVH(RenderState& state, const BVHInterface& bvh, Ray& ray, HitInfo& hitInfo)
{
    // Relevant data in the constructed BVH
    std::span<const BVHInterface::Node> nodes = bvh.nodes();
    std::span<const BVHInterface::Primitive> primitives = bvh.primitives();

    // Return value
    bool is_hit = false;

    if (state.features.enableAccelStructure) {
        // DONE: implement here your (probably stack-based) BVH traversal.
        //
        // Some hints (refer to bvh_interface.h either way). BVH nodes are packed, so the
        // data is not easily extracted. Helper methods are available, however:
        // - For a given node, you can test if the node is a leaf with `node.isLeaf()`.
        // - If the node is not a leaf, you can obtain the left/right children with `node.leftChild()` etc.
        // - If the node is a leaf, you can obtain the offset to and nr. of primitives in the bvh's list
        //   of underlying primitives with `node.primitiveOffset()` and `node.primitiveCount()`
        //
        // In short, you will have to step down the bvh, node by node, and intersect your ray
        // with the node's AABB. If this intersection passes, you should:
        // - if the node is a leaf, intersect with the leaf's primitives
        // - if the node is not a leaf, test the left and right children as well!
        //
        // Note that it is entirely possible for a ray to hit a leaf node, but not its primitives,
        // and it is likewise possible for a ray to hit both children of a node.

        std::stack<uint32_t> nodeStack;
        nodeStack.push(0);
        while(!nodeStack.empty()) {
            const BVHInterface::Node& node = nodes[nodeStack.top()];
            nodeStack.pop();
            float t = ray.t;
            if (intersectRayWithShape(node.aabb, ray)) {
                ray.t = t;
                if (node.isLeaf()) {
                    for (uint32_t i = node.primitiveOffset(); i < node.primitiveOffset() + node.primitiveCount(); i++) {
                        const BVHInterface::Primitive& prim = primitives[i];
                        if (intersectRayWithTriangle(prim.v0.position, prim.v1.position, prim.v2.position, ray, hitInfo)) {
                            updateHitInfo(state, prim, ray, hitInfo);
                            is_hit = true;
                        }
                    }
                } else {
                    nodeStack.push(node.leftChild());
                    nodeStack.push(node.rightChild());
                }
            }
        }


    } else {
        // Naive implementation; simply iterates over all primitives
        for (const auto& prim : primitives) {
            const auto& [v0, v1, v2] = std::tie(prim.v0, prim.v1, prim.v2);
            if (intersectRayWithTriangle(v0.position, v1.position, v2.position, ray, hitInfo)) {
                updateHitInfo(state, prim, ray, hitInfo);
                is_hit = true;
            }
        }
    }

    // Intersect with spheres.
    for (const auto& sphere : state.scene.spheres)
        is_hit |= intersectRayWithShape(sphere, ray, hitInfo);

    return is_hit;
}

// DONE: Standard feature
// Leaf construction routine; you should reuse this in in `buildRecursive()`
// Given an axis-aligned bounding box, and a range of triangles, generate a valid leaf object
// and store the triangles in the `m_primitives` vector.
// You are free to modify this function's signature, as long as the constructor builds a BVH
// - scene;      the active scene
// - features;   the user-specified features object
// - aabb;       the axis-aligned bounding box around the primitives beneath this leaf
// - primitives; the range of triangles to be stored for this leaf
BVH::Node BVH::buildLeafData(const Scene& scene, const Features& features, const AxisAlignedBox& aabb, std::span<Primitive> primitives)
{
    Node node;

    node.aabb = aabb;
    node.data = { Node::LeafBit | static_cast<uint32_t>(m_primitives.size()), static_cast<uint32_t>(primitives.size()) };

    // Copy the current set of primitives to the back of the primitives vector
    std::copy(primitives.begin(), primitives.end(), std::back_inserter(m_primitives));

    return node;
}

// DONE: Standard feature
// Node construction routine; you should reuse this in in `buildRecursive()`
// Given an axis-aligned bounding box, and left/right child indices, generate a valid node object.
// You are free to modify this function's signature, as long as the constructor builds a BVH
// - scene;           the active scene
// - features;        the user-specified features object
// - aabb;            the axis-aligned bounding box around the primitives beneath this node
// - leftChildIndex;  the index of the node's left child in `m_nodes`
// - rightChildIndex; the index of the node's right child in `m_nodes`
BVH::Node BVH::buildNodeData(const Scene& scene, const Features& features, const AxisAlignedBox& aabb, uint32_t leftChildIndex, uint32_t rightChildIndex)
{
    Node node;

    node.aabb = aabb;
    node.data = {leftChildIndex, rightChildIndex};

    return node;
}

// TODO: Standard feature
// Hierarchy construction routine; called by the BVH's constructor,
// you must implement this method and implement it carefully!
//
// You should implement the other BVH standard features first, and this feature last, as you can reuse
// most of the other methods to assemble this part. There are detailed instructions inside the
// method which we recommend you follow.
//
// Arguments:
// - scene;      the active scene
// - features;   the user-specified features object
// - primitives; a range of triangles to be stored in the BVH
// - nodeIndex;  index of the node you are currently working on, this is already allocated
//
// You are free to modify this function's signature, as long as the constructor builds a BVH
void BVH::buildRecursive(const Scene& scene, const Features& features, std::span<Primitive> primitives, uint32_t nodeIndex)
{
    // WARNING: always use nodeIndex to index into the m_nodes array. never hold a reference/pointer,
    // because a push/emplace (in ANY recursive calls) might grow vectors, invalidating the pointers.

    // Compute the AABB of the current node.
    AxisAlignedBox aabb = computeSpanAABB(primitives);

    // As a starting point, we provide an implementation which creates a single leaf, and stores
    // all triangles inside it. You should remove or comment this, and work on your own recursive
    // construction algorithm that implements the following steps. Make sure to reuse the methods
    // you have previously implemented to simplify this process.
    //
    // 1. Determine if the node should be a leaf, when the nr. of triangles is less or equal to 4
    //    (hint; use the `LeafSize` constant)
    // 2. If it is a leaf, fill in the leaf's data, and store its range of triangles in `m_primitives`
    // 3. If it is a node:
    //    3a. Split the range of triangles along the longest axis into left and right subspans,
    //        using either median or SAH-Binning based on the `Features` object
    //    3b. Allocate left/right child nodes
    //        (hint: use `nextNodeIdx()`)
    //    3c. Fill in the current node's data; aabb, left/right child indices
    //    3d. Recursively build left/right child nodes over their respective triangles
    //        (hint; use `std::span::subspan()` to split into left/right ranges)

    if(primitives.size() <= LeafSize) {
        m_nodes[nodeIndex] = buildLeafData(scene, features, aabb, primitives);
        return;
    }

    uint32_t axis = computeAABBLongestAxis(aabb);
    size_t splitIndex = splitPrimitivesByMedian(aabb, axis, primitives);

    std::span<Primitive> leftPrimitives = primitives.subspan(0, splitIndex);
    std::span<Primitive> rightPrimitives = primitives.subspan(splitIndex, primitives.size() - splitIndex);

    uint32_t leftIdx = nextNodeIdx();

    uint32_t rightIdx = nextNodeIdx();

    m_nodes[nodeIndex] = buildNodeData(scene, features, aabb, leftIdx, rightIdx);

    buildRecursive(scene, features, leftPrimitives, leftIdx);
    buildRecursive(scene, features, rightPrimitives, rightIdx);
//    std::cout << m_nodes.size() << std::endl;



    // Just configure the current node as a giant leaf for now
//    m_nodes[nodeIndex] = buildLeafData(scene, features, aabb, primitives);
}

// DONE: Standard feature, or part of it
// Compute the nr. of levels in your hierarchy after construction; useful for `debugDrawLevel()`
void BVH::buildNumLevels()
{
    m_numLevels = static_cast<uint32_t>(glm::ceil(glm::log2((float) m_nodes.size() + 1 - RootIndex)) - 1);
}

// Compute the nr. of leaves in your hierarchy after construction; useful for `debugDrawLeaf()`
// You are free to modify this function's signature, as long as the constructor builds a BVH
void BVH::buildNumLeaves()
{
    if(m_primitives.empty()) m_numLeaves = 0;
    m_numLeaves = 0;

    for(uint32_t i = RootIndex; i < m_nodes.size(); i++) {
        if(m_nodes[i].isLeaf()) m_numLeaves++;
    }

//    else if(m_primitives.size() < 5) m_numLeaves = 1;
//    else {
//        auto s = static_cast<uint32_t>(m_primitives.size());
//        float log = glm::log2((float) m_primitives.size() );
//        auto a = 2 ^ static_cast<uint32_t>(glm::ceil(log));
//        auto b = 2 ^ static_cast<uint32_t>(glm::floor(log));
//
//        m_numLeaves = (b/4) + std::min(s - b, (a - b) / 4);
//
//    }
}

// Draw the bounding boxes of the nodes at the selected level. Use this function to visualize nodes
// for debugging. You may wish to implement `buildNumLevels()` first. We suggest drawing the AABB
// of all nodes on the selected level.
// You are free to modify this function's signature.
void BVH::debugDrawLevel(int level)
{
    // Example showing how to draw an AABB as a (white) wireframe box.
    // Hint: use draw functions (see `draw.h`) to draw the contained boxes with different
    // colors, transparencies, etc.
    std::queue<uint32_t> childQueue;
    std::queue<uint32_t> curQueue;
    curQueue.push(RootIndex);

    int curLevel = 0;
    while(curLevel != level && (!childQueue.empty() || !curQueue.empty())) {
        if(curQueue.empty()) {
            curQueue = childQueue;
            childQueue = std::queue<uint32_t>();
            curLevel++;
        }
        else{
            Node & curNode = m_nodes[curQueue.front()];
                if(!curNode.isLeaf()) {
                        childQueue.push(curNode.leftChild());
                        childQueue.push(curNode.rightChild());
                }
            curQueue.pop();
        }
    }

    while(!curQueue.empty()) {
        drawAABB(m_nodes[curQueue.front()].aabb, DrawMode::Wireframe, glm::vec3(0.05f, 1.0f, 0.05f), 0.1f);
        curQueue.pop();
    }


//    AxisAlignedBox aabb { .lower = glm::vec3(0.0f), .upper = glm::vec3(0.0f, 1.05f, 1.05f) };
//    drawAABB(aabb, DrawMode::Wireframe, glm::vec3(0.05f, 1.0f, 0.05f), 0.1f);
}

// Draw data of the leaf at the selected index. Use this function to visualize leaf nodes
// for debugging. You may wish to implement `buildNumLeaves()` first. We suggest drawing the AABB
// of the selected leaf, and then its underlying primitives with different colors.
// - leafIndex; index of the selected leaf.
//              (Hint: not the index of the i-th node, but of the i-th leaf!)
// You are free to modify this function's signature.
void BVH::debugDrawLeaf(int leafIndex)
{
    // Example showing how to draw an AABB as a (white) wireframe box.
    // Hint: use drawTriangle (see `draw.h`) to draw the contained primitives
//    AxisAlignedBox aabb { .lower = glm::vec3(0.0f), .upper = glm::vec3(0.0f, 1.05f, 1.05f) };
    if (leafIndex < 0 || leafIndex > m_numLeaves) return;
    int j = 0;
    uint32_t ind = 0;
    for(ind = RootIndex; ind < m_nodes.size() && j < leafIndex; ind++) {
        if(m_nodes[ind].isLeaf()) {
            j ++;
        }
    }
    if (j != leafIndex || ind >= m_nodes.size() || !m_nodes[ind].isLeaf()) return;

    const Node & node = m_nodes[ind];
    AxisAlignedBox aabb = node.aabb;
    drawAABB(aabb, DrawMode::Wireframe, glm::vec3(0.05f, 1.0f, 0.05f), 0.1f);
    for(uint32_t i = node.primitiveOffset(); i <= node.primitiveOffset() + node.primitiveCount() - 1; i ++) {
        Primitive & p = m_primitives[i];
        drawTriangle(p.v0, p.v1, p.v2);
    }
}