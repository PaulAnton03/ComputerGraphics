#include "interpolate.h"
#include <glm/geometric.hpp>

// TODO Standard feature
// Given three triangle vertices and a point on the triangle, compute the corresponding barycentric coordinates of the point.
// and return a vec3 with the barycentric coordinates (alpha, beta, gamma).
// - v0;     Triangle vertex 0
// - v1;     Triangle vertex 1
// - v2;     Triangle vertex 2
// - p;      Point on triangle
// - return; Corresponding barycentric coordinates for point p.
// This method is unit-tested, so do not change the function signature.
glm::vec3 computeBarycentricCoord(const glm::vec3& v0, const glm::vec3& v1, const glm::vec3& v2, const glm::vec3& p)
{
    // In case the normal is the zero vector, this will break
    //glm::vec3 normal = glm::vec3 { 0 };
    //if (resVec.x == 0 && resVec.y == 0 && resVec.z == 0) {
    glm::vec3 n = glm::normalize(glm::cross(v0 - v2, v1 - v2));
    //}
    //glm::vec3 na = glm::cross(v2 - v1, p - v1);
    //glm::vec3 nb = glm::cross(v0 - v2, p - v2);
    //glm::vec3 nc = glm::cross(v1 - v0, p - v0);

    //float a = glm::dot(normal, na);
    //float b = glm::dot(normal, nb);
    //float c = glm::dot(normal, nc);
    //return glm::vec3 { a, b, c };
    float area = glm::dot(n, glm::cross(v2 - v0, v1 - v0));

    if (area == 0) {
        return { 0, 0, 0 };
    }
    float a0 = glm::dot(n, glm::cross(p - v1, v2 - v1)) / area;
    float a1 = glm::dot(n, glm::cross(p - v2, v0 - v2)) / area;
    return { a0, a1, 1.f - a1 - a0 };
}

// TODO Standard feature
// Linearly interpolate three normals using barycentric coordinates.
// - n0;     Triangle normal 0
// - n1;     Triangle normal 1
// - n2;     Triangle normal 2
// - bc;     Barycentric coordinate
// - return; The smoothly interpolated normal.
// This method is unit-tested, so do not change the function signature.
glm::vec3 interpolateNormal(const glm::vec3& n0, const glm::vec3& n1, const glm::vec3& n2, const glm::vec3 bc)
{
    return glm::normalize(n0 * bc.x + n1 * bc.y + n2 * bc.z);
}

// TODO Standard feature
// Linearly interpolate three texture coordinates using barycentric coordinates.
// - n0;     Triangle texture coordinate 0
// - n1;     Triangle texture coordinate 1
// - n2;     Triangle texture coordinate 2
// - bc;     Barycentric coordinate
// - return; The smoothly interpolated texturre coordinate.
// This method is unit-tested, so do not change the function signature.
glm::vec2 interpolateTexCoord(const glm::vec2& t0, const glm::vec2& t1, const glm::vec2& t2, const glm::vec3 bc)
{
    glm::vec2 cur = t0 * bc.x + t1 * bc.y + t2 * bc.z;
    
    return t0 * bc.x + t1 * bc.y + t2 * bc.z;
    //return glm::normalize(t0 * bc.x + t1 * bc.y + t2 * bc.z);
}
