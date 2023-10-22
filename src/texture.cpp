#include "texture.h"
#include "render.h"
#include <framework/image.h>
#include <iostream>

// TODO: Standard feature
// Given an image, and relevant texture coordinates, sample the texture s.t.
// the nearest texel to the coordinates is acquired from the image.
// - image;    the image object to sample from.
// - texCoord; sample coordinates, generally in [0, 1]
// - return;   the nearest corresponding texel
// This method is unit-tested, so do not change the function signature.
glm::vec3 sampleTextureNearest(const Image& image, const glm::vec2& texCoord)
{
    // TODO: implement this function.
    // Note: the pixels are stored in a 1D array, row-major order. You can convert from (i, j) to
    //       an index using the method seen in the lecture.
    // Note: the center of the first pixel should be at coordinates (0.5, 0.5)
    // Given texcoords, return the corresponding pixel of the image
    // The pixel are stored in a 1D array of row major order
    // you can convert from position (i,j) to an index using the method seen in the lecture
    // Note, the center of the first pixel is at image coordinates (0.5, 0.5)
    int i = (int) glm::min(image.width-1.0f,glm::max(0.0f,(texCoord.x*image.width)));
    int j = (int)glm::min(image.width - 1.0f, glm::max(0.0f, (texCoord.y * image.width)));
    return image.pixels[i+j*image.width];
}

// TODO: Standard feature
// Given an image, and relevant texture coordinates, sample the texture s.t.
// a bilinearly interpolated texel is acquired from the image.
// - image;    the image object to sample from.
// - texCoord; sample coordinates, generally in [0, 1]
// - return;   the filter of the corresponding texels
// This method is unit-tested, so do not change the function signature.
glm::vec3 sampleTextureBilinear(const Image& image, const glm::vec2& texCoord)
{
    // TODO: implement this function.
    // Note: the pixels are stored in a 1D array, row-major order. You can convert from (i, j) to
    //       an index using the method seen in the lecture.
    // Note: the center of the first pixel should be at coordinates (0.5, 0.5)
    // Given texcoords, return the corresponding pixel of the image
    // The pixel are stored in a 1D array of row major order
    // you can convert from position (i,j) to an index using the method seen in the lecture
    // Note, the center of the first pixel is at image coordinates (0.5, 0.5)
    float i = glm::max(0.0f, (texCoord.x * image.width)-.5f);
    float j = glm::max(0.0f, (texCoord.y * image.width) - .5f);
    float alpha = i - glm::floor(i);
    float beta = j - glm::floor(j);
    int i1 = glm::floor(i);
    int i2 = glm::ceil(i);
    int j1 = glm::floor(j);
    int j2 = glm::ceil(j);
    //if (i >= image.width - 1 || i <= 1) {
    //    std::cout << i << std::endl;
    //    return glm::vec3 { 1.0f, 0.0f, 0.0f };
    //}
    //if (j >= image.height - 1 || j <= 1) {
    //    std::cout << j << std::endl;
    //    return glm::vec3 { 1.0f, 0.0f, 0.0f };
    //}
    glm::vec3 res = ((1.0f - alpha) * image.pixels[i1 + j1 * image.width] + alpha * image.pixels[i2 + j1 * image.width]) * (1.0f - beta) + ((1.0f - alpha) * image.pixels[i1 + j2 * image.width] + alpha * image.pixels[i2 + j2 * image.width]) * beta;
    return res;
}