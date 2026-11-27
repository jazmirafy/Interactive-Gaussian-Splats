#include "ScreenSpace.h"

#include <cmath>
#include <limits>

void ScreenSpace::update(const glm::mat4& view, const glm::mat4& projection, int viewportWidth, int viewportHeight, float nearPlane)
{
    viewMatrix = view;
    this->nearPlane = nearPlane;
    // Calculate focal lengths in pixels
    focalX = glm::abs(projection[0][0]) * viewportWidth * 0.5f;
    focalY = glm::abs(projection[1][1]) * viewportHeight * 0.5f;
    valid = viewportWidth > 0 && viewportHeight > 0 &&
        nearPlane > 0.0f && std::isfinite(nearPlane) &&
        focalX > 0.0f && focalY > 0.0f &&
        std::isfinite(focalX) && std::isfinite(focalY);
}

float ScreenSpace::estimateSphereRadius(const glm::vec3& center, float sphereRadius) const
{
    const float infinity = std::numeric_limits<float>::infinity();
    if (!valid || !std::isfinite(sphereRadius) || sphereRadius < 0.0f)
    {
        return infinity;
    }
    // Transform center into camera space
    glm::vec4 viewPosition = viewMatrix * glm::vec4(center, 1.0f);
    float depth = -viewPosition.z;
    // Avoid unreliable near-plane projections
    if (!std::isfinite(depth) || depth - sphereRadius <= nearPlane)
    {
        return infinity;
    }
    // Estimate maximum projected radius
    float focalLength = glm::max(focalX, focalY);
    return focalLength * sphereRadius / depth;
}