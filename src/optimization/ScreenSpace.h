#pragma once

#include <glm/glm.hpp>

class ScreenSpace
{
public:
    void update(const glm::mat4& view, const glm::mat4& projection, int viewportWidth, int viewportHeight, float nearPlane);

    float estimateSphereRadius(const glm::vec3& center, float sphereRadius) const;

private:
    glm::mat4 viewMatrix{ 1.0f };
    float focalX = 0.0f;
    float focalY = 0.0f;
    float nearPlane = 0.0f;
    bool valid = false;
};