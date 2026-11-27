#include <cstddef>
#include <cstdint>
#include <span>
#include <vector>
#include <glm/glm.hpp>

#include "Frustum.h"

enum class CullingMode { Disabled, Point, Sphere, AABB, Ellipsoid, Hybrid };

struct GaussianBounds
{
    glm::vec3 position;
    float sphereRadius;
    glm::vec3 aabbHalfExtents;
    glm::mat3 covariance;
};

struct VisibilitySettings
{
    CullingMode cullingMode = CullingMode::Sphere;
    float cutoff = 3.0f;
};

struct VisibilityStats
{
    size_t totalGaussians = 0;
    size_t visibleGaussians = 0;
    size_t culledGaussians = 0;
    double cullingPercentage = 0.0;
    double milliseconds = 0.0;
};

class GaussianVisibility
{
public:
    void update(std::span<const GaussianBounds> gaussians, const glm::mat4& viewProjection, const VisibilitySettings& settings);

    const std::vector<uint32_t>& getVisibleIndices() const;
    const VisibilityStats& getStats() const;

private:
    Frustum frustum;
    std::vector<uint32_t> visibleIndices;
    VisibilityStats stats;

    bool isVisible(const GaussianBounds& gaussian, const VisibilitySettings& settings) const;
};