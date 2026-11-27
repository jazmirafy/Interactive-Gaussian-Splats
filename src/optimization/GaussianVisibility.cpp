#include "GaussianVisibility.h"

#include <chrono>
#include <limits>
#include <stdexcept>

void GaussianVisibility::update(std::span<const GaussianBounds> gaussians, const glm::mat4& viewProjection, const VisibilitySettings& settings)
{
    if (gaussians.size() > std::numeric_limits<uint32_t>::max())
    {
        throw std::length_error("Gaussian count exceeds index capacity.");
    }
    auto start = std::chrono::steady_clock::now();
    // Reset previous frame data
    visibleIndices.clear();
    visibleIndices.reserve(gaussians.size());
    stats = {};
    // Update camera frustum
    if (settings.cullingMode != CullingMode::Disabled)
    {
        frustum.update(viewProjection);
    }
    // Generate visible Gaussian indices
    for (size_t i = 0; i < gaussians.size(); i++)
    {
        if (isVisible(gaussians[i], settings))
        {
            visibleIndices.push_back(static_cast<uint32_t>(i));
        }
    }
    // Update visibility statistics
    stats.totalGaussians = gaussians.size();
    stats.visibleGaussians = visibleIndices.size();
    stats.culledGaussians = stats.totalGaussians - stats.visibleGaussians;
    if (stats.totalGaussians > 0)
    {
        stats.cullingPercentage = 100.0 * stats.culledGaussians / stats.totalGaussians;
    }
    auto end = std::chrono::steady_clock::now();
    stats.milliseconds = std::chrono::duration<double, std::milli>(end - start).count();
}

bool GaussianVisibility::isVisible(const GaussianBounds& gaussian, const VisibilitySettings& settings) const
{
    switch (settings.cullingMode)
    {
    case CullingMode::Disabled:
        return true;
    case CullingMode::Point:
        return frustum.containsPoint(gaussian.position);
    case CullingMode::Sphere:
        return frustum.intersectsSphere(gaussian.position, gaussian.sphereRadius);
    case CullingMode::AABB:
        return frustum.intersectsAABB(gaussian.position, gaussian.aabbHalfExtents);
    case CullingMode::Ellipsoid:
        return frustum.intersectsEllipsoid(gaussian.position, gaussian.covariance, settings.cutoff);
    case CullingMode::Hybrid:
        return frustum.intersectsHybrid(gaussian.position, gaussian.sphereRadius, gaussian.covariance, settings.cutoff);
    default:
        return true;
    }
}

const std::vector<uint32_t>& GaussianVisibility::getVisibleIndices() const
{
    return visibleIndices;
}

const VisibilityStats& GaussianVisibility::getStats() const
{
    return stats;
}