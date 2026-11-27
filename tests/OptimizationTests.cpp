#include <iostream>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
// Added for Benchmarks
#include <algorithm>
#include <chrono>
#include <cstdint>
#include <iomanip>
#include <random>
#include <vector>
#include <cmath>

#include "optimization/Frustum.h"
#include "optimization/GaussianVisibility.h"
#include "optimization/ScreenSpace.h"

#pragma region Test Data Structures

#pragma region Frustum
struct PointTest
{
    const char* name;
    glm::vec3 position;
    bool expectedVisible;
};

struct SphereTest
{
    const char* name;
    glm::vec3 center;
    float radius;
    bool expectedVisible;
};

struct AABBTest
{
    const char* name;
    glm::vec3 center;
    glm::vec3 halfExtents;
    bool expectedVisible;
};

struct EllipsoidTest
{
    const char* name;
    glm::vec3 center;
    glm::mat3 covariance;
    float cutoff;
    bool expectedVisible;
    float sphereRadius = 0.0f;
};
#pragma endregion

struct VisibilityTest 
{
    const char* name;
    CullingMode mode;
    std::vector<uint32_t> expectedIndices;
};

struct ScreenSpaceTest
{
    const char* name;
    glm::vec3 position;
    float sphereRadius;
    float expectedRadius;
};

struct TestResults
{
    int passed = 0;
    int total = 0;

    void report(const char* name, bool expected, bool actual) 
    {
        bool success = expected == actual;
        std::cout << (success ? "[PASS] " : "[FAIL] ")
            << name
            << " | Expected: "
            << (expected ? "Visible" : "Culled")
            << " | Actual: "
            << (actual ? "Visible" : "Culled")
            << '\n';
        if (success) passed++;
        total++;
    }
    void report(const char* name, bool success)
    {
        std::cout << (success ? "[PASS] " : "[FAIL] ") << name << '\n';
        if (success) passed++;
        total++;
    }
};
#pragma endregion

#pragma region Benchmark Structures
enum class GaussianDistribution
{
    Isotropic, AxisAligned, Rotated, Mixed
};

struct TestGaussian
{
    glm::vec3 position;
    glm::vec3 scale;
    glm::mat3 rotation;
    glm::mat3 covariance;
    float sphereRadius;
    glm::vec3 aabbHalfExtents;
};

struct BenchmarkResult
{
    double milliseconds;
    size_t retained;
    size_t culled;
    double cullingPercentage;
};
#pragma endregion

#pragma region Helpers

#pragma region Covariance Helpers
glm::mat3 createCovariance(const glm::vec3& scale, const glm::mat3& rotation)
{
    glm::mat3 scaleMatrix(0.0f);
    scaleMatrix[0][0] = scale.x * scale.x;
    scaleMatrix[1][1] = scale.y * scale.y;
    scaleMatrix[2][2] = scale.z * scale.z;
    return rotation * scaleMatrix * glm::transpose(rotation);
}
#pragma endregion

bool approximatelyEqual(float expected, float actual, float tolerance = 0.001f)
{
    return glm::abs(expected - actual) <= tolerance;
}
#pragma endregion

#pragma region Gaussian Generation
std::vector <TestGaussian> generateGaussians(size_t count, GaussianDistribution distribution, float cutoff)
{
    std::vector<TestGaussian> gaussians;
    gaussians.reserve(count);
    // Fixed seed for reproducible datasets
    std::mt19937 rng(42);
    std::uniform_real_distribution<float> positionDist(-50.0f, 50.0f);
    std::uniform_real_distribution<float> smallScaleDist(0.02f, 0.2f);
    std::uniform_real_distribution<float> largeScaleDist(0.5f, 2.0f);
    std::uniform_real_distribution<float> isotropicScaleDist(0.05f, 1.0f);
    std::uniform_real_distribution<float> angleDist(0.0f, 360.0f);
    std::uniform_int_distribution<int> axisDist(0, 2);
    std::uniform_int_distribution<int> shapeDist(0, 2);
    for (size_t i = 0; i < count; i++)
    {
        TestGaussian gaussian;
        // Generate world-space position
        gaussian.position.x = positionDist(rng);
        gaussian.position.y = positionDist(rng);
        gaussian.position.z = positionDist(rng);
        gaussian.rotation = glm::mat3(1.0f);
        GaussianDistribution shape = distribution;
        // Randomly select a shape for mixed datasets
        if (distribution == GaussianDistribution::Mixed)
        {
            shape = static_cast<GaussianDistribution>(shapeDist(rng));
        }
        if (shape == GaussianDistribution::Isotropic)
        {
            float scale = isotropicScaleDist(rng);
            gaussian.scale = glm::vec3(scale);
        }
        else
        {
            // Generate an elongated Gaussian
            gaussian.scale.x = smallScaleDist(rng);
            gaussian.scale.y = smallScaleDist(rng);
            gaussian.scale.z = smallScaleDist(rng);
            gaussian.scale[axisDist(rng)] = largeScaleDist(rng);

            if (shape == GaussianDistribution::Rotated)
            {
                glm::mat4 rotation(1.0f);
                rotation = glm::rotate(rotation, glm::radians(angleDist(rng)), glm::vec3(1.0f, 0.0f, 0.0f));
                rotation = glm::rotate(rotation, glm::radians(angleDist(rng)), glm::vec3(0.0f, 1.0f, 0.0f));
                rotation = glm::rotate(rotation, glm::radians(angleDist(rng)), glm::vec3(0.0f, 0.0f, 1.0f));
                gaussian.rotation = glm::mat3(rotation);
            }
        }
        // Precompute covariance
        gaussian.covariance = createCovariance(gaussian.scale, gaussian.rotation);
        // Precompute bounding sphere
        gaussian.sphereRadius = cutoff * std::max({
            gaussian.scale.x,
            gaussian.scale.y,
            gaussian.scale.z
            });
        // Precompute world-space AABB
        gaussian.aabbHalfExtents = cutoff * glm::vec3(
            glm::sqrt(gaussian.covariance[0][0]),
            glm::sqrt(gaussian.covariance[1][1]),
            glm::sqrt(gaussian.covariance[2][2])
        );
        gaussians.push_back(gaussian);
    }
    return gaussians;
}
#pragma endregion

#pragma region Culling Benchmarks
template<typename CullFunction>
BenchmarkResult benchmarkCulling(const std::vector<TestGaussian>& gaussians, CullFunction isVisible, int iterations = 20)
{
    std::vector<double> timings;
    timings.reserve(iterations);
    std::vector<uint32_t> visibleIndices;
    visibleIndices.reserve(gaussians.size());
    size_t retained = 0;
    // Warmup
    for (int iteration = 0; iteration < 3; iteration++)
    {
        visibleIndices.clear();
        for (size_t i = 0; i < gaussians.size(); i++)
        {
            if (isVisible(gaussians[i]))
            {
                visibleIndices.push_back(static_cast<uint32_t>(i));
            }
        }
    }
    // Timed iterations
    for (int iteration = 0; iteration < iterations; iteration++)
    {
        visibleIndices.clear();
        auto start = std::chrono::steady_clock::now();
        for (size_t i = 0; i < gaussians.size(); i++)
        {
            if (isVisible(gaussians[i]))
            {
                visibleIndices.push_back(static_cast<uint32_t>(i));
            }
        }
        auto end = std::chrono::steady_clock::now();
        double milliseconds = std::chrono::duration<double, std::milli>(end - start).count();
        timings.push_back(milliseconds);
        retained = visibleIndices.size();
    }
    // Calculate median execution time
    std::sort(timings.begin(), timings.end());
    size_t middle = timings.size() / 2;
    double median = timings[middle];
    if (timings.size() % 2 == 0)
    {
        median = (timings[middle - 1] + timings[middle]) / 2.0;
    }
    size_t culled = gaussians.size() - retained;
    double cullingPercentage = 100.0 * culled / gaussians.size();
    return { median, retained, culled, cullingPercentage };
}

void printBenchmarkResult(const char* name, const BenchmarkResult& result)
{
    std::cout << std::left << std::setw(14) << name
        << std::right << std::setw(12) << std::fixed << std::setprecision(3) << result.milliseconds
        << std::setw(14) << result.retained
        << std::setw(14) << result.culled
        << std::setw(12) << std::setprecision(2) << result.cullingPercentage
        << "%\n";
}

void benchmarkDataset(const Frustum& frustum, const std::vector<TestGaussian>& gaussians, float cutoff)
{
    BenchmarkResult noCulling = benchmarkCulling(gaussians, [](const TestGaussian& gaussian)
        {
            return true;
        });

    BenchmarkResult point = benchmarkCulling(gaussians, [&](const TestGaussian& gaussian)
        {
            return frustum.containsPoint(gaussian.position);
        });

    BenchmarkResult sphere = benchmarkCulling(gaussians, [&](const TestGaussian& gaussian)
        {
            return frustum.intersectsSphere(gaussian.position, gaussian.sphereRadius);
        });

    BenchmarkResult aabb = benchmarkCulling(gaussians, [&](const TestGaussian& gaussian)
        {
            return frustum.intersectsAABB(gaussian.position, gaussian.aabbHalfExtents);
        });

    BenchmarkResult ellipsoid = benchmarkCulling(gaussians, [&](const TestGaussian& gaussian)
        {
            return frustum.intersectsEllipsoid(gaussian.position, gaussian.covariance, cutoff);
        });
    BenchmarkResult hybrid = benchmarkCulling(gaussians, [&](const TestGaussian& gaussian)
        {
            return frustum.intersectsHybrid(gaussian.position, gaussian.sphereRadius, gaussian.covariance, cutoff);
        });

    std::cout << std::left << std::setw(14) << "Method"
        << std::right << std::setw(12) << "Time (ms)"
        << std::setw(14) << "Retained"
        << std::setw(14) << "Culled"
        << std::setw(12) << "Culled (%)"
        << '\n';

    printBenchmarkResult("No Culling", noCulling);
    printBenchmarkResult("Point", point);
    printBenchmarkResult("Sphere", sphere);
    printBenchmarkResult("AABB", aabb);
    printBenchmarkResult("Ellipsoid", ellipsoid);
    printBenchmarkResult("Hybrid", hybrid);
}

const char* getDistributionName(GaussianDistribution distribution)
{
    switch (distribution)
    {
        case GaussianDistribution::Isotropic: return "Isotropic";
        case GaussianDistribution::AxisAligned: return "Axis-Aligned Anisotropic";
        case GaussianDistribution::Rotated: return "Rotated Anisotropic";
        case GaussianDistribution::Mixed: return "Mixed";
        default: return "Unknown";
    }
}

void runCullingBenchmarks()
{
    std::cout << "\nCulling Performance Benchmarks:\n";
    const float cutoff = 3.0f;
    size_t counts[] = { 10000, 100000, 1000000 };

    GaussianDistribution distributions[] = {
        GaussianDistribution::Isotropic,
        GaussianDistribution::AxisAligned,
        GaussianDistribution::Rotated,
        GaussianDistribution::Mixed
    };

    glm::vec3 cameraPosition(0.0f, 0.0f, 0.0f);
    glm::vec3 cameraUp(0.0f, 1.0f, 0.0f);
    glm::vec3 cameraTargets[] = {
        {0.0f, 0.0f, -1.0f},
        {1.0f, 0.2f, -1.0f}
    };
    const char* cameraNames[] = {
        "Forward Camera",
        "Angled Camera"
    };
    glm::mat4 projection = glm::perspective(glm::radians(90.0f), 1.0f, 0.1f, 100.0f);

    for (GaussianDistribution distribution : distributions)
    {
        for (size_t count : counts)
        {
            std::vector<TestGaussian> gaussians = generateGaussians(count, distribution, cutoff);
            std::cout << "\nDistribution: " << getDistributionName(distribution)
                << "\nGaussian Count: " << count << '\n';
            for (int i = 0; i < 2; i++)
            {
                glm::mat4 view = glm::lookAt(cameraPosition, cameraTargets[i], cameraUp);
                Frustum frustum;
                frustum.update(projection * view);
                std::cout << "\n" << cameraNames[i] << ":\n";
                benchmarkDataset(frustum, gaussians, cutoff);
            }
        }
    }
}
#pragma endregion

// ===============================================
// TESTS
// ===============================================

#pragma region Point-Based Frustum Culling
void runPointTests(const Frustum& frustum, TestResults& results)
{
    std::cout << "\nPoint Culling Tests:\n";
    PointTest tests[] = {
        {"Center", {0.0f, 0.0f, -5.0f}, true},
        {"Inside Left", {-2.0f, 0.0f, -10.0f}, true},
        {"Inside Right", {2.0f, 0.0f, -10.0f}, true},
        {"Outside Left", {-10.0f, 0.0f, -5.0f}, false},
        {"Outside Right", {10.0f, 0.0f, -5.0f}, false},
        {"Outside Top", {0.0f, 10.0f, -5.0f}, false},
        {"Outside Bottom", {0.0f, -10.0f, -5.0f}, false},
        {"Before Near", {0.0f, 0.0f, -0.05f}, false},
        {"Beyond Far", {0.0f, 0.0f, -150.0f}, false},
        {"Behind Camera", {0.0f, 0.0f, 5.0f}, false},
        {"Near Interior", {0.0f, 0.0f, -0.11f}, true},
        {"Far Interior", {0.0f, 0.0f, -99.0f}, true}
    };
    for (const PointTest& test : tests) {
        bool visible = frustum.containsPoint(test.position);
        results.report(test.name, test.expectedVisible, visible);
    }
}
#pragma endregion

#pragma region Bounding Sphere Frustum Culling
void runSphereTests(const Frustum& frustum, TestResults& results) 
{
    std::cout << "\nBounding Sphere Tests:\n";
    SphereTest tests[] = {
        // Fully inside
        {"Center Sphere", {0.0f, 0.0f, -5.0f}, 1.0f, true},
        {"Small Interior Sphere", {1.0f, 1.0f, -5.0f}, 0.1f, true},
        // Center inside, sphere extending outside
        {"Extends Past Right", {4.5f, 0.0f, -5.0f}, 1.0f, true},
        // Center outside, sphere intersects frustum
        {"Intersects Left", {-6.0f, 0.0f, -5.0f}, 1.0f, true},
        {"Intersects Right", {6.0f, 0.0f, -5.0f}, 1.0f, true},
        {"Intersects Top", {0.0f, 6.0f, -5.0f}, 1.0f, true},
        {"Intersects Bottom", {0.0f, -6.0f, -5.0f}, 1.0f, true},
        {"Intersects Near", {0.0f, 0.0f, -0.05f}, 0.1f, true},
        {"Intersects Far", {0.0f, 0.0f, -101.0f}, 2.0f, true},
        // Completely outside
        {"Outside Left", {-10.0f, 0.0f, -5.0f}, 1.0f, false},
        {"Outside Right", {10.0f, 0.0f, -5.0f}, 1.0f, false},
        {"Outside Top", {0.0f, 10.0f, -5.0f}, 1.0f, false},
        {"Outside Bottom", {0.0f, -10.0f, -5.0f}, 1.0f, false},
        {"Outside Near", {0.0f, 0.0f, -0.01f}, 0.01f, false},
        {"Outside Far", {0.0f, 0.0f, -110.0f}, 2.0f, false},
        {"Behind Camera", {0.0f, 0.0f, 5.0f}, 1.0f, false},
        // Zero-radius spheres
        {"Zero Radius Inside", {0.0f, 0.0f, -5.0f}, 0.0f, true},
        {"Zero Radius Outside", {6.0f, 0.0f, -5.0f}, 0.0f, false}
    };
    for (const SphereTest& test : tests) {
        bool visible = frustum.intersectsSphere(test.center, test.radius);
        results.report(test.name, test.expectedVisible, visible);
    }
}
#pragma endregion

#pragma region AABB Frustum Culling
void runAABBTests(const Frustum& frustum, TestResults& results)
{
    std::cout << "\nAABB Culling Tests:\n";
    AABBTest tests[] = {
        // Fully inside
        {"Center Box", {0.0f, 0.0f, -5.0f}, {1.0f, 1.0f, 1.0f}, true},
        {"Small Interior Box", {1.0f, 1.0f, -5.0f}, {0.1f, 0.1f, 0.1f}, true},
        // Center inside, box extending outside
        {"Extends Past Right", {4.5f, 0.0f, -5.0f}, {1.0f, 1.0f, 1.0f}, true},
        // Center outside, box intersects frustum
        {"Intersects Left", {-6.0f, 0.0f, -5.0f}, {1.0f, 1.0f, 1.0f}, true},
        {"Intersects Right", {6.0f, 0.0f, -5.0f}, {1.0f, 1.0f, 1.0f}, true},
        {"Intersects Top", {0.0f, 6.0f, -5.0f}, {1.0f, 1.0f, 1.0f}, true},
        {"Intersects Bottom", {0.0f, -6.0f, -5.0f}, {1.0f, 1.0f, 1.0f}, true},
        {"Intersects Near", {0.0f, 0.0f, -0.05f}, {0.1f, 0.1f, 0.1f}, true},
        {"Intersects Far", {0.0f, 0.0f, -101.0f}, {2.0f, 2.0f, 2.0f}, true},
        // Completely outside
        {"Outside Left", {-10.0f, 0.0f, -5.0f}, {1.0f, 1.0f, 1.0f}, false},
        {"Outside Right", {10.0f, 0.0f, -5.0f}, {1.0f, 1.0f, 1.0f}, false},
        {"Outside Top", {0.0f, 10.0f, -5.0f}, {1.0f, 1.0f, 1.0f}, false},
        {"Outside Bottom", {0.0f, -10.0f, -5.0f}, {1.0f, 1.0f, 1.0f}, false},
        {"Outside Near", {0.0f, 0.0f, -0.01f}, {0.01f, 0.01f, 0.01f}, false},
        {"Outside Far", {0.0f, 0.0f, -110.0f}, {2.0f, 2.0f, 2.0f}, false},
        {"Behind Camera", {0.0f, 0.0f, 5.0f}, {1.0f, 1.0f, 1.0f}, false},
        // Zero-sized boxes
        {"Zero Extents Inside", {0.0f, 0.0f, -5.0f}, {0.0f, 0.0f, 0.0f}, true},
        {"Zero Extents Outside", {6.0f, 0.0f, -5.0f}, {0.0f, 0.0f, 0.0f}, false},
        // Anisotropic boxes
        {"Wide Box Intersecting", {8.0f, 0.0f, -5.0f}, {4.0f, 0.2f, 0.2f}, true},
        {"Tall Box Outside Right", {8.0f, 0.0f, -5.0f}, {0.2f, 4.0f, 0.2f}, false},
        {"Deep Box Intersecting", {8.0f, 0.0f, -5.0f}, {0.2f, 0.2f, 4.0f}, true},
        {"Thin Box Outside Top", {0.0f, 8.0f, -5.0f}, {4.0f, 0.2f, 0.2f}, false}
    };
    for (const AABBTest& test : tests) {
        bool visible = frustum.intersectsAABB(test.center, test.halfExtents);
        results.report(test.name, test.expectedVisible, visible);
    }
}
#pragma endregion

#pragma region Covariance-Based Ellipsoid Frustum Culling
void runEllipsoidTests(const Frustum& frustum, TestResults& results)
{
    std::cout << "\nEllipsoid Culling Tests:\n";
    glm::mat3 identity(1.0f);

    // Isotropic covariance
    glm::mat3 unitCovariance = createCovariance({ 1.0f, 1.0f, 1.0f }, identity);
    glm::mat3 smallCovariance = createCovariance({ 0.1f, 0.1f, 0.1f }, identity);
    glm::mat3 largeCovariance = createCovariance({ 2.0f, 2.0f, 2.0f }, identity);
    // Anisotropic covariance
    glm::mat3 wideCovariance = createCovariance({ 4.0f, 0.2f, 0.2f }, identity);
    glm::mat3 tallCovariance = createCovariance({ 0.2f, 4.0f, 0.2f }, identity);
    glm::mat3 deepCovariance = createCovariance({ 0.2f, 0.2f, 4.0f }, identity);
    // Rotated covariance
    glm::mat3 rotation45 = glm::mat3(glm::rotate(glm::mat4(1.0f), glm::radians(45.0f), glm::vec3(0.0f, 1.0f, 0.0f)));
    glm::mat3 rotationNeg45 = glm::mat3(glm::rotate(glm::mat4(1.0f), glm::radians(-45.0f), glm::vec3(0.0f, 1.0f, 0.0f)));
    glm::mat3 rotatedCovariance = createCovariance({ 4.0f, 0.2f, 0.2f }, rotation45);
    glm::mat3 oppositeCovariance = createCovariance({ 4.0f, 0.2f, 0.2f }, rotationNeg45);

    EllipsoidTest tests[] = {
        // Isotropic ellipsoids behave like spheres
        {"Center Ellipsoid", {0.0f, 0.0f, -5.0f}, unitCovariance, 1.0f, true},
        {"Small Interior Ellipsoid", {1.0f, 1.0f, -5.0f}, smallCovariance, 1.0f, true},
        {"Extends Past Right", {4.5f, 0.0f, -5.0f}, unitCovariance, 1.0f, true},
        // Intersecting frustum planes
        {"Intersects Left", {-6.0f, 0.0f, -5.0f}, unitCovariance, 1.0f, true},
        {"Intersects Right", {6.0f, 0.0f, -5.0f}, unitCovariance, 1.0f, true},
        {"Intersects Top", {0.0f, 6.0f, -5.0f}, unitCovariance, 1.0f, true},
        {"Intersects Bottom", {0.0f, -6.0f, -5.0f}, unitCovariance, 1.0f, true},
        {"Intersects Near", {0.0f, 0.0f, -0.05f}, smallCovariance, 1.0f, true},
        {"Intersects Far", {0.0f, 0.0f, -101.0f}, largeCovariance, 1.0f, true},
        // Completely outside
        {"Outside Left", {-10.0f, 0.0f, -5.0f}, unitCovariance, 1.0f, false},
        {"Outside Right", {10.0f, 0.0f, -5.0f}, unitCovariance, 1.0f, false},
        {"Outside Top", {0.0f, 10.0f, -5.0f}, unitCovariance, 1.0f, false},
        {"Outside Bottom", {0.0f, -10.0f, -5.0f}, unitCovariance, 1.0f, false},
        {"Outside Near", {0.0f, 0.0f, -0.01f}, smallCovariance, 0.1f, false},
        {"Outside Far", {0.0f, 0.0f, -110.0f}, largeCovariance, 1.0f, false},
        {"Behind Camera", {0.0f, 0.0f, 5.0f}, unitCovariance, 1.0f, false},
        // Zero cutoff behaves like a point
        {"Zero Cutoff Inside", {0.0f, 0.0f, -5.0f}, unitCovariance, 0.0f, true},
        {"Zero Cutoff Outside", {6.0f, 0.0f, -5.0f}, unitCovariance, 0.0f, false},
        // Anisotropic ellipsoids
        {"Wide Ellipsoid Intersecting", {8.0f, 0.0f, -5.0f}, wideCovariance, 1.0f, true},
        {"Tall Ellipsoid Outside Right", {8.0f, 0.0f, -5.0f}, tallCovariance, 1.0f, false},
        {"Deep Ellipsoid Intersecting", {8.0f, 0.0f, -5.0f}, deepCovariance, 1.0f, true},
        {"Thin Ellipsoid Outside Top", {0.0f, 8.0f, -5.0f}, wideCovariance, 1.0f, false},
        // Rotated ellipsoids
        {"Rotated Parallel To Plane", {8.0f, 0.0f, -5.0f}, rotatedCovariance, 1.0f, false},
        {"Rotated Toward Plane", {8.0f, 0.0f, -5.0f}, oppositeCovariance, 1.0f, true},
        // Cutoff variations
        {"One Sigma Outside", {8.0f, 0.0f, -5.0f}, unitCovariance, 1.0f, false},
        {"Three Sigma Intersecting", {8.0f, 0.0f, -5.0f}, unitCovariance, 3.0f, true}
    };

    for (const EllipsoidTest& test : tests)
    {
        bool visible = frustum.intersectsEllipsoid(test.center, test.covariance, test.cutoff);
        results.report(test.name, test.expectedVisible, visible);
    }
}
#pragma endregion

#pragma region Hybrid Frustum Culling (Sphere + Ellipsoid)
void runHybridTests(const Frustum& frustum, TestResults& results)
{
    std::cout << "\nHybrid Culling Tests:\n";
    glm::mat3 identity(1.0f);
    glm::mat3 unitCovariance = createCovariance({ 1.0f, 1.0f, 1.0f }, identity);
    glm::mat3 tallCovariance = createCovariance({ 0.2f, 4.0f, 0.2f }, identity);
    glm::mat3 wideCovariance = createCovariance({ 4.0f, 0.2f, 0.2f }, identity);
    glm::mat3 rotation45 = glm::mat3(glm::rotate(glm::mat4(1.0f), glm::radians(45.0f), glm::vec3(0.0f, 1.0f, 0.0f)));
    glm::mat3 rotationNeg45 = glm::mat3(glm::rotate(glm::mat4(1.0f), glm::radians(-45.0f), glm::vec3(0.0f, 1.0f, 0.0f)));
    glm::mat3 rotatedCovariance = createCovariance({ 4.0f, 0.2f, 0.2f }, rotation45);
    glm::mat3 oppositeCovariance = createCovariance({ 4.0f, 0.2f, 0.2f }, rotationNeg45);

    EllipsoidTest tests[] = {
        // Center inside frustum
        {"Center Inside", {0.0f, 0.0f, -5.0f}, unitCovariance, 1.0f, true, 1.0f},
        // Sphere rejection
        {"Sphere Outside Left", {-10.0f, 0.0f, -5.0f}, unitCovariance, 1.0f, false, 1.0f},
        {"Sphere Outside Right", {10.0f, 0.0f, -5.0f}, unitCovariance, 1.0f, false, 1.0f},
        {"Sphere Outside Far", {0.0f, 0.0f, -110.0f}, unitCovariance, 1.0f, false, 1.0f},
        {"Sphere Behind Camera", {0.0f, 0.0f, 5.0f}, unitCovariance, 1.0f, false, 1.0f},
        // Covariance refinement
        {"Intersecting Sphere", {6.0f, 0.0f, -5.0f}, unitCovariance, 1.0f, true, 1.0f},
        {"Tall Ellipsoid Outside", {8.0f, 0.0f, -5.0f}, tallCovariance, 1.0f, false, 4.0f},
        {"Wide Ellipsoid Intersecting", {8.0f, 0.0f, -5.0f}, wideCovariance, 1.0f, true, 4.0f},
        {"Rotated Parallel To Plane", {8.0f, 0.0f, -5.0f}, rotatedCovariance, 1.0f, false, 4.0f},
        {"Rotated Toward Plane", {8.0f, 0.0f, -5.0f}, oppositeCovariance, 1.0f, true, 4.0f},
        // Cutoff variations
        {"Zero Cutoff Outside", {6.0f, 0.0f, -5.0f}, unitCovariance, 0.0f, false, 0.0f},
        {"Three Sigma Intersecting", {8.0f, 0.0f, -5.0f}, unitCovariance, 3.0f, true, 3.0f}
    };
    for (const EllipsoidTest& test : tests)
    {
        bool visible = frustum.intersectsHybrid(test.center, test.sphereRadius, test.covariance, test.cutoff);
        results.report(test.name, test.expectedVisible, visible);
    }
}
#pragma endregion

#pragma region Translated Camera Tests
void runCameraTests(const glm::mat4& projection, TestResults& results)
{
    Frustum frustum;
    glm::vec3 cameraUp(0.0f, 1.0f, 0.0f);

    glm::mat3 unitCovariance = createCovariance({ 1.0f, 1.0f, 1.0f }, glm::mat3(1.0f));
    glm::mat3 tallCovariance = createCovariance({ 0.2f, 4.0f, 0.2f }, glm::mat3(1.0f));
    glm::mat3 deepCovariance = createCovariance({ 0.2f, 0.2f, 4.0f }, glm::mat3(1.0f));

    // Translate camera 10 units along X
    std::cout << "\nTranslated Camera Tests:\n";
    glm::vec3 cameraPosition(10.0f, 0.0f, 0.0f);
    glm::vec3 cameraTarget(10.0f, 0.0f, -1.0f);
    glm::mat4 view = glm::lookAt(cameraPosition, cameraTarget, cameraUp);
    frustum.update(projection * view);

    PointTest translatedPoints[] = {
        {"Point Inside", {10.0f, 0.0f, -5.0f}, true},
        {"Point Outside", {0.0f, 0.0f, -5.0f}, false}
    };

    SphereTest translatedSpheres[] = {
        {"Sphere Intersecting", {16.0f, 0.0f, -5.0f}, 1.0f, true},
        {"Sphere Outside", {20.0f, 0.0f, -5.0f}, 1.0f, false}
    };

    AABBTest translatedBoxes[] = {
        {"Box Intersecting", {16.0f, 0.0f, -5.0f}, {1.0f, 1.0f, 1.0f}, true},
        {"Box Outside", {20.0f, 0.0f, -5.0f}, {1.0f, 1.0f, 1.0f}, false}
    };

    EllipsoidTest translatedEllipsoids[] = {
        {"Ellipsoid Intersecting", {16.0f, 0.0f, -5.0f}, unitCovariance, 1.0f, true},
        {"Ellipsoid Outside", {20.0f, 0.0f, -5.0f}, unitCovariance, 1.0f, false}
    };

    for (const PointTest& test : translatedPoints)
    {
        bool visible = frustum.containsPoint(test.position);
        results.report(test.name, test.expectedVisible, visible);
    }

    for (const SphereTest& test : translatedSpheres)
    {
        bool visible = frustum.intersectsSphere(test.center, test.radius);
        results.report(test.name, test.expectedVisible, visible);
    }

    for (const AABBTest& test : translatedBoxes)
    {
        bool visible = frustum.intersectsAABB(test.center, test.halfExtents);
        results.report(test.name, test.expectedVisible, visible);
    }

    for (const EllipsoidTest& test : translatedEllipsoids)
    {
        bool visible = frustum.intersectsEllipsoid(test.center, test.covariance, test.cutoff);
        results.report(test.name, test.expectedVisible, visible);
    }

    // Rotate camera to face positive X
    std::cout << "\nRotated Camera Tests:\n";
    cameraPosition = glm::vec3(0.0f, 0.0f, 0.0f);
    cameraTarget = glm::vec3(1.0f, 0.0f, 0.0f);
    view = glm::lookAt(cameraPosition, cameraTarget, cameraUp);
    frustum.update(projection * view);

    PointTest rotatedPoints[] = {
        {"Point Inside", {5.0f, 0.0f, 0.0f}, true},
        {"Point Outside", {-5.0f, 0.0f, 0.0f}, false}
    };

    SphereTest rotatedSpheres[] = {
        {"Sphere Intersecting", {5.0f, 0.0f, 6.0f}, 1.0f, true},
        {"Sphere Outside", {5.0f, 0.0f, 10.0f}, 1.0f, false}
    };

    AABBTest rotatedBoxes[] = {
        {"Box Intersecting", {5.0f, 0.0f, 6.0f}, {1.0f, 1.0f, 1.0f}, true},
        {"Box Outside", {5.0f, 0.0f, 10.0f}, {1.0f, 1.0f, 1.0f}, false},
        {"Deep Box Intersecting", {5.0f, 0.0f, 8.0f}, {0.2f, 0.2f, 4.0f}, true},
        {"Tall Box Outside", {5.0f, 0.0f, 8.0f}, {0.2f, 4.0f, 0.2f}, false}
    };

    EllipsoidTest rotatedEllipsoids[] = {
        {"Ellipsoid Intersecting", {5.0f, 0.0f, 6.0f}, unitCovariance, 1.0f, true},
        {"Ellipsoid Outside", {5.0f, 0.0f, 10.0f}, unitCovariance, 1.0f, false},
        {"Deep Ellipsoid Intersecting", {5.0f, 0.0f, 8.0f}, deepCovariance, 1.0f, true},
        {"Tall Ellipsoid Outside", {5.0f, 0.0f, 8.0f}, tallCovariance, 1.0f, false}
    };

    for (const PointTest& test : rotatedPoints)
    {
        bool visible = frustum.containsPoint(test.position);
        results.report(test.name, test.expectedVisible, visible);
    }

    for (const SphereTest& test : rotatedSpheres)
    {
        bool visible = frustum.intersectsSphere(test.center, test.radius);
        results.report(test.name, test.expectedVisible, visible);
    }

    for (const AABBTest& test : rotatedBoxes)
    {
        bool visible = frustum.intersectsAABB(test.center, test.halfExtents);
        results.report(test.name, test.expectedVisible, visible);
    }

    for (const EllipsoidTest& test : rotatedEllipsoids)
    {
        bool visible = frustum.intersectsEllipsoid(test.center, test.covariance, test.cutoff);
        results.report(test.name, test.expectedVisible, visible);
    }
}
#pragma endregion

#pragma region Gaussian Visibility Tests
void runVisibilityTests(const glm::mat4& projection, TestResults& results)
{
    std::cout << "\nGaussian Visibility Tests:\n";
    GaussianVisibility visibility;
    VisibilitySettings settings;
    settings.cutoff = 1.0f;
    glm::mat3 identity(1.0f);
    glm::mat3 unitCovariance = createCovariance({ 1.0f, 1.0f, 1.0f }, identity);
    glm::mat3 wideCovariance = createCovariance({ 4.0f, 0.2f, 0.2f }, identity);
    glm::mat3 tallCovariance = createCovariance({ 0.2f, 4.0f, 0.2f }, identity);
    std::vector<GaussianBounds> gaussians = {
        {{0.0f, 0.0f, -5.0f}, 1.0f, {1.0f, 1.0f, 1.0f}, unitCovariance},
        {{10.0f, 0.0f, -5.0f}, 1.0f, {1.0f, 1.0f, 1.0f}, unitCovariance},
        {{6.0f, 0.0f, -5.0f}, 1.0f, {1.0f, 1.0f, 1.0f}, unitCovariance},
        {{8.0f, 0.0f, -5.0f}, 4.0f, {4.0f, 0.2f, 0.2f}, wideCovariance},
        {{8.0f, 0.0f, -5.0f}, 4.0f, {0.2f, 4.0f, 0.2f}, tallCovariance},
        {{0.0f, 0.0f, 5.0f}, 1.0f, {1.0f, 1.0f, 1.0f}, unitCovariance}
    };
    glm::vec3 cameraPosition(0.0f, 0.0f, 0.0f);
    glm::vec3 cameraTarget(0.0f, 0.0f, -1.0f);
    glm::vec3 cameraUp(0.0f, 1.0f, 0.0f);
    glm::mat4 view = glm::lookAt(cameraPosition, cameraTarget, cameraUp);
    glm::mat4 viewProjection = projection * view;
    VisibilityTest tests[] = {
        {"Culling Disabled", CullingMode::Disabled, {0, 1, 2, 3, 4, 5}},
        {"Point Culling", CullingMode::Point, {0}},
        {"Sphere Culling", CullingMode::Sphere, {0, 2, 3, 4}},
        {"AABB Culling", CullingMode::AABB, {0, 2, 3}},
        {"Ellipsoid Culling", CullingMode::Ellipsoid, {0, 2, 3}},
        {"Hybrid Culling", CullingMode::Hybrid, {0, 2, 3}}
    };
    // Test each culling mode
    for (const VisibilityTest& test : tests)
    {
        settings.cullingMode = test.mode;
        visibility.update(gaussians, viewProjection, settings);
        bool correctIndices = visibility.getVisibleIndices() == test.expectedIndices;
        results.report(test.name, correctIndices);
    }
    // Test visibility statistics
    settings.cullingMode = CullingMode::Sphere;
    visibility.update(gaussians, viewProjection, settings);
    const VisibilityStats& stats = visibility.getStats();
    results.report("Total Gaussian Count", stats.totalGaussians == 6);
    results.report("Visible Gaussian Count", stats.visibleGaussians == 4);
    results.report("Culled Gaussian Count", stats.culledGaussians == 2);
    results.report("Culling Percentage", glm::abs(stats.cullingPercentage - 33.333333) < 0.001);
    results.report("Valid Processing Time", stats.milliseconds >= 0.0);
    // Test camera rotation
    cameraTarget = glm::vec3(0.0f, 0.0f, 1.0f);
    view = glm::lookAt(cameraPosition, cameraTarget, cameraUp);
    visibility.update(gaussians, projection * view, settings);
    results.report("Camera Rotation Update", visibility.getVisibleIndices() == std::vector<uint32_t>{5});
    // Test repeated updates
    visibility.update(gaussians, viewProjection, settings);
    results.report("Visibility Reset", visibility.getVisibleIndices() == std::vector<uint32_t>{0, 2, 3, 4});
    // Test empty dataset
    std::vector<GaussianBounds> emptyGaussians;
    visibility.update(emptyGaussians, viewProjection, settings);
    results.report("Empty Visible Indices", visibility.getVisibleIndices().empty());
    results.report("Empty Total Count", visibility.getStats().totalGaussians == 0);
    results.report("Empty Visible Count", visibility.getStats().visibleGaussians == 0);
    results.report("Empty Culled Count", visibility.getStats().culledGaussians == 0);
    results.report("Empty Culling Percentage", visibility.getStats().cullingPercentage == 0.0);
}
#pragma endregion

#pragma region Screen-Space Sphere Projection Tests (LOD Method)
void runScreenSpaceTests(TestResults& results)
{
    std::cout << "\nScreen-Space Sphere Projection Tests:\n";
    ScreenSpace screenSpace;
    glm::vec3 cameraPosition(0.0f, 0.0f, 0.0f);
    glm::vec3 cameraTarget(0.0f, 0.0f, -1.0f);
    glm::vec3 cameraUp(0.0f, 1.0f, 0.0f);
    glm::mat4 view = glm::lookAt(cameraPosition, cameraTarget, cameraUp);
    glm::mat4 projection = glm::perspective(glm::radians(90.0f), 1920.0f / 1080.0f, 0.1f, 100.0f);
    screenSpace.update(view, projection, 1920, 1080, 0.1f);
    
    ScreenSpaceTest tests[] = {
        // Depth variations
        {"Depth 5", {0.0f, 0.0f, -5.0f}, 1.0f, 108.0f},
        {"Depth 10", {0.0f, 0.0f, -10.0f}, 1.0f, 54.0f},
        {"Depth 20", {0.0f, 0.0f, -20.0f}, 1.0f, 27.0f},
        {"Depth 50", {0.0f, 0.0f, -50.0f}, 1.0f, 10.8f},
        // Radius variations
        {"Half Radius", {0.0f, 0.0f, -10.0f}, 0.5f, 27.0f},
        {"Double Radius", {0.0f, 0.0f, -10.0f}, 2.0f, 108.0f},
        {"Small Radius", {0.0f, 0.0f, -10.0f}, 0.01f, 0.54f},
        {"Zero Radius", {0.0f, 0.0f, -10.0f}, 0.0f, 0.0f},
        // Off-center positions with identical depth
        {"Left Of Center", {-3.0f, 0.0f, -10.0f}, 1.0f, 54.0f},
        {"Right Of Center", {3.0f, 0.0f, -10.0f}, 1.0f, 54.0f},
        {"Above Center", {0.0f, 3.0f, -10.0f}, 1.0f, 54.0f},
        {"Below Center", {0.0f, -3.0f, -10.0f}, 1.0f, 54.0f}
    };
    for (const ScreenSpaceTest& test : tests)
    {
        float projectedRadius = screenSpace.estimateSphereRadius(test.position, test.sphereRadius);
        results.report(test.name, approximatelyEqual(test.expectedRadius, projectedRadius));
    }
    
    // Near-plane and invalid projection cases
    results.report("Near Plane Intersection", std::isinf(screenSpace.estimateSphereRadius({ 0.0f, 0.0f, -0.5f }, 1.0f)));
    results.report("Behind Camera", std::isinf(screenSpace.estimateSphereRadius({ 0.0f, 0.0f, 5.0f }, 1.0f)));
    results.report("At Camera", std::isinf(screenSpace.estimateSphereRadius({ 0.0f, 0.0f, 0.0f }, 1.0f)));
    results.report("Negative Radius", std::isinf(screenSpace.estimateSphereRadius({ 0.0f, 0.0f, -10.0f }, -1.0f)));
    
    // Resolution variations
    screenSpace.update(view, projection, 1280, 720, 0.1f);
    results.report("720p Projection", approximatelyEqual(36.0f, screenSpace.estimateSphereRadius({ 0.0f, 0.0f, -10.0f }, 1.0f)));
    screenSpace.update(view, projection, 2560, 1440, 0.1f);
    results.report("1440p Projection", approximatelyEqual(72.0f, screenSpace.estimateSphereRadius({ 0.0f, 0.0f, -10.0f }, 1.0f)));
    screenSpace.update(view, projection, 3840, 2160, 0.1f);
    results.report("4K Projection", approximatelyEqual(108.0f, screenSpace.estimateSphereRadius({ 0.0f, 0.0f, -10.0f }, 1.0f)));
    
    // Field-of-view variations
    glm::mat4 narrowProjection = glm::perspective(glm::radians(60.0f), 1920.0f / 1080.0f, 0.1f, 100.0f);
    screenSpace.update(view, narrowProjection, 1920, 1080, 0.1f);
    results.report("Narrow FOV", approximatelyEqual(93.5307f, screenSpace.estimateSphereRadius({ 0.0f, 0.0f, -10.0f }, 1.0f)));
    glm::mat4 wideProjection = glm::perspective(glm::radians(120.0f), 1920.0f / 1080.0f, 0.1f, 100.0f);
    screenSpace.update(view, wideProjection, 1920, 1080, 0.1f);
    results.report("Wide FOV", approximatelyEqual(31.1769f, screenSpace.estimateSphereRadius({ 0.0f, 0.0f, -10.0f }, 1.0f)));
    
    // Camera translation
    cameraPosition = glm::vec3(0.0f, 0.0f, -5.0f);
    cameraTarget = glm::vec3(0.0f, 0.0f, -6.0f);
    view = glm::lookAt(cameraPosition, cameraTarget, cameraUp);
    screenSpace.update(view, projection, 1920, 1080, 0.1f);
    results.report("Translated Camera", approximatelyEqual(108.0f, screenSpace.estimateSphereRadius({ 0.0f, 0.0f, -10.0f }, 1.0f)));
    
    // Camera rotation
    cameraPosition = glm::vec3(0.0f, 0.0f, 0.0f);
    cameraTarget = glm::vec3(1.0f, 0.0f, 0.0f);
    view = glm::lookAt(cameraPosition, cameraTarget, cameraUp);
    screenSpace.update(view, projection, 1920, 1080, 0.1f);
    results.report("Rotated Camera", approximatelyEqual(54.0f, screenSpace.estimateSphereRadius({ 10.0f, 0.0f, 0.0f }, 1.0f)));
    
    // Invalid viewport dimensions
    screenSpace.update(view, projection, 0, 1080, 0.1f);
    results.report("Invalid Viewport Width", std::isinf(screenSpace.estimateSphereRadius({ 10.0f, 0.0f, 0.0f }, 1.0f)));
    screenSpace.update(view, projection, 1920, 0, 0.1f);
    results.report("Invalid Viewport Height", std::isinf(screenSpace.estimateSphereRadius({ 10.0f, 0.0f, 0.0f }, 1.0f)));
}
#pragma endregion

#pragma region Main
int main()
{
    std::cout << "visibility optimization tests:\n";

    glm::vec3 cameraPosition(0.0f, 0.0f, 0.0f);
    glm::vec3 cameraTarget(0.0f, 0.0f, -1.0f);
    glm::vec3 cameraUp(0.0f, 1.0f, 0.0f);

    glm::mat4 view = glm::lookAt(cameraPosition, cameraTarget, cameraUp);
    glm::mat4 projection = glm::perspective(glm::radians(90.0f), 1.0f, 0.1f, 100.0f);
    glm::mat4 viewProjection = projection * view;

    // Construct the viewing frustum
    Frustum frustum;
    frustum.update(viewProjection);

    // Run Visibility Tests
    TestResults results;

    runPointTests(frustum, results);
    runSphereTests(frustum, results);
    runAABBTests(frustum, results);
    runEllipsoidTests(frustum, results);
    runHybridTests(frustum, results);
    runCameraTests(projection, results);
    runVisibilityTests(projection, results);
    runScreenSpaceTests(results);

    std::cout << "\nResults: "
        << results.passed << "/" << results.total
        << " tests passed.\n";

    if (results.passed != results.total) return 1;

    //runCullingBenchmarks();
}
#pragma endregion
