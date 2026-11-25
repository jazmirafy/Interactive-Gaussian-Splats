#include <iostream>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

#include "optimization/Frustum.h"

struct PointTest
{
    const char* name;
    glm::vec3 position;
    bool expectedVisible;
};

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

    // Test points against the frustum (dummy data)
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

    int passed = 0;
    int total = 0;

    for (const PointTest& test : tests)
    {
        bool visible = frustum.containsPoint(test.position);
        bool success = visible == test.expectedVisible;
        std::cout << (success ? "[PASS] " : "[FAIL] ")
            << test.name
            << " | Expected: "
            << (test.expectedVisible ? "Visible" : "Culled")
            << " | Actual: "
            << (visible ? "Visible" : "Culled")
            << '\n';
        if (success) passed++;
        total++;
    }

    std::cout << "\nResults: "
        << passed << "/" << total
        << " tests passed.\n";

    return passed == total ? 0 : 1;

    //std::cout << "View matrix created successfully.\n";
    //std::cout << "Projection matrix created successfully.\n";
    //std::cout << "View-projection matrix created successfully.\n";

    //return 0;
}