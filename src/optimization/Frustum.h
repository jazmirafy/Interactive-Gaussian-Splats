#include <array>
#include <glm/glm.hpp>

class Frustum 
{
public:
	// Stores a normal vector & offset
	struct Plane 
	{
		glm::vec3 normal;
		float distance;
		float signedDistance(const glm::vec3& point) const;
	};
	enum PlaneIndex
	{
		Left = 0, Right, Bottom, Top, Near, Far, PlaneCount
	};
	// Extract the frustum planes from a view-projection matrix
	void update(const glm::mat4& viewProjection);

	// Determine whether a point is inside the frustum (visibility check)
	bool containsPoint(const glm::vec3& point) const;
private:
	std::array<Plane, PlaneCount> planes{};

	// Convert and normalize plane coefficients
	static Plane normalizePlane(const glm::vec4& coefficients);
};