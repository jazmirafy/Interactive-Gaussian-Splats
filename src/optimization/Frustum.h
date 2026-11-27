#pragma once

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

	// Point-based frustum culling
	bool containsPoint(const glm::vec3& point) const;

	// Bounding sphere frustum culling
	bool intersectsSphere(const glm::vec3& center, float radius) const;

	// AABB frustum culling (Center & Half Extents Method)
	bool intersectsAABB(const glm::vec3& center, const glm::vec3& halfExtents) const;

	// Covariance-based ellipsoid frustum culling
	bool intersectsEllipsoid(const glm::vec3& center, const glm::mat3& covariance, float cutoff) const;

	// Hybrid sphere/ellipsoid frustum culling
	bool intersectsHybrid(const glm::vec3& center, float sphereRadius, const glm::mat3& covariance, float cutoff) const;

private:
	std::array<Plane, PlaneCount> planes{};

	// Convert and normalize plane coefficients
	static Plane normalizePlane(const glm::vec4& coefficients);
};