#include "Frustum.h"

float Frustum::Plane::signedDistance(const glm::vec3& point) const
{
	return glm::dot(normal, point) + distance;
}

Frustum::Plane Frustum::normalizePlane(const glm::vec4& coefficients)
{
	glm::vec3 normal(coefficients.x, coefficients.y, coefficients.z);
	float length = glm::length(normal);
	return Plane{ normal / length, coefficients.w / length };
}

void Frustum::update(const glm::mat4& viewProjection)
{
	// GLM matrices are indexed as [column][row]
	// Transpose the matrix to access its original rows
	glm::mat4 rows = glm::transpose(viewProjection);
	const glm::vec4& row0 = rows[0];
	const glm::vec4& row1 = rows[1];
	const glm::vec4& row2 = rows[2];
	const glm::vec4& row3 = rows[3];
	
	// Extract and normalize the 6 frustum planes
	planes[Left] = normalizePlane(row3 + row0);
	planes[Right] = normalizePlane(row3 - row0);
	planes[Bottom] = normalizePlane(row3 + row1);
	planes[Top] = normalizePlane(row3 - row1);
	planes[Near] = normalizePlane(row3 + row2);
	planes[Far] = normalizePlane(row3 - row2);
}

bool Frustum::containsPoint(const glm::vec3& point) const
{
	for (const Plane& plane : planes) 
	{
		if (plane.signedDistance(point) < 0.0f) return false;
	}
	return true;
}