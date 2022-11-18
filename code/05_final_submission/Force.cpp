#include "PhysicsEngine.h"
#include "Force.h"
#include "PhysicsObject.h"

using namespace glm;

void Force::Gravity(Particle& p)
{
	auto force = vec3(0, -9.81, 0) * p.Mass();
	p.ApplyForce(force);
}

void Force::Drag(Particle& p, vec3 vAir)
{
	auto v = p.Velocity() - vAir;
	float density = 1.0f;
	float c = 0.47f;
	float area = 1.0f;

	auto vMag = glm::length(v);

	vec3 e;
	if (vMag != 0.0f)
		e = -glm::normalize(v);

	auto force = 0.5f * density * (vMag * vMag) * c * area * e;
	p.ApplyForce(force);
}