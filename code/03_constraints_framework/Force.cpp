#include "PhysicsEngine.h"
#include "Force.h"
#include "PhysicsObject.h"
#include <iostream>

using namespace glm;

void Force::Gravity(Particle& p)
{
	auto force = vec3(0, -9.81, 0) * p.Mass();
	p.ApplyForce(force);
}

void Force::Drag(Particle& p)
{
	vec3 v = p.Velocity();
	auto force = 0.5f * 1.0f * glm::length(v) * 0.47f * (3.14f * (0.05f * 0.05f)) * (-v);
	p.ApplyForce(force);
}

void Force::Hooke(Particle& p1, Particle& p2, float restLength, float ks, float kd)
{
	auto l = glm::length(p2.Position() - p1.Position());
	auto e = (p2.Position() - p1.Position()) / l;
	auto v1 = glm::dot(e, p1.Velocity());
	auto v2 = glm::dot(e, p2.Velocity());
	auto fsd1 = (-ks * (restLength - l)) - kd * v1;
	auto fsd2 = (-ks * (restLength - l)) - kd * v2;
	auto f1 = fsd1 * e;
	auto f2 = -fsd2 * e;

	std::cout << "l = " << l << std::endl;

	p1.ApplyForce(f1);
	p2.ApplyForce(f2);
}