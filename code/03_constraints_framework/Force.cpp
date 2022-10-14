#include "PhysicsEngine.h"
#include "Force.h"
#include "PhysicsObject.h"
#include <iostream>
#include <glm/glm.hpp>
#include <glm/gtx/io.hpp>

using namespace glm;

std::ostream& operator<< (std::ostream& out, const glm::vec3& vec) {
	out << "{" << vec.x << " " << vec.y << " " << vec.z << "}";
	return out;
}

void Force::Gravity(Particle& p)
{
	auto force = vec3(0, -9.81, 0) * p.Mass();
	p.ApplyForce(force);
}

void Force::Drag(Particle& p)
{
	float density = 1.0f;
	float cd = 0.47f;
	float pi = 3.1415f;

	vec3 v = p.Velocity();

	auto force = 0.5f * density * glm::length(v) * cd * (pi * (0.05f * 0.05f)) * (-v);
	p.ApplyForce(force);
}

void Force::Hooke(Particle& p1, Particle& p2, float restLength, float ks, float kd)
{
	auto l = glm::length(p2.Position() - p1.Position());
	auto e = glm::normalize(p2.Position() - p1.Position());
	auto v1 = glm::dot(e, p1.Velocity());
	auto v2 = glm::dot(e, p2.Velocity());
	auto fsd1 = (-ks * (restLength - l)) -(kd * v1);
	auto fsd2 = (-ks * (restLength - l)) -(kd * v2);
	auto f1 = fsd1 * e;
	auto f2 = -fsd2 * e;

	/*
	std::cout << "l=" << l << ", e=" << e << std::endl;
	std::cout << "v1=" << v1 << ", v2=" << v2 << std::endl;
	std::cout << "fsd1=" << fsd1 << ", fsd2=" << fsd2 << std::endl;
	std::cout << "f1=" << f1 << ", f2=" << f2 << std::endl;
	*/

	p1.ApplyForce(f1);
	p2.ApplyForce(f2);
}