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

void Force::Drag(Particle& p, vec3 vAir)
{
	auto v = p.Velocity() - vAir;
	float density = 1.0f;
	float c = 0.47f;
	//float pi = 3.14f;						// Leaving this here to show I attempted calculating the area properly. 
	//float area = pi * (0.05f * 0.05f);	// The results were disastrous.
	float area = 1.0f;

	auto vMag = glm::length(v);
	
	vec3 e;
	if (vMag != 0.0f)
		e = - glm::normalize(v);

	auto force = 0.5f * density * (vMag * vMag) * c * area * e;
	p.ApplyForce(force);
}

void Force::Drag(Particle& p1, Particle& p2, Particle& p3, vec3 vAir)
{
	float density = 1.0f;
	float c = 0.47f;

	auto v1 = p1.Velocity(), v2 = p2.Velocity(), v3 = p3.Velocity();
	auto vSurface = (v1 + v2 + v3) / 3.0f;
	auto v = vSurface - vAir;
	
	auto r1 = p1.Position(), r2 = p2.Position(), r3 = p3.Position();
	auto x = cross(r2 - r1, r3 - r1);

	vec3 n;
	if (length(x) != 0.0f)
		n = x / length(x);

	float a = 0.5f * length(x);
	auto vMag = length(v);

	float area = 0.0f;
	if (vMag != 0.0f)
		area = a * (dot(v, n) / vMag);

	auto f = 0.5f * density * vMag * vMag * c * area * n;
	auto force = f / 3.0f;

	p1.ApplyForce(force);
	p2.ApplyForce(force);
	p3.ApplyForce(force);
}

void Force::Hooke(Particle& p1, Particle& p2, float restLength, float ks, float kd)
{
	auto l = length(p2.Position() - p1.Position());

	vec3 e;
	if (l != 0.0f)
		e = normalize(p2.Position() - p1.Position());

	auto v1 = dot(e, p1.Velocity());
	auto v2 = dot(e, p2.Velocity());
	auto fsd1 = -ks * (restLength - l) - kd * v1;
	auto fsd2 = -ks * (restLength - l) - kd * v2;
	auto f1 = fsd1 * e;
	auto f2 = -fsd2 * e;

	// Both disabled code blocks below print the same information.
	// Enabling either has a major impact on performance.

	/*
	printf("l=%f, e=(%f %f %f)\n", l, e.x, e.y, e.z);
	printf("v1=%f, v2=%f\n", v1, v2);
	printf("fsd1=%f, fsd2=%f\n", fsd1, fsd2);
	printf("f1=(%f, %f, %f), f2=(%f, %f, %f)\n\n", f1.x, f1.y, f1.z, f2.x, f2.y, f2.z);
	*/

	/*
	std::cout << "l=" << l << ", e=" << e << std::endl;
	std::cout << "v1=" << v1 << ", v2=" << v2 << std::endl;
	std::cout << "fsd1=" << fsd1 << ", fsd2=" << fsd2 << std::endl;
	std::cout << "f1=" << f1 << ", f2=" << f2 << std::endl;
	*/

	p1.ApplyForce(f1);
	p2.ApplyForce(f2);
}