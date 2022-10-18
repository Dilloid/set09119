#pragma once

#include <glm/glm.hpp>

class Particle;

class Force
{
public:
	static void Gravity(Particle& p);
	static void Drag(Particle& p, glm::vec3 vAir);
	static void Drag(Particle& p1, Particle& p2, Particle& p3, glm::vec3 vAir);
	static void Hooke(Particle& p1, Particle& p2, float restLength, float ks, float kd);
private:
};