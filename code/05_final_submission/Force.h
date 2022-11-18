#pragma once

#include <glm/glm.hpp>

class Particle;

class Force
{
public:
	static void Gravity(Particle& p);
	static void Drag(Particle& p, glm::vec3 vAir);
private:
};