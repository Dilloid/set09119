#pragma once


#include <glm/glm.hpp>

#include "PhysicsObject.h"

// Fwd declaration
class MeshDb;
class ShaderDb;
class Camera;

enum Demo {
	Task1,
	Task2,
	Task3,
	Task4,
	Task5
};

class PhysicsEngine
{
public:
	void Init(Camera& camera, MeshDb& meshDb, ShaderDb& shaderDb);
	void Update(float deltaTime, float totalTime);
	void Display(const glm::mat4& viewMatrix, const glm::mat4& projMatrix);
	void HandleInputKey(int keyCode, bool pressed);

	void Task123Init();
	void Task123Update(float deltaTime, float totalTime);
	void Task123Display(const glm::mat4& viewMatrix, const glm::mat4& projMatrix);
	
	void Task4Init();
	void Task4Update(float deltaTime, float totalTime);
	
	void Task5Init();
	void Task5Update(float deltaTime, float totalTime);

	void Task45Display(const glm::mat4& viewMatrix, const glm::mat4& projMatrix);

private:
	const int TASK123LENGTH = 10;
	const int TASK45HEIGHT = 10;
	const int TASK45LENGTH = 10;
	const float RESTLENGTH = 1.0f;
	const float TENSION = 30.0f;

	PhysicsBody ground;
	Particle t123Particles[10], t45Particles[10][10];
};