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
	Task4
};

class PhysicsEngine
{
public:

	void StandardInit();
	void StandardUpdate(float deltaTime, float totalTime);
	void StandardDisplay(const glm::mat4& viewMatrix, const glm::mat4& projMatrix);

	void ExtendedInit();
	void ExtendedUpdate(float deltaTime, float totalTime);
	void ExtendedDisplay(const glm::mat4& viewMatrix, const glm::mat4& projMatrix);

	void Task3Init();
	void Task3Update(float deltaTime, float totalTime);
	void Task3Display(const glm::mat4& viewMatrix, const glm::mat4& projMatrix);

	void Task4Init();
	void Task4Update(float deltaTime, float totalTime);
	void Task4Display(const glm::mat4& viewMatrix, const glm::mat4& projMatrix);

	void Init(Camera& camera, MeshDb& meshDb, ShaderDb& shaderDb);
	void Update(float deltaTime, float totalTime);
	void Display(const glm::mat4& viewMatrix, const glm::mat4& projMatrix);
	void LoadDemo(Demo task);

	void HandleInputKey(int keyCode, bool pressed);

private:

	PhysicsBody ground;

	Particle task1Particle;
	Particle task2Particles[4];
	Particle task3Particles[4];
};