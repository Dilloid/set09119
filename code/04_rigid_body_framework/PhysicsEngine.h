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

	void Task1Init();
	void Task1Update(float deltaTime, float totalTime);
	void Task1Display(const glm::mat4& viewMatrix, const glm::mat4& projMatrix);

	void Task2Init();
	void Task2Update(float deltaTime, float totalTime);
	void Task2Display(const glm::mat4& viewMatrix, const glm::mat4& projMatrix);
	
	void Task3Init();
	void Task3Update(float deltaTime, float totalTime);
	void Task3Display(const glm::mat4& viewMatrix, const glm::mat4& projMatrix);

private:
	PhysicsBody ground;
	RigidBody rbody1, rbody2, rbody3;
};