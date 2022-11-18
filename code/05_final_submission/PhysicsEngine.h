#pragma once

#include <glm/glm.hpp>

#include "PhysicsObject.h"

// Fwd declaration
class MeshDb;
class ShaderDb;
class Camera;

enum Demo {
	Standard,
	Extended,
};

class PhysicsEngine
{
public:
	void Init(Camera& camera, MeshDb& meshDb, ShaderDb& shaderDb);
	void Update(float deltaTime, float totalTime);
	void Display(const glm::mat4& viewMatrix, const glm::mat4& projMatrix);
	void HandleInputKey(int keyCode, bool pressed);

	void StandardInit();
	void StandardUpdate(float deltaTime, float totalTime);
	void StandardDisplay(const glm::mat4& viewMatrix, const glm::mat4& projMatrix);

	void ExtendedInit();
	void ExtendedUpdate(float deltaTime, float totalTime);
	void ExtendedDisplay(const glm::mat4& viewMatrix, const glm::mat4& projMatrix);

private:
	RigidBody table, spheres[];
};