#include "PhysicsEngine.h"

#include <map>
#include <list>
#include <numeric>
#include <iostream>

#include "Application.h"
#include "Camera.h"
#include "Force.h"

#include <glm/glm.hpp>
#include <glm/gtx/io.hpp>
#include <glm/gtx/matrix_cross_product.hpp>
#include <glm/gtx/orthonormalize.hpp>

using namespace glm;

const glm::vec3 GRAVITY = glm::vec3(0, -9.81f, 0);

bool paused = false;
bool accelerating = true;
glm::mat4 upright;

Demo activeDemo = Standard;

void ExplicitEuler(vec3& pos, vec3& vel, float mass, const vec3& accel, const vec3& impulse, float dt)
{
	vec3 v = vel + (impulse / mass);
	vel += dt * accel + (impulse / mass);
	pos += dt * v;
}

void SymplecticEuler(vec3& pos, vec3& vel, float mass, const vec3& accel, const vec3& impulse, float dt)
{
	vel += (1.0f / mass) * ((accel * dt) + impulse);
	pos += vel * dt;
}

void Integrate(RigidBody& rb, float dt, vec3 angAcc)
{
	// dt: deltaTime , angAcc : angular acceleration
	// integration ( rotation )
	auto newAngVel = rb.AngularVelocity() + dt * angAcc;
	//rb.SetAngularVelocity(newAngVel);

	// create skew symmetric matrix for w
	glm::mat3 angVelSkew = glm::matrixCross3(newAngVel);

	// create 3x3 rotation matrix from rigidBody matrix
	glm::mat3 R = glm::mat3(rb.Orientation());

	// update rotation matrix
	R += dt * angVelSkew * R;
	R = glm::orthonormalize(R);

	rb.SetOrientation(glm::mat4(R));
}

void CollisionImpulse(RigidBody& rb, float coefficientOfRestitution)
{
	vec3 impulse{ 0.0f };

	vec3 lowestPoint = vec3(0.0f);

	for (vec3 pos : rb.GetMesh()->Data().positions.data)
	{
		vec4 localPos = vec4(pos, 1);
		vec3 worldPos = rb.ModelMatrix() * localPos;

		if (worldPos.y < 0.0f)
		{
			if (worldPos.y < lowestPoint.y)
				lowestPoint = worldPos;
		}
	}

	if (lowestPoint.y < 0.0f)
	{
		rb.SetPosition(vec3(rb.Position().x, rb.Position().y - lowestPoint.y, rb.Position().z));

		float e = coefficientOfRestitution;
		vec3 floorN = vec3(0.0f, 1.0f, 0.0f);

		float vClose = glm::dot(rb.Velocity(), floorN);
		impulse = -(1 + e) * rb.Mass() * vClose * floorN;
	}
	
	rb.ApplyImpulse(impulse);
}

float CollisionImpulseJr(RigidBody& rb, vec3& lowestPoint)
{
	float jr = 0.0f;
	std::list<vec3> contactPoints;
	contactPoints.clear();

	for (vec3 pos : rb.GetMesh()->Data().positions.data)
	{
		vec4 localPos = vec4(pos, 1);
		vec3 worldPos = rb.ModelMatrix() * localPos;

		if (worldPos.y < 0.0f)
		{
			float threshold = 0.005f;
			if (worldPos.y > lowestPoint.y - threshold && worldPos.y < lowestPoint.y + threshold)
				contactPoints.push_back(worldPos);

			if (worldPos.y < lowestPoint.y)
			{
				contactPoints.clear();
				contactPoints.push_back(worldPos);
				lowestPoint = worldPos;
			}

		}
	}

	if (lowestPoint.y < 0.0f)
	{
		float contacts = contactPoints.size();

		if (contacts > 1)
		{
			vec3 point = vec3(0.0f);

			for (vec3 p : contactPoints)
				point += p;

			lowestPoint = point / contacts;
		}

		contactPoints.clear();
		rb.Translate(vec3(0.0f, -lowestPoint.y, 0.0f));

		float e = rb.CoefficientOfRestitution();

		vec3 v1 = rb.Velocity();
		vec3 w1 = rb.AngularVelocity();
		vec3 r1 = lowestPoint - rb.Position();

		vec3 v2 = vec3(0.0f);
		vec3 w2 = vec3(0.0f);
		vec3 r2 = lowestPoint - vec3(0.0f);

		vec3 vr = v1 + cross(w1, r1) - (v2 + cross(w2, r2));

		vec3 floorN = vec3(0.0f, 1.0f, 0.0f);

		float m = rb.Mass();
		float m1i = 1.0f / m;

		mat3 ii = rb.InverseInertia();

		// jr = -(1 + e)vr . floorN / m1i + floorN . (ii(r1 x floorN) x r1)
		vec3 c1 = cross(r1, floorN), c2 = cross((ii * c1), r1);
		jr = -(1 + e) * dot(vr, floorN) / (m1i + dot(floorN, c2));
	}

	return jr;
}

// This is called once
void PhysicsEngine::Init(Camera& camera, MeshDb& meshDb, ShaderDb& shaderDb)
{
	// Get a few meshes/shaders from the databases
	auto defaultShader = shaderDb.Get("default");
	auto groundMesh = meshDb.Get("plane");

	meshDb.Add("cube", Mesh(MeshDataFromWavefrontObj("resources/models/cube.obj")));
	meshDb.Add("sphere", Mesh(MeshDataFromWavefrontObj("resources/models/sphere.obj")));
	meshDb.Add("cone", Mesh(MeshDataFromWavefrontObj("resources/models/cone.obj")));
	auto mesh = meshDb.Get("cube");

	// Initialise objects

	StandardInit();

	// TODO: Get the mesh and shader for rigidy body
	camera = Camera(vec3(0, 5, 10));
}

void PhysicsEngine::StandardInit()
{
	
}

void PhysicsEngine::StandardUpdate(float deltaTime, float totalTime)
{
	
}

void PhysicsEngine::StandardDisplay(const mat4& viewMatrix, const mat4& projMatrix)
{
	
}

void PhysicsEngine::ExtendedInit()
{

}

void PhysicsEngine::ExtendedUpdate(float deltaTime, float totalTime)
{
	
}

void PhysicsEngine::ExtendedDisplay(const mat4& viewMatrix, const mat4& projMatrix)
{

}

// This is called every frame
void PhysicsEngine::Update(float deltaTime, float totalTime)
{
	if (!paused)
	{
		switch (activeDemo)
		{
		default:
		case Standard:
			StandardUpdate(deltaTime, totalTime);
			break;
		case Extended:
			ExtendedUpdate(deltaTime, totalTime);
			break;
		}
	}
}

// This is called every frame, after Update
void PhysicsEngine::Display(const mat4& viewMatrix, const mat4& projMatrix)
{
	// Draw table

	switch (activeDemo)
	{
	default:
	case Standard:
		StandardDisplay(viewMatrix, projMatrix);
		break;
	case Extended:
		ExtendedDisplay(viewMatrix, projMatrix);
		break;
	}
}

void PhysicsEngine::HandleInputKey(int keyCode, bool pressed)
{
	switch (keyCode)
	{
	case GLFW_KEY_1:
		printf("Key 1 was %s\n", pressed ? "pressed" : "released");
		StandardInit();
		activeDemo = Standard;
		break;
	case GLFW_KEY_2:
		printf("Key 2 was %s\n", pressed ? "pressed" : "released");
		ExtendedInit();
		activeDemo = Extended;
		break;
	case GLFW_KEY_P:
		printf("Key P was %s\n", pressed ? "pressed" : "released");
		if (!pressed)
			paused = !paused;
		break;
	case GLFW_KEY_K:
		printf("Key K was %s\n", pressed ? "pressed" : "released");
		if (!pressed)
			accelerating = !accelerating;
		break;
	default:
		break;
	}
}