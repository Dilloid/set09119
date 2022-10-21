#include "PhysicsEngine.h"

#include <map>
#include <numeric>

#include "Application.h"
#include "Camera.h"
#include "Force.h"

#include <glm/gtx/matrix_cross_product.hpp>
#include <glm/gtx/orthonormalize.hpp>

using namespace glm;

const glm::vec3 GRAVITY = glm::vec3(0, -9.81f, 0);

bool paused = false;
bool accelerating = true;

Demo activeDemo = Task2;

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
	rb.SetAngularVelocity(newAngVel);

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
	vec3 floorN{ 0.0f, 1.0f, 0.0f };

	vec3 pos = rb.Position();
	float e = coefficientOfRestitution;

	if (pos.y < rb.Scale().y)
	{
		rb.SetPosition(vec3(pos.x, rb.Scale().y, pos.z));
		float vClose = glm::dot(rb.Velocity(), floorN);
		impulse = -(1 + e) * rb.Mass() * vClose * floorN;
	}

	rb.ApplyImpulse(impulse);
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

	// Initialise ground
	ground.SetMesh(groundMesh);
	ground.SetShader(defaultShader);
	ground.SetScale(vec3(10.0f));

	rbody1.SetMesh(mesh);
	rbody1.SetShader(defaultShader);
	
	rbody2.SetMesh(mesh);
	rbody2.SetShader(defaultShader);
	
	rbody3.SetMesh(mesh);
	rbody3.SetShader(defaultShader);

	Task2Init();

	// TODO: Get the mesh and shader for rigidy body
	camera = Camera(vec3(0, 5, 10));
}

void PhysicsEngine::Task1Init()
{
	rbody1.SetColor(vec4(1, 0, 0, 1));
	rbody1.SetPosition(vec3(0, 5, 0));
	rbody1.SetScale(vec3(1, 2, 1));
	rbody1.SetVelocity(vec3(1.0f, 0.0f, 5.0f));
}

void PhysicsEngine::Task1Update(float deltaTime, float totalTime)
{
	rbody1.ClearForcesImpulses();

	CollisionImpulse(rbody1, 1.0f);

	vec3 p = rbody1.Position();
	vec3 v = rbody1.Velocity();
	
	Force::Gravity(rbody1);
	Force::Drag(rbody1, vec3{ 0 });

	vec3 acceleration = rbody1.AccumulatedForce() / rbody1.Mass();

	SymplecticEuler(p, v, rbody1.Mass(), acceleration, rbody1.AccumulatedImpulse(), deltaTime);

	rbody1.SetPosition(p);
	rbody1.SetVelocity(v);
}

void PhysicsEngine::Task1Display(const mat4& viewMatrix, const mat4& projMatrix)
{
	rbody1.Draw(viewMatrix, projMatrix);
}

void PhysicsEngine::Task2Init()
{
	rbody2.SetColor(vec4(1, 0, 0, 1));
	rbody2.SetPosition(vec3(-2, 5, 0));
	rbody2.SetScale(vec3(1, 2, 1));
	rbody2.SetAngularVelocity(vec3(0, 10.0f, 0));

	rbody3.SetColor(vec4(1, 0, 0, 1));
	rbody3.SetPosition(vec3(2, 5, 0));
	rbody3.SetScale(vec3(1, 2, 1));
}

void PhysicsEngine::Task2Update(float deltaTime, float totalTime)
{
	rbody2.ClearForcesImpulses();
	rbody3.ClearForcesImpulses();

	// ##################################################

	CollisionImpulse(rbody2, 0.8f);

	vec3 p = rbody2.Position(), v = rbody3.Velocity();

	Force::Drag(rbody2, vec3{ 0 });

	vec3 acceleration = rbody2.AccumulatedForce() / rbody2.Mass();

	SymplecticEuler(p, v, rbody2.Mass(), acceleration, rbody2.AccumulatedImpulse(), deltaTime);

	Integrate(rbody2, deltaTime, vec3(0, 0, 0));

	rbody2.SetPosition(p);
	rbody2.SetVelocity(v);

	// ##################################################

	vec3 angularAcceleration;
	angularAcceleration = accelerating ? vec3(0, 1, 0) : vec3(0, 0, 0);

	CollisionImpulse(rbody3, 0.8f);

	p = rbody3.Position(), v = rbody3.Velocity();

	Force::Drag(rbody3, vec3{ 0 });

	acceleration = rbody3.AccumulatedForce() / rbody3.Mass();

	SymplecticEuler(p, v, rbody3.Mass(), acceleration, rbody3.AccumulatedImpulse(), deltaTime);

	Integrate(rbody3, deltaTime, angularAcceleration);

	rbody3.SetPosition(p);
	rbody3.SetVelocity(v);
}

void PhysicsEngine::Task2Display(const mat4& viewMatrix, const mat4& projMatrix)
{
	rbody2.Draw(viewMatrix, projMatrix);
	rbody3.Draw(viewMatrix, projMatrix);
}

// This is called every frame
void PhysicsEngine::Update(float deltaTime, float totalTime)
{
	if (!paused)
	{
		switch (activeDemo)
		{
		default:
		case Task1:
			Task1Update(deltaTime, totalTime);
			break;
		case Task2:
			Task2Update(deltaTime, totalTime);
			break;
		}
	}
}

// This is called every frame, after Update
void PhysicsEngine::Display(const mat4& viewMatrix, const mat4& projMatrix)
{
	ground.Draw(viewMatrix, projMatrix);

	switch (activeDemo)
	{
	default:
	case Task1:
		Task1Display(viewMatrix, projMatrix);
		break;
	case Task2:
		Task2Display(viewMatrix, projMatrix);
		break;
	}
}

void PhysicsEngine::HandleInputKey(int keyCode, bool pressed)
{
	switch (keyCode)
	{
	case GLFW_KEY_1:
		printf("Key 1 was %s\n", pressed ? "pressed" : "released");
		Task1Init();
		activeDemo = Task1;
		break;
	case GLFW_KEY_2:
		printf("Key 2 was %s\n", pressed ? "pressed" : "released");
		Task2Init();
		activeDemo = Task2;
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