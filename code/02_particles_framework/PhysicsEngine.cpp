#include "PhysicsEngine.h"
#include "Application.h"
#include "Camera.h"

using namespace glm;

const glm::vec3 GRAVITY = glm::vec3(0, -9.81, 0);

Demo activeDemo = Task1;

void ExplicitEuler(vec3& pos, vec3& vel, float mass, const vec3& accel, const vec3& impulse, float dt)
{
	//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	// TODO: Implement
	//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
}

void SymplecticEuler(vec3& pos, vec3& vel, float mass, const vec3& accel, const vec3& impulse, float dt)
{
	vel += dt * accel + (impulse/mass);
	pos += dt * vel;
}

vec3 CollisionImpulse(Particle& pobj, const glm::vec3& cubeCentre, float cubeHalfExtent, float coefficientOfRestitution = 0.9f)
{
	vec3 impulse{ 0.0f };
	vec3 floorN{ 0.0f };

	if (pobj.Position().y <= (cubeCentre.y - cubeHalfExtent)) {
		pobj.SetPosition(vec3{ pobj.Position().x, (cubeCentre.y - cubeHalfExtent), pobj.Position().z });
		floorN = { 0.0f, 1.0f, 0.0f };
	}

	if (pobj.Position().y >= (cubeCentre.y + cubeHalfExtent)) {
		pobj.SetPosition(vec3{ pobj.Position().x, (cubeCentre.y + cubeHalfExtent), pobj.Position().z });
		floorN = { 0.0f, -1.0f, 0.0f };
	}
	
	if (pobj.Position().x <= (cubeCentre.x - cubeHalfExtent)) {
		pobj.SetPosition(vec3{ (cubeCentre.x - cubeHalfExtent), pobj.Position().y, pobj.Position().z });
		floorN = { -1.0f, 0.0f, 0.0f };
	}
	
	if (pobj.Position().x >= (cubeCentre.x + cubeHalfExtent)) {
		pobj.SetPosition(vec3{ (cubeCentre.x + cubeHalfExtent), pobj.Position().y, pobj.Position().z });
		floorN = { 1.0f, 0.0f, 0.0f };
	}
	
	if (pobj.Position().z <= (cubeCentre.z - cubeHalfExtent)) {
		pobj.SetPosition(vec3{ pobj.Position().x, pobj.Position().y, (cubeCentre.z - cubeHalfExtent) });
		floorN = { 0.0f, 0.0f, -1.0f };
	}

	if (pobj.Position().z >= (cubeCentre.z + cubeHalfExtent)) {
		pobj.SetPosition(vec3{ pobj.Position().x, pobj.Position().y, (cubeCentre.z + cubeHalfExtent) });
		floorN = { 0.0f, 0.0f, 1.0f };
	}
	
	float vClose = dot(pobj.Velocity(), floorN);
	impulse = -(1 + coefficientOfRestitution) * pobj.Mass() * vClose * floorN;
	return impulse;
}

vec3 BlowDryerForce(const vec3& particlePosition, float cone_y_base, float cone_y_tip, float cone_r_base, float max_force = 100)
{
	//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	// TODO: Calculate blow dryer force
	vec3 force = {0,0,0};
	return force;
}

void PhysicsEngine::Task1Init()
{
	task1Particle.SetColor(vec4(1, 0, 0, 1));
	task1Particle.SetPosition(vec3(0, 5, 0));
	task1Particle.SetScale(vec3(0.1f));
	task1Particle.SetVelocity(vec3(1.f, 0.0f, 5.f));
}

void PhysicsEngine::Task1Update(float deltaTime, float totalTime)
{
	auto impulse = CollisionImpulse(task1Particle, glm::vec3(0.0f, 5.0f, 0.0f), 5.0f, 1.0f);

	vec3 p = task1Particle.Position(), v = task1Particle.Velocity();
	vec3 fGravity = task1Particle.Mass() * GRAVITY;

	float vMag = sqrt(pow(v.x, 2) + pow(v.y, 2) + pow(v.z, 2));

	vec3 fAero = 0.5f * 1.0f * glm::length(v) * 0.47f * (3.14f * (0.05f * 0.05f)) * (-v);
	vec3 acceleration = (fGravity + fAero) / task1Particle.Mass();

	SymplecticEuler(p, v, task1Particle.Mass(), acceleration, impulse, deltaTime);
	task1Particle.SetPosition(p);
	task1Particle.SetVelocity(v);
}

void PhysicsEngine::Task1Display(const mat4& viewMatrix, const mat4& projMatrix)
{
	task1Particle.Draw(viewMatrix, projMatrix);
}

void PhysicsEngine::Task2Init()
{
	task2Particles[0].SetColor(vec4(1, 0, 0, 1));
	task2Particles[0].SetPosition(vec3(-2, 5, 0));
	task2Particles[0].SetScale(vec3(0.1f));
	task2Particles[0].SetVelocity(vec3(1.f, 0.0f, 5.f));
	
	task2Particles[1].SetColor(vec4(0, 1, 0, 1));
	task2Particles[1].SetPosition(vec3(0, 5, 0));
	task2Particles[1].SetScale(vec3(0.1f));
	task2Particles[1].SetVelocity(vec3(1.f, 0.0f, 5.f));

	task2Particles[2].SetColor(vec4(0, 0, 1, 1));
	task2Particles[2].SetPosition(vec3(2, 5, 0));
	task2Particles[2].SetScale(vec3(0.1f));
	task2Particles[2].SetVelocity(vec3(1.f, 0.0f, 5.f));

	// TODO
}

void PhysicsEngine::Task2Update(float deltaTime, float totalTime)
{
	// TODO
}

void PhysicsEngine::Task2Display(const mat4& viewMatrix, const mat4& projMatrix)
{
	for (Particle p : task2Particles)
	{
		p.Draw(viewMatrix, projMatrix);
	}
}

void PhysicsEngine::Task3Init()
{
	// TODO
}

void PhysicsEngine::Task3Update(float deltaTime, float totalTime)
{
	// TODO
}

void PhysicsEngine::Task3Display(const mat4& viewMatrix, const mat4& projMatrix)
{
	// TODO
}

void PhysicsEngine::Task4Init()
{
	// TODO
}

void PhysicsEngine::Task4Update(float deltaTime, float totalTime)
{
	// TODO
}

void PhysicsEngine::Task4Display(const mat4& viewMatrix, const mat4& projMatrix)
{
	// TODO
}

// This is called once
void PhysicsEngine::Init(Camera& camera, MeshDb& meshDb, ShaderDb& shaderDb)
{
	// Get a few meshes/shaders from the databases
	auto defaultShader = shaderDb.Get("default");
	auto particleMesh = meshDb.Get("tetra");
	auto groundMesh = meshDb.Get("plane");

	meshDb.Add("cube", Mesh(MeshDataFromWavefrontObj("resources/models/cube.obj")));
	meshDb.Add("sphere", Mesh(MeshDataFromWavefrontObj("resources/models/sphere.obj")));
	meshDb.Add("cone", Mesh(MeshDataFromWavefrontObj("resources/models/cone.obj")));
	auto mesh = meshDb.Get("sphere");

	task1Particle.SetMesh(mesh);
	task1Particle.SetShader(defaultShader);

	task2Particles[0].SetMesh(mesh);
	task2Particles[0].SetShader(defaultShader);

	task2Particles[1].SetMesh(mesh);
	task2Particles[1].SetShader(defaultShader);

	task2Particles[2].SetMesh(mesh);
	task2Particles[2].SetShader(defaultShader);

	LoadDemo(Task2);

	// Initialise ground
	ground.SetMesh(groundMesh);
	ground.SetShader(defaultShader);
	ground.SetScale(vec3(10.0f));

	camera = Camera(vec3(0, 2.5, 10));
}

void PhysicsEngine::LoadDemo(Demo task)
{
	switch (task)
	{
	default:
	case Task1:
		Task1Init();
		break;
	case Task2:
		Task2Init();
		break;
	case Task3:
		Task3Init();
		break;
	case Task4:
		Task4Init();
		break;
	}

	activeDemo = task;
}

// This is called every frame
void PhysicsEngine::Update(float deltaTime, float totalTime)
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
	case Task3:
		Task3Update(deltaTime, totalTime);
		break;
	case Task4:
		Task4Update(deltaTime, totalTime);
		break;
	}
}

// This is called every frame, after Update
void PhysicsEngine::Display(const mat4& viewMatrix, const mat4& projMatrix)
{
	switch (activeDemo)
	{
	default:
	case Task1:
		Task1Display(viewMatrix, projMatrix);
		break;
	case Task2:
		Task2Display(viewMatrix, projMatrix);
		break;
	case Task3:
		Task3Display(viewMatrix, projMatrix);
		break;
	case Task4:
		Task4Display(viewMatrix, projMatrix);
		break;
	}

	ground.Draw(viewMatrix, projMatrix);
}

void PhysicsEngine::HandleInputKey(int keyCode, bool pressed)
{
	switch (keyCode)
	{
	case GLFW_KEY_1:
		printf("Key 1 was %s\n", pressed ? "pressed" : "released");
		LoadDemo(Task1);
		break;
	case GLFW_KEY_2:
		printf("Key 2 was %s\n", pressed ? "pressed" : "released");
		LoadDemo(Task2);
		break;
	case GLFW_KEY_3:
		printf("Key 3 was %s\n", pressed ? "pressed" : "released");
		LoadDemo(Task3);
		break;
	case GLFW_KEY_4:
		printf("Key 4 was %s\n", pressed ? "pressed" : "released");
		LoadDemo(Task4);
		break;
	default:
		break;
	}
}