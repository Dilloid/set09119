#include "PhysicsEngine.h"
#include "Application.h"
#include "Camera.h"
#include "Force.h"

using namespace glm;

const glm::vec3 GRAVITY = glm::vec3(0, -9.81, 0);

void SymplecticEuler(vec3& pos, vec3& vel, float mass, const vec3& accel, const vec3& impulse, float dt)
{
	vel += dt * accel + (impulse / mass);
	pos += dt * vel;
}

void ExplicitEuler(vec3& pos, vec3& vel, float mass, const vec3& accel, const vec3& impulse, float dt)
{
	vec3 v = vel + (impulse / mass);
	vel += dt * accel + (impulse / mass);
	pos += dt * v;
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

// This is called once
void PhysicsEngine::Init(Camera& camera, MeshDb& meshDb, ShaderDb& shaderDb)
{
	// Get a few meshes/shaders from the databases
	auto defaultShader = shaderDb.Get("default");
	auto groundMesh = meshDb.Get("plane");

	meshDb.Add("cube", Mesh(MeshDataFromWavefrontObj("resources/models/cube.obj")));
	meshDb.Add("sphere", Mesh(MeshDataFromWavefrontObj("resources/models/sphere.obj")));
	meshDb.Add("cone", Mesh(MeshDataFromWavefrontObj("resources/models/cone.obj")));
	
	auto particleMesh = meshDb.Get("sphere");
	t1Particle.SetMesh(particleMesh);
	t1Particle.SetShader(defaultShader);

	for (Particle &particle : t2Particles)
	{
		particle.SetMesh(particleMesh);
		particle.SetShader(defaultShader);
	}

	for (Particle& particle : t3Particles)
	{
		particle.SetMesh(particleMesh);
		particle.SetShader(defaultShader);
	}

	Task3Init();

	// Initialise ground
	ground.SetMesh(groundMesh);
	ground.SetShader(defaultShader);
	ground.SetScale(vec3(10.0f));

	camera = Camera(vec3(0, 5, 10));
}

void PhysicsEngine::Task1Init()
{
	t1Particle.SetColor(vec4(1, 0, 0, 1));
	t1Particle.SetPosition(vec3(0, 5, 0));
	t1Particle.SetScale(vec3(0.1f));
	t1Particle.SetVelocity(vec3(1.0f, 0.0f, 5.0f));
}

void PhysicsEngine::Task1Update(float deltaTime, float totalTime)
{
	t1Particle.ClearForcesImpulses();

	auto impulse = CollisionImpulse(t1Particle, glm::vec3(0.0f, 5.0f, 0.0f), 5.0f, 1.0f);

	vec3 p = t1Particle.Position();
	vec3 v = t1Particle.Velocity();

	Force::Gravity(t1Particle);
	Force::Drag(t1Particle);

	vec3 acceleration = t1Particle.AccumulatedForce() / t1Particle.Mass();

	SymplecticEuler(p, v, t1Particle.Mass(), acceleration, impulse, deltaTime);

	t1Particle.SetPosition(p);
	t1Particle.SetVelocity(v);
}

void PhysicsEngine::Task1Display(const mat4& viewMatrix, const mat4& projMatrix)
{
	t1Particle.Draw(viewMatrix, projMatrix);
}

void PhysicsEngine::Task2Init()
{
	for (Particle &particle : t2Particles)
	{
		particle.SetColor(vec4(1, 0, 0, 1));
		particle.SetScale(vec3(0.1f));
	}

	t2Particles[0].SetPosition(vec3(0, 5, 0));
	t2Particles[0].SetVelocity(vec3(0, 0, 0));

	t2Particles[1].SetPosition(vec3(1, 3, 0));
	t2Particles[1].SetVelocity(vec3(0, 0, 0));
}

void PhysicsEngine::Task2Update(float deltaTime, float totalTime)
{
	t2Particles[0].ClearForcesImpulses();
	t2Particles[1].ClearForcesImpulses();

	auto impulse = vec3{ 0 };//CollisionImpulse(t2Particles[1], glm::vec3(0.0f, 5.0f, 0.0f), 5.0f, 1.0f);

	vec3 p = t2Particles[1].Position();
	vec3 v = t2Particles[1].Velocity();

	Force::Gravity(t2Particles[1]);
	Force::Drag(t2Particles[1]);
	Force::Hooke(t2Particles[0], t2Particles[1], 1.0f, 8.0f, 0.01f);

	vec3 acceleration = t2Particles[1].AccumulatedForce() / t2Particles[1].Mass();

	SymplecticEuler(p, v, t2Particles[1].Mass(), acceleration, impulse, deltaTime);

	t2Particles[1].SetPosition(p);
	t2Particles[1].SetVelocity(v);
}

void PhysicsEngine::Task2Display(const mat4& viewMatrix, const mat4& projMatrix)
{
	for (Particle &particle : t2Particles)
	{
		particle.Draw(viewMatrix, projMatrix);
	}
}

void PhysicsEngine::Task3Init()
{
	int x = -5;
	int y = 8;

	for (Particle &particle : t3Particles)
	{
		particle.SetColor(vec4(1, 0, 0, 1));
		particle.SetScale(vec3(0.1f));

		particle.SetPosition(vec3(x, 4, 0));
		particle.SetVelocity(vec3(0, 0, 0));

		x++;
		y--;
	}
}

void PhysicsEngine::Task3Update(float deltaTime, float totalTime)
{
	for (Particle &particle : t3Particles)
		particle.ClearForcesImpulses();

	for (int i = 0; i < 10; i++)
	{
		auto impulse =  CollisionImpulse(t3Particles[i], glm::vec3(0.0f, 5.0f, 0.0f), 5.0f, 0.9f);

		vec3 p = t3Particles[i].Position();
		vec3 v = t3Particles[i].Velocity();

		Force::Gravity(t3Particles[i]);
		Force::Drag(t3Particles[i]);

		if (i < 10)
			Force::Hooke(t3Particles[i], t3Particles[i+1], 0.1f, 15.0f, 0.0f);

		t3Particles[0].ClearForcesImpulses();
		t3Particles[9].ClearForcesImpulses();

		vec3 acceleration = t3Particles[i].AccumulatedForce() / t3Particles[i].Mass();

		SymplecticEuler(p, v, t3Particles[i].Mass(), acceleration, impulse, deltaTime);

		t3Particles[i].SetPosition(p);
		t3Particles[i].SetVelocity(v);
	}

}

void PhysicsEngine::Task3Display(const mat4& viewMatrix, const mat4& projMatrix)
{
	for (Particle &particle : t3Particles)
	{
		particle.Draw(viewMatrix, projMatrix);
	}
}

// This is called every frame
void PhysicsEngine::Update(float deltaTime, float totalTime)
{
	Task3Update(deltaTime, totalTime);
}

// This is called every frame, after Update
void PhysicsEngine::Display(const mat4& viewMatrix, const mat4& projMatrix)
{
	ground.Draw(viewMatrix, projMatrix);
	Task3Display(viewMatrix, projMatrix);
}

void PhysicsEngine::HandleInputKey(int keyCode, bool pressed)
{
	switch (keyCode)
	{
	default:
		break;
	}
}