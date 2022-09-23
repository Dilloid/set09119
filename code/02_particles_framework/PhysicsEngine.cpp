#include "PhysicsEngine.h"
#include "Application.h"
#include "Camera.h"

using namespace glm;

const glm::vec3 GRAVITY = glm::vec3(0, -9.81, 0);


void ExplicitEuler(vec3& pos, vec3& vel, float mass, const vec3& accel, const vec3& impulse, float dt)
{
	//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	// TODO: Implement
	//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
}

void SymplecticEuler(vec3& pos, vec3& vel, float mass, const vec3& accel, const vec3& impulse, float dt)
{
	//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	// TODO: Implement
	vel += dt * accel + (impulse/mass);
	pos += dt * vel;
	//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
}

vec3 CollisionImpulse(Particle& pobj, const glm::vec3& cubeCentre, float cubeHalfExtent, float coefficientOfRestitution = 0.9f)
{
	//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	// TODO: Calculate collision impulse
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
	//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
}

vec3 BlowDryerForce(const vec3& particlePosition, float cone_y_base, float cone_y_tip, float cone_r_base, float max_force = 100)
{
	//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	// TODO: Calculate blow dryer force
	vec3 force = {0,0,0};
	return force;
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

	// Initialise ground
	ground.SetMesh(groundMesh);
	ground.SetShader(defaultShader);
	ground.SetScale(vec3(10.0f));

	// Initialise particle
	particle.SetMesh(mesh);
	particle.SetShader(defaultShader);
	particle.SetColor(vec4(1, 0, 0, 1));
	particle.SetPosition(vec3(0, 5, 0));
	particle.SetScale(vec3(0.1f));
	particle.SetVelocity(vec3(1.f, 0.0f, 5.f));

	camera = Camera(vec3(0, 2.5, 10));

}

// This is called every frame
void PhysicsEngine::Update(float deltaTime, float totalTime)
{
	//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	// TODO: Handle collisions and calculate impulse
	//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	auto impulse = CollisionImpulse(particle, glm::vec3(0.0f,5.0f,0.0f), 5.0f, 1.0f);
	// Calculate acceleration by accumulating all forces (here we just have gravity) and dividing by the mass
	//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	// TODO: Implement a simple integration scheme
	vec3 p = particle.Position(), v = particle.Velocity();
	vec3 fGravity = particle.Mass() * GRAVITY;

	float vMag = sqrt(pow(v.x, 2) + pow(v.y, 2) + pow(v.z, 2));
	// REPLACE 
	vec3 fAero = 0.5f * 1.0f * glm::length(v) * 0.47f * (3.14f * (0.05f * 0.05f)) * (-v);
	vec3 acceleration = (fGravity + fAero) / particle.Mass();

	SymplecticEuler(p,v, particle.Mass(), acceleration, impulse, deltaTime);
	particle.SetPosition(p);
	particle.SetVelocity(v);

	

	//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
}

// This is called every frame, after Update
void PhysicsEngine::Display(const mat4& viewMatrix, const mat4& projMatrix)
{
	particle.Draw(viewMatrix, projMatrix);
	ground.Draw(viewMatrix, projMatrix);
}

void PhysicsEngine::HandleInputKey(int keyCode, bool pressed)
{
	switch (keyCode)
	{
	case GLFW_KEY_1:
		printf("Key 1 was %s\n", pressed ? "pressed" : "released");
		break; // don't forget this at the end of every "case" statement!
	default:
		break;
	}
}