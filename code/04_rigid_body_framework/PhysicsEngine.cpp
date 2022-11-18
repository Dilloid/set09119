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

Demo activeDemo = Task3;

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

			//std::cout << "Collision detected! Contact points merged: " << std::endl;
			for (vec3 p : contactPoints)
			{
				point += p;
				//std::cout << p << std::endl;
			}

			lowestPoint = point / contacts;
			//std::cout << "Lowest point changed to " << lowestPoint << std::endl;
		}
		else
		{
			//std::cout << "Collision detected! Lowest point: " << lowestPoint << std::endl;
		}

		contactPoints.clear();
		rb.Translate(vec3(0.0f, -lowestPoint.y, 0.0f));
		//std::cout << "Translated body by " << vec3(0.0f, -lowestPoint.y, 0.0f) << std::endl << std::endl;

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
		
		/*
		std::cout << "v1=" << v1 << ", w1=" << w1 << ", r1=" << r1 << std::endl;
		std::cout << "v2=" << v2 << ", w2=" << w2 << ", r2=" << r2 << std::endl;
		std::cout << "vr=" << vr << std::endl;
		std::cout << "m1i=" << m1i << std::endl;
		std::cout << "ii=" << ii << std::endl;
		std::cout << "jr=" << jr << std::endl << std::endl << std::endl;
		*/
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

	upright = rbody1.Orientation();
	Task3Init();

	// TODO: Get the mesh and shader for rigidy body
	camera = Camera(vec3(0, 5, 10));
}

void PhysicsEngine::StandardInit()
{
	rbody1.SetColor(vec4(1, 0, 0, 1));
	rbody1.SetPosition(vec3(0, 5, 0));
	rbody1.SetScale(vec3(1, 2, 1));
	rbody1.SetVelocity(vec3(1.0f, 0.0f, 5.0f));
}

void PhysicsEngine::StandardUpdate(float deltaTime, float totalTime)
{
	rbody1.ClearForcesImpulses();

	CollisionImpulse(rbody1, 0.8f);

	vec3 p = rbody1.Position();
	vec3 v = rbody1.Velocity();
	
	Force::Gravity(rbody1);
	Force::Drag(rbody1, vec3(0.0f));

	vec3 acceleration = rbody1.AccumulatedForce() / rbody1.Mass();

	SymplecticEuler(p, v, rbody1.Mass(), acceleration, rbody1.AccumulatedImpulse(), deltaTime);

	rbody1.SetPosition(p);
	rbody1.SetVelocity(v);
}

void PhysicsEngine::StandardDisplay(const mat4& viewMatrix, const mat4& projMatrix)
{
	rbody1.Draw(viewMatrix, projMatrix);
}

void PhysicsEngine::ExtendedInit()
{
	rbody2.SetColor(vec4(1, 0, 0, 1));
	rbody2.SetPosition(vec3(-2, 5, 0));
	rbody2.SetScale(vec3(1, 2, 1));
	rbody2.SetAngularVelocity(vec3(0, 10.0f, 0));

	rbody3.SetColor(vec4(1, 0, 0, 1));
	rbody3.SetPosition(vec3(2, 5, 0));
	rbody3.SetScale(vec3(1, 2, 1));
}

void PhysicsEngine::ExtendedUpdate(float deltaTime, float totalTime)
{
	rbody2.ClearForcesImpulses();
	rbody3.ClearForcesImpulses();

	// ##################################################

	CollisionImpulse(rbody2, 0.8f);

	vec3 p = rbody2.Position(), v = rbody3.Velocity();

	Force::Drag(rbody2, vec3(0.0f));

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

	Force::Drag(rbody3, vec3(0.0f));

	acceleration = rbody3.AccumulatedForce() / rbody3.Mass();

	SymplecticEuler(p, v, rbody3.Mass(), acceleration, rbody3.AccumulatedImpulse(), deltaTime);

	Integrate(rbody3, deltaTime, angularAcceleration);

	rbody3.SetPosition(p);
	rbody3.SetVelocity(v);
}

void PhysicsEngine::ExtendedDisplay(const mat4& viewMatrix, const mat4& projMatrix)
{
	rbody2.Draw(viewMatrix, projMatrix);
	rbody3.Draw(viewMatrix, projMatrix);
}

void PhysicsEngine::Task3Init()
{
	rbody1.SetScale(vec3(1, 3, 1));
	rbody1.SetColor(vec4(0.3f, 0.3f, 0.8f, 1.0f));
	rbody1.SetCoefficientOfRestitution(0.7f);
	rbody1.SetMass(2.0f);

	rbody1.SetOrientation(upright);
	rbody1.SetPosition(vec3(0.0f, 3.0f, 0.0f));
	rbody1.SetVelocity(vec3(2.0f, 0.0f, 0.0f));
	rbody1.SetAngularVelocity(vec3(0.0f));
}

void PhysicsEngine::Task3Update(float deltaTime, float totalTime)
{
	rbody1.ClearForcesImpulses();

	vec3 p = rbody1.Position();
	vec3 v = rbody1.Velocity();
	vec3 w = rbody1.AngularVelocity();

	Force::Gravity(rbody1);
	Force::Drag(rbody1, vec3(0.0f));

	vec3 acceleration = rbody1.AccumulatedForce() / rbody1.Mass();
	vec3 currentPos = p;
	
	v += (1.0f / rbody1.Mass()) * (acceleration * deltaTime);
	p += v * deltaTime;

	rbody1.Translate(p - currentPos);
	Integrate(rbody1, deltaTime, vec3(0.0f));

	vec3 floorN = vec3(0.0f, 1.0f, 0.0f);
	vec3 lowestPoint = vec3(0.0f);
	float jr = CollisionImpulseJr(rbody1, lowestPoint);

	v += (jr / rbody1.Mass()) * floorN;
	w += jr * rbody1.InverseInertia() * cross(normalize(lowestPoint - p), floorN);

	if (totalTime == 2.0f)
	{
		v += vec3(-v.x, 0.0f, 0.0f);
	}

	rbody1.SetVelocity(v);
	rbody1.SetAngularVelocity(w);
}

void PhysicsEngine::Task3Display(const mat4& viewMatrix, const mat4& projMatrix)
{
	rbody1.Draw(viewMatrix, projMatrix);
}

void PhysicsEngine::Task4Init()
{
	rbody1.SetScale(vec3(1, 3, 1));
	rbody1.SetColor(vec4(0.3f, 0.3f, 0.8f, 1.0f));
	rbody1.SetCoefficientOfRestitution(0.7f);
	rbody1.SetMass(2.0f);

	rbody1.SetOrientation(upright);
	rbody1.SetPosition(vec3(0.0f, 8.0f, 0.0f));
	rbody1.SetVelocity(vec3(0.0f, 0.0f, 0.0f));
	rbody1.SetAngularVelocity(vec3(0.0f));
}

void PhysicsEngine::Task4Update(float deltaTime, float totalTime)
{
	rbody1.ClearForcesImpulses();

	vec3 p = rbody1.Position();
	vec3 v = rbody1.Velocity();
	vec3 w = rbody1.AngularVelocity();

	Force::Gravity(rbody1);
	Force::Drag(rbody1, vec3(0.0f));

	vec3 acceleration = rbody1.AccumulatedForce() / rbody1.Mass();
	vec3 currentPos = p;
	
	v += (1.0f / rbody1.Mass()) * (acceleration * deltaTime);
	p += v * deltaTime;

	rbody1.Translate(p - currentPos);
	Integrate(rbody1, deltaTime, vec3(0.0f));

	vec3 floorN = vec3(0.0f, 1.0f, 0.0f);
	vec3 lowestPoint = vec3(0.0f);
	float jr = CollisionImpulseJr(rbody1, lowestPoint);

	v += (jr / rbody1.Mass()) * floorN;
	w += jr * rbody1.InverseInertia() * cross(normalize(lowestPoint - p), floorN);

	rbody1.SetVelocity(v);
	rbody1.SetAngularVelocity(w);
}

void PhysicsEngine::Task4Display(const mat4& viewMatrix, const mat4& projMatrix)
{
	rbody1.Draw(viewMatrix, projMatrix);
}

void PhysicsEngine::Task5Init()
{
	rbody1.SetScale(vec3(1, 3, 1));
	rbody1.SetColor(vec4(0.3f, 0.3f, 0.8f, 1.0f));
	rbody1.SetCoefficientOfRestitution(1.0f);
	rbody1.SetMass(2.0f);

	rbody1.SetOrientation(upright);
	rbody1.SetPosition(vec3(0.0f, 8.0f, 0.0f));
	rbody1.SetVelocity(vec3(0.0f, 0.0f, 0.0f));
	rbody1.SetAngularVelocity(vec3(0.1f));
}

void PhysicsEngine::Task5Update(float deltaTime, float totalTime)
{
	rbody1.ClearForcesImpulses();

	vec3 p = rbody1.Position();
	vec3 v = rbody1.Velocity();
	vec3 w = rbody1.AngularVelocity();

	Force::Gravity(rbody1);
	Force::Drag(rbody1, vec3(0.0f));

	vec3 acceleration = rbody1.AccumulatedForce() / rbody1.Mass();
	vec3 currentPos = p;
	
	v += (1.0f / rbody1.Mass()) * (acceleration * deltaTime);
	p += v * deltaTime;

	rbody1.Translate(p - currentPos);
	Integrate(rbody1, deltaTime, vec3(0.0f));

	vec3 floorN = vec3(0.0f, 1.0f, 0.0f);
	vec3 lowestPoint = vec3(0.0f);
	float jr = CollisionImpulseJr(rbody1, lowestPoint);

	v += (jr / rbody1.Mass()) * floorN;
	w += jr * rbody1.InverseInertia() * cross(normalize(lowestPoint - p), floorN);

	rbody1.SetVelocity(v);
	rbody1.SetAngularVelocity(w);
}

void PhysicsEngine::Task5Display(const mat4& viewMatrix, const mat4& projMatrix)
{
	rbody1.Draw(viewMatrix, projMatrix);
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
			StandardUpdate(deltaTime, totalTime);
			break;
		case Task2:
			ExtendedUpdate(deltaTime, totalTime);
			break;
		case Task3:
			Task3Update(deltaTime, totalTime);
			break;
		case Task4:
			Task4Update(deltaTime, totalTime);
			break;
		case Task5:
			Task5Update(deltaTime, totalTime);
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
		StandardDisplay(viewMatrix, projMatrix);
		break;
	case Task2:
		ExtendedDisplay(viewMatrix, projMatrix);
		break;
	case Task3:
		Task3Display(viewMatrix, projMatrix);
		break;
	case Task4:
		Task4Display(viewMatrix, projMatrix);
		break;
	case Task5:
		Task5Display(viewMatrix, projMatrix);
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
		activeDemo = Task1;
		break;
	case GLFW_KEY_2:
		printf("Key 2 was %s\n", pressed ? "pressed" : "released");
		ExtendedInit();
		activeDemo = Task2;
		break;
	case GLFW_KEY_3:
		printf("Key 3 was %s\n", pressed ? "pressed" : "released");
		Task3Init();
		activeDemo = Task3;
		break;
	case GLFW_KEY_4:
		printf("Key 4 was %s\n", pressed ? "pressed" : "released");
		Task4Init();
		activeDemo = Task4;
		break;
	case GLFW_KEY_5:
		printf("Key 5 was %s\n", pressed ? "pressed" : "released");
		Task5Init();
		activeDemo = Task5;
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