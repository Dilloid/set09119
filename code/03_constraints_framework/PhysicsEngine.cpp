#include "PhysicsEngine.h"
#include "Application.h"
#include "Camera.h"
#include "Force.h"

using namespace glm;

const glm::vec3 GRAVITY = glm::vec3(0, -9.81, 0);

Demo activeDemo = Task1;

void SymplecticEuler(vec3& pos, vec3& vel, float mass, const vec3& accel, const vec3& impulse, float dt)
{
	vel += (1.0f / mass) * ((accel * dt) + impulse);
	pos += vel * dt;
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
	vec3 floorN{ 0.0f, 1.0f, 0.0f };

	vec3 pos = pobj.Position();
	float e = coefficientOfRestitution;

	if (pos.y < 0.1f)
	{
		pobj.SetPosition({ pos.x, 0.1f, pos.z });
		float vClose = glm::dot(pobj.Velocity(), floorN);
		impulse = -(1 + e) * pobj.Mass() * vClose * floorN;
	}

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

	for (Particle &particle : t123Particles)
	{
		particle.SetMesh(particleMesh);
		particle.SetShader(defaultShader);
	}

	for (int y = 0; y < TASK45HEIGHT; y++)
	{
		for (int x = 0; x < TASK45LENGTH; x++)
		{
			t45Particles[y][x].SetMesh(particleMesh);
			t45Particles[y][x].SetShader(defaultShader);
		}
	}

	Task123Init();

	// Initialise ground
	ground.SetMesh(groundMesh);
	ground.SetShader(defaultShader);
	ground.SetScale(vec3(10.0f));

	camera = Camera(vec3(0, 5, 15));
}

void PhysicsEngine::Task123Init()
{
	int x = -5;
	int y = 8;

	for (Particle& particle : t123Particles)
	{
		particle.SetColor(vec4(1, 0, 0, 1));
		particle.SetScale(vec3(0.1f));

		particle.SetPosition(vec3(x, 4, 0));
		particle.SetVelocity(vec3(0, 0, 0));

		x++;
		y--;
	}
}

void PhysicsEngine::Task123Display(const mat4& viewMatrix, const mat4& projMatrix)
{
	for (Particle& particle : t123Particles)
	{
		particle.Draw(viewMatrix, projMatrix);
	}
}

void PhysicsEngine::Task123Update(float deltaTime, float totalTime)
{
	for (Particle& particle : t123Particles)
		particle.ClearForcesImpulses();

	for (int i = 0; i < TASK123LENGTH; i++)
	{
		float rl = RESTLENGTH;
		float ks = TENSION;
		float kd = 0.0f;
		vec3 impulse = vec3{ 0 };

		switch (activeDemo)
		{
		default:
		case Task1:
			break;
		case Task2:
			kd = 0.5f;
			break;
		case Task3:
			impulse = CollisionImpulse(t123Particles[i], glm::vec3(0.0f, 5.0f, 0.0f), 5.0f, 0.8f);
			kd = 0.5f;
			break;
		}

		t123Particles[i].ApplyImpulse(impulse);
		vec3 p = t123Particles[i].Position();
		vec3 v = t123Particles[i].Velocity();

		Force::Gravity(t123Particles[i]);
		Force::Drag(t123Particles[i], vec3{ 0 });

		if (i < TASK123LENGTH-1)
			Force::Hooke(t123Particles[i], t123Particles[i + 1], rl, ks, kd);

		t123Particles[0].ClearForcesImpulses();
		t123Particles[TASK123LENGTH-1].ClearForcesImpulses();

		vec3 acceleration = t123Particles[i].AccumulatedForce() / t123Particles[i].Mass();

		SymplecticEuler(p, v, t123Particles[i].Mass(), acceleration, t123Particles[i].AccumulatedImpulse(), deltaTime);

		t123Particles[i].SetPosition(p);
		t123Particles[i].SetVelocity(v);
	}
}

void PhysicsEngine::Task4Init()
{
	int posX = -5;
	int posY = 4;
	int posZ = -5;

	for (int y = 0; y < TASK45HEIGHT; y++)
	{
		for (int x = 0; x < TASK45LENGTH; x++)
		{
			t45Particles[y][x].SetColor(vec4(1, 0, 0, 1));
			t45Particles[y][x].SetScale(vec3(0.1f));

			t45Particles[y][x].SetPosition(vec3(posX, posY, posZ));
			t45Particles[y][x].SetVelocity(vec3(0, 0, 0));

			posX++;
		}
		
		posX = -5;
		posZ++;
	}
}

void PhysicsEngine::Task4Update(float deltaTime, float totalTime)
{
	for (int y = 0; y < TASK45HEIGHT; y++)
		for (int x = 0; x < TASK45LENGTH; x++)
			t45Particles[y][x].ClearForcesImpulses();

	for (int y = 0; y < TASK45HEIGHT; y++)
	{
		for (int x = 0; x < TASK45LENGTH; x++)
		{
			float rl = RESTLENGTH;
			float ks = 30.0f;
			float kd = 0.5f;
			vec3 impulse = CollisionImpulse(t45Particles[y][x], glm::vec3(0.0f, 5.0f, 0.0f), 5.0f, 0.8f);

			t45Particles[y][x].ApplyImpulse(impulse);
			vec3 p = t45Particles[y][x].Position();
			vec3 v = t45Particles[y][x].Velocity();

			Force::Gravity(t45Particles[y][x]);
			Force::Drag(t45Particles[y][x], vec3{ 0 });

			if (x < TASK45LENGTH - 1)
				Force::Hooke(t45Particles[y][x], t45Particles[y][x + 1], rl, ks, kd);
			
			if (y < TASK45HEIGHT - 1)
				Force::Hooke(t45Particles[y][x], t45Particles[y + 1][x], rl, ks, kd);

			if (x > 0 && y < TASK45HEIGHT - 1)
				Force::Hooke(t45Particles[y][x], t45Particles[y + 1][x - 1], rl * sqrt(2), ks, kd);
			
			if (x < TASK45LENGTH - 1 && y < TASK45HEIGHT - 1)
				Force::Hooke(t45Particles[y][x], t45Particles[y + 1][x + 1], rl * sqrt(2), ks, kd);

			t45Particles[0][0].ClearForcesImpulses();
			t45Particles[0][TASK45LENGTH-1].ClearForcesImpulses();
			t45Particles[TASK45HEIGHT-1][0].ClearForcesImpulses();
			t45Particles[TASK45HEIGHT-1][TASK45LENGTH-1].ClearForcesImpulses();
			
			vec3 acceleration = t45Particles[y][x].AccumulatedForce() / t45Particles[y][x].Mass();

			SymplecticEuler(p, v, t45Particles[y][x].Mass(), acceleration, t45Particles[y][x].AccumulatedImpulse(), deltaTime);

			t45Particles[y][x].SetPosition(p);
			t45Particles[y][x].SetVelocity(v);
		}
	}
}

void PhysicsEngine::Task5Init()
{
	int posX = -5;
	int posY = 10;
	int posZ = -5;

	for (int y = 0; y < TASK45HEIGHT; y++)
	{
		for (int x = 0; x < TASK45LENGTH; x++)
		{
			t45Particles[y][x].SetColor(vec4(1, 0, 0, 1));
			t45Particles[y][x].SetScale(vec3(0.1f));

			t45Particles[y][x].SetPosition(vec3(posX, posY, posZ));
			t45Particles[y][x].SetVelocity(vec3(0, 0, 0));

			posX++;
			posZ++;
		}

		posY--;
		posX = -5;
		posZ = -5;
	}
}

void PhysicsEngine::Task5Update(float deltaTime, float totalTime)
{
	float r = (float)rand() / RAND_MAX;

	for (int y = 0; y < TASK45HEIGHT; y++)
		for (int x = 0; x < TASK45LENGTH; x++)
			t45Particles[y][x].ClearForcesImpulses();

	for (int y = 0; y < TASK45HEIGHT; y++)
	{
		for (int x = 0; x < TASK45LENGTH; x++)
		{
			float rl = RESTLENGTH;
			float ks = 100.0f;
			float kd = 0.5f;
			vec3 impulse = CollisionImpulse(t45Particles[y][x], glm::vec3(0.0f, 5.0f, 0.0f), 5.0f, 0.9f);

			t45Particles[y][x].ApplyImpulse(impulse);
			vec3 p = t45Particles[y][x].Position();
			vec3 v = t45Particles[y][x].Velocity();

			Force::Gravity(t45Particles[y][x]);
			Force::Drag(t45Particles[y][x], vec3(5.0f, 0.0f, -5.0f) * r);

			if (x < TASK45LENGTH - 1)
				Force::Hooke(t45Particles[y][x], t45Particles[y][x + 1], rl, ks, kd);

			if (y < TASK45HEIGHT - 1)
				Force::Hooke(t45Particles[y][x], t45Particles[y + 1][x], rl, ks, kd);

			if (x > 0 && y < TASK45HEIGHT - 1)
				Force::Hooke(t45Particles[y][x], t45Particles[y + 1][x - 1], rl * 1.4f, ks, kd);

			if (x < TASK45LENGTH - 1 && y < TASK45HEIGHT - 1)
			{
				Force::Hooke(t45Particles[y][x], t45Particles[y + 1][x + 1], rl * 1.4f, ks, kd);

				/*
				// Attempted to apply aerodynamic drag to triangles here, but you can disable the
				// standard version above and uncomment these lines to see how that turned out.
				Force::Drag(t45Particles[y + 1][x], t45Particles[y + 1][x + 1], t45Particles[y][x], vec3{ 1.0f, 0.0f, -5.0f} * r);
				Force::Drag(t45Particles[y + 1][x + 1], t45Particles[y][x + 1], t45Particles[y][x], vec3{ 1.0f, 0.0f, -5.0f} * r);
				*/
			}

			/*
			// An attempt at bending constraints. Did not work.
			Force::Hooke(t45Particles[0][0], t45Particles[0][TASK45LENGTH - 1], rl * TASK45LENGTH, ks, kd);
			Force::Hooke(t45Particles[TASK45HEIGHT - 1][0], t45Particles[TASK45HEIGHT - 1][TASK45LENGTH - 1], rl * TASK45LENGTH, ks, kd);
			Force::Hooke(t45Particles[0][0], t45Particles[TASK45HEIGHT - 1][0], rl * TASK45HEIGHT, ks, kd);
			Force::Hooke(t45Particles[0][TASK45LENGTH - 1], t45Particles[TASK45HEIGHT - 1][TASK45LENGTH - 1], rl * TASK45HEIGHT, ks, kd);
			*/

			t45Particles[0][0].ClearForcesImpulses();
			t45Particles[0][TASK45LENGTH - 1].ClearForcesImpulses();

			vec3 acceleration = t45Particles[y][x].AccumulatedForce() / t45Particles[y][x].Mass();

			SymplecticEuler(p, v, t45Particles[y][x].Mass(), acceleration, t45Particles[y][x].AccumulatedImpulse(), deltaTime);

			t45Particles[y][x].SetPosition(p);
			t45Particles[y][x].SetVelocity(v);
		}
	}
}

void PhysicsEngine::Task45Display(const glm::mat4& viewMatrix, const glm::mat4& projMatrix)
{
	for (int y = 0; y < TASK45HEIGHT; y++)
	{
		for (int x = 0; x < TASK45LENGTH; x++)
		{
			t45Particles[y][x].Draw(viewMatrix, projMatrix);
		}
	}
}

// This is called every frame
void PhysicsEngine::Update(float deltaTime, float totalTime)
{
	switch (activeDemo)
	{
	default:
	case Task1:
	case Task2:
	case Task3:
		Task123Update(deltaTime, totalTime);
		break;
	case Task4:
		Task4Update(deltaTime, totalTime);
		break;
	case Task5:
		Task5Update(deltaTime, totalTime);
		break;
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
	case Task2:
	case Task3:
		Task123Display(viewMatrix, projMatrix);
		break;
	case Task4:
	case Task5:
		Task45Display(viewMatrix, projMatrix);
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
		Task123Init();
		activeDemo = Task1;
		break;
	case GLFW_KEY_2:
		printf("Key 2 was %s\n", pressed ? "pressed" : "released");
		Task123Init();
		activeDemo = Task2;
		break;
	case GLFW_KEY_3:
		printf("Key 3 was %s\n", pressed ? "pressed" : "released");
		Task123Init();
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
	default:
		break;
	}
}