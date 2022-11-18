#include "PhysicsObject.h"

#include <glm/glm.hpp>

#include "PhysicsEngine.h"
#include "Mesh.h"
#include "Shader.h"


void PhysicsBody::Draw(const glm::mat4& viewMatrix, const glm::mat4& projectionMatrix) const
{
	m_shader->Use();
	//m_shader->SetUniform("model", ModelMatrix());
	//m_shader->SetUniform("view", viewMatrix);
	//m_shader->SetUniform("projection", projectionMatrix);
	m_shader->SetUniform("color", m_color);

	auto mvp = projectionMatrix * viewMatrix * ModelMatrix();
	m_shader->SetUniform("modelViewProjectionMatrix", mvp);
	m_shader->SetUniform("normalMatrix", transpose(inverse(viewMatrix * ModelMatrix())));
	m_mesh->DrawVertexArray();
}

glm::mat3 RigidBody::InverseInertia()
{
	auto massDivided = (1.0f / 12.0f) * Mass();

	float height = 6.0f;
	float width = 2.0f;
	float depth = 2.0f;

	float h2 = height * height;
	float w2 = width * width;
	float d2 = depth * depth;

	glm::mat3 inertiaTensor = {
		massDivided * (h2 + d2), 0.0f, 0.0f,
		0.0f, massDivided * (w2 + d2), 0.0f,
		0.0f, 0.0f, massDivided * (w2 + h2)
	};

	glm::mat3 R = Orientation();
	glm::mat3 inverseInertia = R * inverse(inertiaTensor) * glm::transpose(R);

	return inverseInertia;
}
