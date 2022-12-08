#pragma once

#include <string>
#include <vector>
#include "../render/shader.h"
#include "../render/texture.h"

constexpr unsigned int MAX_BONE_INFLUENCE = 4;

// ÿ�������Ϣ
struct Vertex
{
	glm::vec3 position;		// λ��
	glm::vec3 normal;		// ������
	glm::vec2 texCoords;	// ��������
	glm::vec3 tangent;		//
	glm::vec3 bitangent;	//
	unsigned int m_BoneIDs[MAX_BONE_INFLUENCE];
	float m_Weights[MAX_BONE_INFLUENCE];
};

class Mesh
{
public:
	// ��������
	std::vector<Vertex> vertices;
	std::vector<unsigned int> indices;
	std::vector<Texture> textures;

	// ����
	Mesh(std::vector<Vertex> vertices, std::vector<unsigned int> indices, std::vector<Texture> textures);

	void draw(Shader& shader);

private:
	unsigned int VAO, VBO, EBO;

	void setupMesh();

	std::string textureType2NameInShader(TextureType type);
};
