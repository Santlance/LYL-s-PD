#pragma once

#include <string>
#include <vector>
#include "../render/shader.h"
#include "../render/texture.h"

constexpr unsigned int MAX_BONE_INFLUENCE = 4;

// 每个点的信息
struct Vertex
{
	glm::vec3 position;		// 位置
	glm::vec3 normal;		// 法向量
	glm::vec2 texCoords;	// 纹理坐标
	glm::vec3 tangent;		//
	glm::vec3 bitangent;	//
	unsigned int m_BoneIDs[MAX_BONE_INFLUENCE];
	float m_Weights[MAX_BONE_INFLUENCE];
};

class Mesh
{
public:
	// 网格数据
	std::vector<Vertex> vertices;
	std::vector<unsigned int> indices;
	std::vector<Texture> textures;

	// 函数
	Mesh(std::vector<Vertex> vertices, std::vector<unsigned int> indices, std::vector<Texture> textures);

	void draw(Shader& shader);

private:
	unsigned int VAO, VBO, EBO;

	void setupMesh();

	std::string textureType2NameInShader(TextureType type);
};
