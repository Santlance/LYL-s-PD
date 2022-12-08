#pragma once

#include <glad/glad.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <stb_image.h>

#include <assimp/Importer.hpp>
#include <assimp/scene.h>
#include <assimp/postprocess.h>

#include "mesh.h"

#include <string>
#include <fstream>
#include <sstream>
#include <iostream>
#include <map>
#include <vector>


class Model
{
public:
	std::vector<Texture> texturesLoaded;	//
	std::vector<Mesh> meshes;
	std::string directory;					// ģ��·��

	// ���캯��
	Model(const std::string& path);

	// ����Model
	void draw(Shader& shader);

private:
	// ����ģ��
	void loadModel(const std::string& path);
	// �ݹ鴦������node��������Ա������ӽڵ���Ϣ
	void processNode(aiNode* node, const aiScene* scene);
	// ������mesh
	Mesh processMesh(aiMesh* mesh, const aiScene* scene);
	// ���ز�����������и������͵�����
	std::vector<Texture> loadMaterialTexture(aiMaterial* material, aiTextureType type, TextureType textureType, const aiScene* scene);
};


// һ������ģ�͵�demo
/*

#include "winCam.h"
#include "render/shader.h"
#include "render/texture.h"
#include "load_model/model.h"
#include "load_model/tetMesh.h"
using namespace std;

// �ð�ɫ������������ģ��
#define IS_LINE_MESH

int main()
{
	WinCam winCam(1600, 1200, "Undergraduate Project by Yinglong Li");

	Model model("../Resources/Models/liver/gall.obj");
	Shader shader("../Resources/Shaders/model_phong.vs", "../Resources/Shaders/model_phong.fs");

#ifdef IS_LINE_MESH
	glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
#endif

	while (!glfwWindowShouldClose(winCam.window))
	{
		// ��������
		glfwPollEvents();
		glfwSwapBuffers(winCam.window);

		// �������winCam��ÿһ֡���º���
		winCam.update();

		// ��������
		winCam.processInput();

		// ����window
		glClearColor(0.1f, 0.1f, 0.1f, 1.0f);
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);


		// ��Ҫѭ������
		shader.use();
		shader.setMat4("projection", winCam.projection);
		shader.setMat4("view", winCam.view);
		shader.setMat4("model", winCam.model);
		// ���÷���Ⱥ����λ��
		shader.setFloat("material.shininess", 32.0f);
		shader.setVec3("viewPos", winCam.cameraPosition);
		// ���ö����
		shader.setVec3("dirLight.direction", -0.2f, -1.0f, -0.3f);
		shader.setVec3("dirLight.ambient", 0.05f, 0.05f, 0.05f);
		shader.setVec3("dirLight.diffuse", 0.8f, 0.8f, 0.8f);
		shader.setVec3("dirLight.specular", 0.5f, 0.5f, 0.5f);

#ifdef IS_LINE_MESH
		shader.setInt("isLineMesh", 1);
#else
		shader.setInt("isLineMesh", 0);
#endif

		// ����
		model.draw(shader);
	}

	// ����
	glfwTerminate();

	return 0;
}


*/
