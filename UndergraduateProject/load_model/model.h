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
	std::string directory;					// 模型路径

	// 构造函数
	Model(const std::string& path);

	// 绘制Model
	void draw(Shader& shader);

private:
	// 加载模型
	void loadModel(const std::string& path);
	// 递归处理所有node，这里可以保留父子节点信息
	void processNode(aiNode* node, const aiScene* scene);
	// 处理单个mesh
	Mesh processMesh(aiMesh* mesh, const aiScene* scene);
	// 加载材质里面的所有给定类型的纹理
	std::vector<Texture> loadMaterialTexture(aiMaterial* material, aiTextureType type, TextureType textureType, const aiScene* scene);
};


// 一个加载模型的demo
/*

#include "winCam.h"
#include "render/shader.h"
#include "render/texture.h"
#include "load_model/model.h"
#include "load_model/tetMesh.h"
using namespace std;

// 用白色线条绘制网格模型
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
		// 交换缓冲
		glfwPollEvents();
		glfwSwapBuffers(winCam.window);

		// 必须调用winCam的每一帧更新函数
		winCam.update();

		// 处理输入
		winCam.processInput();

		// 清理window
		glClearColor(0.1f, 0.1f, 0.1f, 1.0f);
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);


		// 主要循环代码
		shader.use();
		shader.setMat4("projection", winCam.projection);
		shader.setMat4("view", winCam.view);
		shader.setMat4("model", winCam.model);
		// 设置反光度和相机位置
		shader.setFloat("material.shininess", 32.0f);
		shader.setVec3("viewPos", winCam.cameraPosition);
		// 设置定向光
		shader.setVec3("dirLight.direction", -0.2f, -1.0f, -0.3f);
		shader.setVec3("dirLight.ambient", 0.05f, 0.05f, 0.05f);
		shader.setVec3("dirLight.diffuse", 0.8f, 0.8f, 0.8f);
		shader.setVec3("dirLight.specular", 0.5f, 0.5f, 0.5f);

#ifdef IS_LINE_MESH
		shader.setInt("isLineMesh", 1);
#else
		shader.setInt("isLineMesh", 0);
#endif

		// 绘制
		model.draw(shader);
	}

	// 结束
	glfwTerminate();

	return 0;
}


*/
