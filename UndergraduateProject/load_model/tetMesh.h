#pragma once

#include <glad/glad.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <vector>
#include <string>
#include <iostream>
#include <fstream>
#include "../render/shader.h"
#include "../header.h"
#include "../projective_dynamics/pd_cuda.h"

// 四面体顶点
typedef glm::vec3 TetVertex;


// mesh的表面三角形
struct TetFace
{
	unsigned int vertexIds[3];	// 记录顶点索引

	TetFace(unsigned int id1, unsigned int id2, unsigned int id3);
};


// 四面体
struct TetElement
{
	unsigned int vertexIds[4];	// 顶点索引

	TetElement(unsigned int id1, unsigned int id2, unsigned int id3, unsigned int id4);
};


// 四面体网格
class TetMesh
{
public:
	std::vector<TetVertex> vertices;
	std::vector<TetFace> faces;
	std::vector<TetElement> elements;

	TetMesh() {}
	TetMesh(std::string path);

	// 获取数目
	int getVertexCount();
	int getFaceCount();
	int getElementCount();

	// 设置VAO、VBO及EBO的方法，因为顶点位置会变化，所以应该要动态设置
	void setupFaceObject();
	void setupTetObject();
	std::vector<unsigned int> elementIndices;	// 存储四面体的每个三角形顶点索引

	// 绘制
	void drawFace(Shader& shader);
	void drawTet(Shader& shader);

	// 从simulation的Eigen向量中拷贝位置信息
	void copyPositionsFromEigenVector(const EigenVectorX& positions);

private:
	unsigned int faceVAO, faceVBO, faceEBO;		// 绘制表面网格需要的
	unsigned int tetVAO, tetVBO, tetEBO;		// 绘制四面体网格需要的

	/**
	* 读取.msh文件，该文件通过Gmsh工具生成
	* 具体的文件格式为：
	*	$MeshFormat
	*	version-number file-type data-size
	*	$EndMeshFormat
	*	...
	*	$Nodes
	*	number_of_nodes
	*	node_id, x, y, z
	*	...
	*	$EndNodes
	*	$Elements
	*	number_of_elements
	*	element_id, element_type, number_of_tags, <tags>, node_id_list
	*	...
	*	$EndElements
	*	...
	*
	*
	*其中，element_type的一些取值以及代表的含义如下：
	*	1： 线段
	*	2：	三角形
	*	4： 四面体
	*
	* 上面只是列举出本项目可能用到的数据，更多的文件格式可查阅 http://gmsh.info/doc/texinfo/
	*/
	void load_msh_file(std::string path);
};
