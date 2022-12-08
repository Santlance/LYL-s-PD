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

// �����嶥��
typedef glm::vec3 TetVertex;


// mesh�ı���������
struct TetFace
{
	unsigned int vertexIds[3];	// ��¼��������

	TetFace(unsigned int id1, unsigned int id2, unsigned int id3);
};


// ������
struct TetElement
{
	unsigned int vertexIds[4];	// ��������

	TetElement(unsigned int id1, unsigned int id2, unsigned int id3, unsigned int id4);
};


// ����������
class TetMesh
{
public:
	std::vector<TetVertex> vertices;
	std::vector<TetFace> faces;
	std::vector<TetElement> elements;

	TetMesh() {}
	TetMesh(std::string path);

	// ��ȡ��Ŀ
	int getVertexCount();
	int getFaceCount();
	int getElementCount();

	// ����VAO��VBO��EBO�ķ�������Ϊ����λ�û�仯������Ӧ��Ҫ��̬����
	void setupFaceObject();
	void setupTetObject();
	std::vector<unsigned int> elementIndices;	// �洢�������ÿ�������ζ�������

	// ����
	void drawFace(Shader& shader);
	void drawTet(Shader& shader);

	// ��simulation��Eigen�����п���λ����Ϣ
	void copyPositionsFromEigenVector(const EigenVectorX& positions);

private:
	unsigned int faceVAO, faceVBO, faceEBO;		// ���Ʊ���������Ҫ��
	unsigned int tetVAO, tetVBO, tetEBO;		// ����������������Ҫ��

	/**
	* ��ȡ.msh�ļ������ļ�ͨ��Gmsh��������
	* ������ļ���ʽΪ��
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
	*���У�element_type��һЩȡֵ�Լ�����ĺ������£�
	*	1�� �߶�
	*	2��	������
	*	4�� ������
	*
	* ����ֻ���оٳ�����Ŀ�����õ������ݣ�������ļ���ʽ�ɲ��� http://gmsh.info/doc/texinfo/
	*/
	void load_msh_file(std::string path);
};
