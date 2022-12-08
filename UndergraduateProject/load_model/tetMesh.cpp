#include "tetMesh.h"


TetFace::TetFace(unsigned int id1, unsigned int id2, unsigned int id3)
{
	vertexIds[0] = id1;
	vertexIds[1] = id2;
	vertexIds[2] = id3;
}


TetElement::TetElement(unsigned int id1, unsigned int id2, unsigned int id3, unsigned int id4)
{
	vertexIds[0] = id1;
	vertexIds[1] = id2;
	vertexIds[2] = id3;
	vertexIds[3] = id4;
}


TetMesh::TetMesh(std::string path)
{
	// 初始化
	glGenVertexArrays(1, &faceVAO);
	glGenBuffers(1, &faceVBO);
	glGenBuffers(1, &faceEBO);
	glGenVertexArrays(1, &tetVAO);
	glGenBuffers(1, &tetVBO);
	glGenBuffers(1, &tetEBO);
	// 加载网格模型
	load_msh_file(path);

	// 处理elementIndices
	for (TetElement& e : elements)
	{
		elementIndices.push_back(e.vertexIds[0]);
		elementIndices.push_back(e.vertexIds[1]);
		elementIndices.push_back(e.vertexIds[2]);

		elementIndices.push_back(e.vertexIds[0]);
		elementIndices.push_back(e.vertexIds[2]);
		elementIndices.push_back(e.vertexIds[3]);

		elementIndices.push_back(e.vertexIds[0]);
		elementIndices.push_back(e.vertexIds[1]);
		elementIndices.push_back(e.vertexIds[3]);

		elementIndices.push_back(e.vertexIds[1]);
		elementIndices.push_back(e.vertexIds[2]);
		elementIndices.push_back(e.vertexIds[3]);
	}

	std::cout << "number of vertex: " << getVertexCount() << std::endl;
	std::cout << "number of face: " << getFaceCount() << std::endl;
	std::cout << "number of element: " << getElementCount() << std::endl;
}

int TetMesh::getVertexCount()
{
	return vertices.size();
}

int TetMesh::getFaceCount()
{
	return faces.size();
}

int TetMesh::getElementCount()
{
	return elements.size();
}

void TetMesh::setupFaceObject()
{
	glBindVertexArray(faceVAO);

	glBindBuffer(GL_ARRAY_BUFFER, faceVBO);
	glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(TetVertex), &vertices[0], GL_STATIC_DRAW);

	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, faceEBO);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, faces.size() * sizeof(TetFace), &faces[0], GL_STATIC_DRAW);

	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(TetVertex), (void*)0);
	glEnableVertexAttribArray(0);

	glBindVertexArray(0);
}

void TetMesh::setupTetObject()
{
	glBindVertexArray(tetVAO);

	glBindBuffer(GL_ARRAY_BUFFER, tetVBO);
	glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(TetVertex), &vertices[0], GL_STATIC_DRAW);

	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, tetEBO);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, elementIndices.size() * sizeof(unsigned int), &elementIndices[0], GL_STATIC_DRAW);

	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(TetVertex), (void*)0);
	glEnableVertexAttribArray(0);

	glBindVertexArray(0);
}

void TetMesh::drawFace(Shader& shader)
{
	glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
	shader.use();

	glBindVertexArray(faceVAO);
	glDrawElements(GL_TRIANGLES, faces.size() * 3, GL_UNSIGNED_INT, 0);
	glBindVertexArray(0);
}

void TetMesh::drawTet(Shader& shader)
{
	glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
	shader.use();

	glBindVertexArray(tetVAO);
	glDrawElements(GL_TRIANGLES, elementIndices.size() * 12, GL_UNSIGNED_INT, 0);
	glBindVertexArray(0);
}

void TetMesh::copyPositionsFromEigenVector(const EigenVectorX& positions)
{
	for (unsigned int i = 0; i < vertices.size(); i++)
	{
		vertices[i].x = positions[i * 3 + 0];
		vertices[i].y = positions[i * 3 + 1];
		vertices[i].z = positions[i * 3 + 2];
	}
}


void TetMesh::load_msh_file(std::string path)
{
	// 打开文件
	std::ifstream infile;
	infile.open(path.c_str(), std::ios::in);
	if (!infile.is_open())
	{
		std::cout << "cannot open msh file " << path << std::endl;
		return;
	}

	// 循环读取
	while (!infile.eof())
	{
		std::string str;

		// 读取Node（vertex）数据
		while (std::getline(infile, str) && str != "$Nodes");
		int vertexNum, id;
		float x, y, z;
		infile >> vertexNum;
		for (unsigned int i = 0; i < vertexNum; i++)
		{
			infile >> id >> x >> y >> z;
			vertices.push_back(TetVertex(x, y, z));
		}

		// 读取elements
		while (std::getline(infile, str) && str != "$Elements");
		int elementNum, elementType, tagNum, tag;
		unsigned int id1, id2, id3, id4;
		infile >> elementNum;
		for (unsigned int i = 0; i < elementNum; i++)
		{
			infile >> id >> elementType >> tagNum;
			while (tagNum--) infile >> tag;		// 不处理tag，不知道是啥东西
			if (elementType == 2)
			{
				// 三角形
				infile >> id1 >> id2 >> id3;
				// msh文件里面node编号从1开始，所以这里要-1
				faces.push_back(TetFace(id1 - 1, id2 - 1, id3 - 1));
			}
			else if (elementType == 4)
			{
				// 四面体
				infile >> id1 >> id2 >> id3 >> id4;
				// msh文件里面node编号从1开始，所以这里要-1
				elements.push_back(TetElement(id1 - 1, id2 - 1, id3 - 1, id4 - 1));
			}
			else if (elementType == 1)
			{
				infile >> id1 >> id2;
			}
			else if (elementType == 15)
			{
				infile >> id1;
			}
		}

		// 读完剩下的
		while (std::getline(infile, str));
	}
}
