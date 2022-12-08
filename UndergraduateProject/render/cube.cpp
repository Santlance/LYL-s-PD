#include "cube.h"

const d_float pure_cube_vertices[] = {
	// positions
	-0.5f, -0.5f, -0.5f,
	 0.5f, -0.5f, -0.5f,
	 0.5f,  0.5f, -0.5f,
	 0.5f,  0.5f, -0.5f,
	-0.5f,  0.5f, -0.5f,
	-0.5f, -0.5f, -0.5f,

	-0.5f, -0.5f,  0.5f,
	 0.5f, -0.5f,  0.5f,
	 0.5f,  0.5f,  0.5f,
	 0.5f,  0.5f,  0.5f,
	-0.5f,  0.5f,  0.5f,
	-0.5f, -0.5f,  0.5f,

	-0.5f,  0.5f,  0.5f,
	-0.5f,  0.5f, -0.5f,
	-0.5f, -0.5f, -0.5f,
	-0.5f, -0.5f, -0.5f,
	-0.5f, -0.5f,  0.5f,
	-0.5f,  0.5f,  0.5f,

	 0.5f,  0.5f,  0.5f,
	 0.5f,  0.5f, -0.5f,
	 0.5f, -0.5f, -0.5f,
	 0.5f, -0.5f, -0.5f,
	 0.5f, -0.5f,  0.5f,
	 0.5f,  0.5f,  0.5f,

	-0.5f, -0.5f, -0.5f,
	 0.5f, -0.5f, -0.5f,
	 0.5f, -0.5f,  0.5f,
	 0.5f, -0.5f,  0.5f,
	-0.5f, -0.5f,  0.5f,
	-0.5f, -0.5f, -0.5f,

	-0.5f,  0.5f, -0.5f,
	 0.5f,  0.5f, -0.5f,
	 0.5f,  0.5f,  0.5f,
	 0.5f,  0.5f,  0.5f,
	-0.5f,  0.5f,  0.5f,
	-0.5f,  0.5f, -0.5f,
};

const d_float texture_cube_vertices[] = {
	// positions          // texture Coords
	-0.5f, -0.5f, -0.5f,  0.0f, 0.0f,
	 0.5f, -0.5f, -0.5f,  1.0f, 0.0f,
	 0.5f,  0.5f, -0.5f,  1.0f, 1.0f,
	 0.5f,  0.5f, -0.5f,  1.0f, 1.0f,
	-0.5f,  0.5f, -0.5f,  0.0f, 1.0f,
	-0.5f, -0.5f, -0.5f,  0.0f, 0.0f,

	-0.5f, -0.5f,  0.5f,  0.0f, 0.0f,
	 0.5f, -0.5f,  0.5f,  1.0f, 0.0f,
	 0.5f,  0.5f,  0.5f,  1.0f, 1.0f,
	 0.5f,  0.5f,  0.5f,  1.0f, 1.0f,
	-0.5f,  0.5f,  0.5f,  0.0f, 1.0f,
	-0.5f, -0.5f,  0.5f,  0.0f, 0.0f,

	-0.5f,  0.5f,  0.5f,  1.0f, 0.0f,
	-0.5f,  0.5f, -0.5f,  1.0f, 1.0f,
	-0.5f, -0.5f, -0.5f,  0.0f, 1.0f,
	-0.5f, -0.5f, -0.5f,  0.0f, 1.0f,
	-0.5f, -0.5f,  0.5f,  0.0f, 0.0f,
	-0.5f,  0.5f,  0.5f,  1.0f, 0.0f,

	 0.5f,  0.5f,  0.5f,  1.0f, 0.0f,
	 0.5f,  0.5f, -0.5f,  1.0f, 1.0f,
	 0.5f, -0.5f, -0.5f,  0.0f, 1.0f,
	 0.5f, -0.5f, -0.5f,  0.0f, 1.0f,
	 0.5f, -0.5f,  0.5f,  0.0f, 0.0f,
	 0.5f,  0.5f,  0.5f,  1.0f, 0.0f,

	-0.5f, -0.5f, -0.5f,  0.0f, 1.0f,
	 0.5f, -0.5f, -0.5f,  1.0f, 1.0f,
	 0.5f, -0.5f,  0.5f,  1.0f, 0.0f,
	 0.5f, -0.5f,  0.5f,  1.0f, 0.0f,
	-0.5f, -0.5f,  0.5f,  0.0f, 0.0f,
	-0.5f, -0.5f, -0.5f,  0.0f, 1.0f,

	-0.5f,  0.5f, -0.5f,  0.0f, 1.0f,
	 0.5f,  0.5f, -0.5f,  1.0f, 1.0f,
	 0.5f,  0.5f,  0.5f,  1.0f, 0.0f,
	 0.5f,  0.5f,  0.5f,  1.0f, 0.0f,
	-0.5f,  0.5f,  0.5f,  0.0f, 0.0f,
	-0.5f,  0.5f, -0.5f,  0.0f, 1.0f
};

Cube::Cube(CubeType type)
{
	this->type = type;

	/*
		简言而之就是，通过一个VAO（vertex array）去整合VBO(vertex buffer)以及顶点属性（vertex attribute）。
		其中VBO表示顶点缓冲，用于存储顶点的位置等信息，而顶点属性用于描述如何解析VBO。
		通常一个VAO对应一个VBO，以及多个vertex attribute用于解释VBO
		而EBO保存索引序列，绑定中的VAO会保存EBO的绑定状态
	*/
	// 配置一个VAO的基本过程
	// --------------------------------------
	// 1. 建立VAO、VBO、EBO（非必要）对象
	glGenVertexArrays(1, &VAO);
	glGenBuffers(1, &VBO);
	//glGenBuffers(1, &EBO);
	// 2. 绑定VAO
	glBindVertexArray(VAO);
	// 3. 配置VBO，设定缓冲数据
	glBindBuffer(GL_ARRAY_BUFFER, VBO);
	if (type == CubeType::PURE_CUBE)
		glBufferData(GL_ARRAY_BUFFER, sizeof(pure_cube_vertices), pure_cube_vertices, GL_STATIC_DRAW);
	else if (type == CubeType::TEXTURE_CUBE)
		glBufferData(GL_ARRAY_BUFFER, sizeof(texture_cube_vertices), texture_cube_vertices, GL_STATIC_DRAW);
	// (非必要）配置EBO，设定缓冲数据
	//glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
	//glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(indices), indices, GL_STATIC_DRAW);
	// 4. 链接顶点属性
	// glVertexAttribPointer(GLuint index, GLint size, GLenum type, GLboolean normalized, GLsizei stride, const void *pointer)
	// index: 要配置的顶点属性的编号
	// size: 顶点属性的大小，vec3就是3
	// type: 数据类型
	// normalized: 是否标准化
	// stride: 步长
	// pointer: 偏移量
	if (type == CubeType::PURE_CUBE)
	{
		glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(d_float), (void*)0);
		glEnableVertexAttribArray(0);
	}
	else if (type == CubeType::TEXTURE_CUBE)
	{
		glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 5 * sizeof(d_float), (void*)0);
		glEnableVertexAttribArray(0);
		glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 5 * sizeof(d_float), (void*)(3 * sizeof(d_float)));
		glEnableVertexAttribArray(1);
	}
	// 5. 解除数组缓冲的绑定
	// glVertexAttribPointer将VBO注册为顶点属性绑定的顶点缓冲，因此可以解除绑定
	glBindBuffer(GL_ARRAY_BUFFER, 0);
	// 当VAO属于激活状态时，不能解除EBO的绑定，因为VAO会存储glBindBuffer的函数调用，而VAO需要直接绑定EBO
	// 6. 解除VAO的绑定、（非必要）解除EBO的绑定
	// 第5、6步不是必要的，因为绑定其它的VAO、VBO的时候就会自动解绑这个
	glBindVertexArray(0);
	//glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
	//--------------------------------------------
}

void Cube::draw(Shader& shader)
{
	shader.use();
	glBindVertexArray(VAO);
	glDrawArrays(GL_TRIANGLES, 0, 36);
	glBindVertexArray(0);
}
