#pragma once

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

#include "shader.h"
#include "../header.h"

// 立方体 几何中心为中心点

/*
* 定义了若干种立方体类型，包括：
*	- 单纯的立方体	PURE_CUBE		：VBO中每个单位包括3个float表示顶点坐标
*	- 纹理立方体	TEXTURE_CUBE	：VBO中每个单位包括5个float表示 顶点坐标3+纹理坐标2
*/

enum CubeType
{
	PURE_CUBE,
	TEXTURE_CUBE,
};

class Cube
{
public:
	// 构造函数
	Cube(CubeType type);

	void draw(Shader& shader);

private:
	CubeType type;
	d_uint VAO, VBO;
};
