#pragma once

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

#include "shader.h"
#include "../header.h"

// ������ ��������Ϊ���ĵ�

/*
* ���������������������ͣ�������
*	- ������������	PURE_CUBE		��VBO��ÿ����λ����3��float��ʾ��������
*	- ����������	TEXTURE_CUBE	��VBO��ÿ����λ����5��float��ʾ ��������3+��������2
*/

enum CubeType
{
	PURE_CUBE,
	TEXTURE_CUBE,
};

class Cube
{
public:
	// ���캯��
	Cube(CubeType type);

	void draw(Shader& shader);

private:
	CubeType type;
	d_uint VAO, VBO;
};
