#pragma once

#include <glad/glad.h>
#include "stb_image.h"
#include <iostream>
#include <string>
#include <assimp/Importer.hpp>
#include <assimp/scene.h>
#include <assimp/postprocess.h>
#include <map>

// ��������
enum class TextureType
{
	DIFFUSE,
	SPECULAR,
	NORMAL,
	HEIGHT,
	// ����Ҫ�������͵�ʱ�򣬶���ΪNONE
	NONE
};


class Texture
{
public:
	unsigned int ID = 0;	// ������
	TextureType type;		// ��������
	std::string path;		// �������ݵ�·��

	// ���캯��
	Texture() {}
	Texture(const char* image_path, TextureType type, bool flipY = false);
	Texture(std::string image_path, TextureType type, bool flipY = false);
	Texture(const aiTexture* aiTex, TextureType type, bool flipY = false);

	// ָ���ڵڼ�������Ԫ���������
	// textureIndex������0,1,2,...��Ҳ������GL_TEXTURE0��
	void use(int textureIndex);

private:
	void load_texture(bool flipY, unsigned char* data, int width, int height, int nrChannels);
	unsigned char* load_data(const char* image_path, int* width, int* height, int* nrChannels);
	unsigned char* load_data(const aiTexture* aiTex, int* width, int* height, int* nrChannels);
};
