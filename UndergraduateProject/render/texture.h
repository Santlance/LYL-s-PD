#pragma once

#include <glad/glad.h>
#include "stb_image.h"
#include <iostream>
#include <string>
#include <assimp/Importer.hpp>
#include <assimp/scene.h>
#include <assimp/postprocess.h>
#include <map>

// 纹理类型
enum class TextureType
{
	DIFFUSE,
	SPECULAR,
	NORMAL,
	HEIGHT,
	// 不需要纹理类型的时候，定义为NONE
	NONE
};


class Texture
{
public:
	unsigned int ID = 0;	// 纹理编号
	TextureType type;		// 纹理类型
	std::string path;		// 纹理数据的路径

	// 构造函数
	Texture() {}
	Texture(const char* image_path, TextureType type, bool flipY = false);
	Texture(std::string image_path, TextureType type, bool flipY = false);
	Texture(const aiTexture* aiTex, TextureType type, bool flipY = false);

	// 指定在第几个纹理单元用这个纹理
	// textureIndex可以是0,1,2,...，也可以是GL_TEXTURE0等
	void use(int textureIndex);

private:
	void load_texture(bool flipY, unsigned char* data, int width, int height, int nrChannels);
	unsigned char* load_data(const char* image_path, int* width, int* height, int* nrChannels);
	unsigned char* load_data(const aiTexture* aiTex, int* width, int* height, int* nrChannels);
};
