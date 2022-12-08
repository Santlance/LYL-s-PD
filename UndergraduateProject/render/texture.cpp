#include "texture.h"

Texture::Texture(const char* image_path, TextureType type, bool flipY)
{
	this->type = type;

	int width, height, nrChannels;
	unsigned char* data = load_data(image_path, &width, &height, &nrChannels);
	load_texture(flipY, data, width, height, nrChannels);
}

Texture::Texture(std::string image_path, TextureType type, bool flipY)
{
	*this = Texture(image_path.c_str(), type, flipY);
}

Texture::Texture(const aiTexture* aiTex, TextureType type, bool flipY)
{
	this->type = type;

	int width, height, nrChannels;
	unsigned char* data = load_data(aiTex, &width, &height, &nrChannels);
	load_texture(flipY, data, width, height, nrChannels);
}

void Texture::use(int textureIndex)
{
	int x;
	glGetIntegerv(GL_MAX_TEXTURE_IMAGE_UNITS, &x);
	if (textureIndex < x) textureIndex += GL_TEXTURE0;

	glActiveTexture(textureIndex);
	glBindTexture(GL_TEXTURE_2D, ID);
}

void Texture::load_texture(bool flipY, unsigned char* data, int width, int height, int nrChannels)
{
	if (!data)
	{
		std::cout << "load texture error:  " << std::endl;
		return;
	}

	// ����һ������Ļ�������
	// --------------------------------------------
	// 1. ����һ��������
	glGenTextures(1, &ID);
	// 2. ������
	glBindTexture(GL_TEXTURE_2D, ID);
	// 3. ���������Ʒ�ʽ�������ͼ��С��ʱ����ô����Ĭ��Ϊ�ظ���
	if (nrChannels <= 3)
	{
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
	}
	else if (nrChannels == 4)	// RGBA
	{
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
	}
	//float borderColor[] = { 1.0f, 1.0f, 0.0f, 1.0f };
	//glTexParameterfv(GL_TEXTURE_2D, GL_TEXTURE_BORDER_COLOR, borderColor);

	// 4. ����������˷�ʽ Texture Filtering
	//		(a)�ڽ�����(GL_NEAREST)��ѡȡ��������ص�
	//		(b)���Թ���(GL+LINEAR)�����ڸ�����������ɫ����һ����ֵ
	// �ڽ��зŴ����С������ʱ����������������ѡ��
	//		(c)�༶��Զ���� Mipmap
	//			��ͼ�����ŵ���С��ʱ��Ϊ�˱���ʧ�棨��֪����λ�ȡ��ȷ����ɫֵ����ʹ��Mipmapȥ��������ʵ��Ǹ�
	//			ֻ����С��ʱ��ſ���ʹ��Mipmap
	//glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

	// 5. ��������Ȼ���ͷ�ͼ���ڴ�
	if (flipY) stbi_set_flip_vertically_on_load(true);

	GLenum format = GL_RGB;
	if (nrChannels == 1) format = GL_RED;
	else if (nrChannels == 3) format = GL_RGB;
	else if (nrChannels == 4) format = GL_RGBA;
	// ������ɫͨ����Ŀ�������������ݸ�ʽ
	glTexImage2D(GL_TEXTURE_2D, 0, format, width, height, 0, format, GL_UNSIGNED_BYTE, data);
	glGenerateMipmap(GL_TEXTURE_2D);	// ���ɶ༶��Զ����ͼ��

	if (flipY) stbi_set_flip_vertically_on_load(false);
	stbi_image_free(data);
	// 6. �����
	glBindTexture(GL_TEXTURE_2D, 0);
	// --------------------------------------------
}

unsigned char* Texture::load_data(const char* image_path, int* width, int* height, int* nrChannels)
{
	// ����·��
	this->path = std::string(image_path);

	return stbi_load(image_path, width, height, nrChannels, 0);
}

unsigned char* Texture::load_data(const aiTexture* aiTex, int* width, int* height, int* nrChannels)
{
	// ����·��
	this->path = std::string(aiTex->mFilename.C_Str());

	unsigned int image_size = 1;
	if (aiTex->mHeight != 0) image_size *= aiTex->mHeight;
	if (aiTex->mWidth != 0) image_size *= aiTex->mWidth;

	return stbi_load_from_memory(reinterpret_cast<unsigned char*>(aiTex->pcData), image_size, width, height, nrChannels, 0);
}

