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

	// 生成一个纹理的基本过程
	// --------------------------------------------
	// 1. 生成一个纹理编号
	glGenTextures(1, &ID);
	// 2. 绑定纹理
	glBindTexture(GL_TEXTURE_2D, ID);
	// 3. 设置纹理环绕方式（纹理比图像小的时候怎么处理，默认为重复）
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

	// 4. 设置纹理过滤方式 Texture Filtering
	//		(a)邻近过滤(GL_NEAREST)：选取最近的像素点
	//		(b)线性过滤(GL+LINEAR)：基于附近的像素颜色计算一个插值
	// 在进行放大和缩小操作的时候可以设置纹理过滤选项
	//		(c)多级渐远纹理 Mipmap
	//			在图形缩放到很小的时候，为了避免失真（不知道如何获取正确的颜色值），使用Mipmap去适配最合适的那个
	//			只在缩小的时候才可以使用Mipmap
	//glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

	// 5. 生成纹理，然后释放图像内存
	if (flipY) stbi_set_flip_vertically_on_load(true);

	GLenum format = GL_RGB;
	if (nrChannels == 1) format = GL_RED;
	else if (nrChannels == 3) format = GL_RGB;
	else if (nrChannels == 4) format = GL_RGBA;
	// 根据颜色通道数目来决定纹理数据格式
	glTexImage2D(GL_TEXTURE_2D, 0, format, width, height, 0, format, GL_UNSIGNED_BYTE, data);
	glGenerateMipmap(GL_TEXTURE_2D);	// 生成多级渐远纹理图像

	if (flipY) stbi_set_flip_vertically_on_load(false);
	stbi_image_free(data);
	// 6. 解除绑定
	glBindTexture(GL_TEXTURE_2D, 0);
	// --------------------------------------------
}

unsigned char* Texture::load_data(const char* image_path, int* width, int* height, int* nrChannels)
{
	// 保存路径
	this->path = std::string(image_path);

	return stbi_load(image_path, width, height, nrChannels, 0);
}

unsigned char* Texture::load_data(const aiTexture* aiTex, int* width, int* height, int* nrChannels)
{
	// 保存路径
	this->path = std::string(aiTex->mFilename.C_Str());

	unsigned int image_size = 1;
	if (aiTex->mHeight != 0) image_size *= aiTex->mHeight;
	if (aiTex->mWidth != 0) image_size *= aiTex->mWidth;

	return stbi_load_from_memory(reinterpret_cast<unsigned char*>(aiTex->pcData), image_size, width, height, nrChannels, 0);
}

