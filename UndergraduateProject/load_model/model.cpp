#include "model.h"

Model::Model(const std::string& path)
{
	loadModel(path);
}

void Model::draw(Shader& shader)
{
	// 绘制每一个mesh
	for (unsigned int i = 0; i < meshes.size(); i++)
		meshes[i].draw(shader);
}


void Model::loadModel(const std::string& path)
{
	Assimp::Importer importer;
	// scene中包含了所有数据，而下面的node等包含了索引信息
	const aiScene* scene = importer.ReadFile(
		path,
		// aiProcess_Triangulate 将图元形状变成三角形
		// aiProcess_GenSmoothNormals 如果没有法向量，建立法向量
		// aiProcess_FlipUVs 翻转y轴纹理坐标
		// aiProcess_CalcTangentSpace 生成切线和副切线
		// 更多查看 http://assimp.sourceforge.net/lib_html/postprocess_8h.html
		aiProcess_Triangulate | aiProcess_GenSmoothNormals | aiProcess_FlipUVs | aiProcess_CalcTangentSpace
	);
	// 检查是否成功加载
	if (!scene || (scene->mFlags && AI_SCENE_FLAGS_INCOMPLETE) || !scene->mRootNode)
	{
		std::cout << "ERROR::ASSIMP::" << importer.GetErrorString() << std::endl;
		return;
	}

	// 一些调试信息
	std::cout << "Mesh number:  " << scene->mNumMeshes << std::endl;
	std::cout << "Materials number:  " << scene->mNumMaterials << std::endl;

	// 保存目录
	directory = path.substr(0, path.find_last_of('/'));
	// 调用函数递归处理node
	processNode(scene->mRootNode, scene);
}

void Model::processNode(aiNode* node, const aiScene* scene)
{
	// 处理node里面的所有mesh
	for (unsigned int i = 0; i < node->mNumMeshes; i++)
	{
		aiMesh* mesh = scene->mMeshes[node->mMeshes[i]];
		meshes.push_back(processMesh(mesh, scene));
	}
	// 然后递归处理所有儿子节点，所以这里是一个先序遍历
	for (unsigned int i = 0; i < node->mNumChildren; i++)
	{
		processNode(node->mChildren[i], scene);
	}
}

Mesh Model::processMesh(aiMesh* mesh, const aiScene* scene)
{
	// 定义mesh需要的数据容器
	std::vector<Vertex> vertices;
	std::vector<unsigned int> indices;
	std::vector<Texture> textures;

	// 遍历mesh的所有vertex（顶点），获取vertices
	for (unsigned int i = 0; i < mesh->mNumVertices; i++)
	{
		// 构建vertex
		Vertex vertex;
		// vertex.position
		vertex.position.x = mesh->mVertices[i].x;
		vertex.position.y = mesh->mVertices[i].y;
		vertex.position.z = mesh->mVertices[i].z;
		// vertex.normal
		if (mesh->HasNormals())
		{
			vertex.normal.x = mesh->mNormals[i].x;
			vertex.normal.y = mesh->mNormals[i].y;
			vertex.normal.z = mesh->mNormals[i].z;
		}
		// vertex.texCoords
		if (mesh->HasTextureCoords(0))
		{
			// 这里默认只使用了编号为0的纹理坐标? [PBUG]
			vertex.texCoords.x = mesh->mTextureCoords[0][i].x;
			vertex.texCoords.y = mesh->mTextureCoords[0][i].y;
		}
		else
			vertex.texCoords = glm::vec2(0, 0);
		// vertex.tangent, vertex.bitangent
		if (mesh->HasTangentsAndBitangents())
		{
			vertex.tangent.x = mesh->mTangents[i].x;
			vertex.tangent.y = mesh->mTangents[i].y;
			vertex.tangent.z = mesh->mTangents[i].z;

			vertex.bitangent.x = mesh->mBitangents[i].x;
			vertex.bitangent.y = mesh->mBitangents[i].y;
			vertex.bitangent.z = mesh->mBitangents[i].z;
		}

		vertices.push_back(vertex);
	}

	// 遍历mesh的所有face（图元？），获取indices
	for (unsigned int i = 0; i < mesh->mNumFaces; i++)
	{
		aiFace face = mesh->mFaces[i];
		// 把每个face的顶点索引依序保存
		for (unsigned int j = 0; j < face.mNumIndices; j++)
		{
			indices.push_back(face.mIndices[j]);
		}
	}

	if (mesh->mMaterialIndex >= 0)
	{
		// 处理材质信息，一个mesh只会有一个material，如果有多个Assimp会自动分割成多个mesh
		aiMaterial* material = scene->mMaterials[mesh->mMaterialIndex];

		// 约定shader里面的名字命名规范为：
		// "texture_" + ("diffuse"/"specular"/"normal"/"height") + to_string(N),   N = 1, 2, ...

		// 1. diffuse maps
		std::vector<Texture> diffuseMaps = loadMaterialTexture(material, aiTextureType_DIFFUSE, TextureType::DIFFUSE, scene);
		textures.insert(textures.end(), diffuseMaps.begin(), diffuseMaps.end());
		// 2. specular maps
		std::vector<Texture> specularMaps = loadMaterialTexture(material, aiTextureType_SPECULAR, TextureType::SPECULAR, scene);
		textures.insert(textures.end(), specularMaps.begin(), specularMaps.end());
		// 3. normal maps
		std::vector<Texture> normalMaps = loadMaterialTexture(material, aiTextureType_NORMALS, TextureType::NORMAL, scene);
		textures.insert(textures.end(), normalMaps.begin(), normalMaps.end());
		// 4. height maps
		std::vector<Texture> heightMaps = loadMaterialTexture(material, aiTextureType_HEIGHT, TextureType::HEIGHT, scene);
		textures.insert(textures.end(), heightMaps.begin(), heightMaps.end());
	}

	// return
	return Mesh(vertices, indices, textures);
}

std::vector<Texture> Model::loadMaterialTexture(aiMaterial* material, aiTextureType type, TextureType textureType, const aiScene* scene)
{
	std::vector<Texture> textures;

	// 这里每加载一个texture就要遍历一遍所有加载过的texture，待优化 [TODO]
	for (unsigned int i = 0; i < material->GetTextureCount(type); i++)
	{
		// 获取纹理相对路径
		aiString str;
		material->GetTexture(type, i, &str);
		// filename记录文理的路径，这里的路径是模型文件路径+纹理相对路径
		std::string filename(str.C_Str());
		filename = directory + '/' + filename;

		// 这里为了性能做了一个优化，用texturesLoaded来保存所有已经加载了的texture
		// 所以如果当前纹理已经加载过，则直接用即可，否则进行纹理加载
		bool flag = false;
		for (unsigned int j = 0; j < texturesLoaded.size(); j++)
		{
			if (texturesLoaded[j].path == filename)
			{
				textures.push_back(texturesLoaded[j]);
				flag = true;
				break;
			}
		}
		if (!flag)
		{
			// 加载纹理
			const aiTexture* aitexture = scene->GetEmbeddedTexture(filename.c_str());
			if (aitexture)
			{
				Texture texture(aitexture, textureType);
				textures.push_back(texture);
				texturesLoaded.push_back(texture);
			}
			else
			{
				Texture texture(filename, textureType);
				textures.push_back(texture);
				texturesLoaded.push_back(texture);
			}
		}
	}

	return textures;
}
