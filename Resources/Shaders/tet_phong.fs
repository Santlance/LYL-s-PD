#version 430 core

in vec3 fragNormal;     // 片段法向量（非单位向量）
in vec3 FragPos;        // 片段位置

out vec4 FragColor;

uniform vec3 viewPos;       // 观察者位置


// 材质
struct Material
{
    vec3 ambient;
    vec3 diffuse;
    vec3 specular;
    float shininess;    // 反光度：越高，反光能力越强，散射的越少，高光点越小
};
uniform Material material;  // 物体材质



//----------------- 方向光 ------------------
struct DirLight
{
    vec3 direction;     // 光照方向
    // 三个光照分量
    vec3 ambient;
    vec3 diffuse;
    vec3 specular;
};

vec3 calDirLight(DirLight light, vec3 normal, vec3 viewDir)
{
    vec3 lightDir = normalize(-light.direction);
    // 环境光照
    //vec3 ambient = light.ambient * vec3(texture(material.texture_diffuse1, TexCoords));
    // 漫反射光照
    float diff = max(dot(normal, lightDir), 0.0);
    //vec3 diffuse = light.diffuse * diff * vec3(texture(material.texture_diffuse1, TexCoords));
    // 镜面光照
    vec3 reflectDir = reflect(-lightDir, normal);
    float spec = pow(max(dot(viewDir, reflectDir), 0.0), material.shininess);   // 材质中【反光度】决定的系数
    //vec3 specular = light.specular * spec * vec3(texture(material.texture_specular1, TexCoords));
    
    // 没有纹理
    vec3 ambient = light.ambient * material.ambient;
    vec3 diffuse = light.diffuse * diff * material.diffuse;
    vec3 specular = light.specular * spec * material.specular;

    // 合并
    return (ambient + diffuse + specular);
}


uniform DirLight dirLight;

void main()
{
    vec3 norm = normalize(fragNormal);              // 法向量的单位向量
    vec3 viewDir = normalize(viewPos - FragPos);    // 观察方向的单位向量

    vec3 result = calDirLight(dirLight, norm, viewDir);

    FragColor = vec4(result, 1.0f);
}
