#version 430 core



// ---------- 这一部分定义了一些必需的参数 ----------

// 从前面传下来的数据
in vec3 fragNormal;     // 片段法向量（非单位向量）
in vec3 FragPos;        // 片段位置
in vec2 TexCoords;      // 纹理采样坐标

// 片段着色器的颜色输出
out vec4 FragColor;

// 物体材质
struct Material
{
    // 注意这里的纹理命名规范
    sampler2D texture_diffuse1;
    sampler2D texture_specular1;
    float shininess;    // 反光度：越高，反光能力越强，散射的越少，高光点越小
};
uniform Material material;  // 物体材质
uniform vec3 viewPos;       // 观察者位置

// ----------------------------------------------------








// 这一部分实现了冯氏光照模型在三种不同光源下的计算方法：方向光、点光源、聚光灯（可以根据需要调用）
// 函数参数的normal、viewDir必须是单位向量

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
    vec3 ambient = light.ambient * vec3(texture(material.texture_diffuse1, TexCoords));
    // 漫反射光照
    float diff = max(dot(normal, lightDir), 0.0);
    vec3 diffuse = light.diffuse * diff * vec3(texture(material.texture_diffuse1, TexCoords));
    // 镜面光照
    vec3 reflectDir = reflect(-lightDir, normal);
    float spec = pow(max(dot(viewDir, reflectDir), 0.0), material.shininess);   // 材质中【反光度】决定的系数
    vec3 specular = light.specular * spec * vec3(texture(material.texture_specular1, TexCoords));
    
    // 没有纹理
    //vec3 ambient = light.ambient * vec3(1.0f ,1.0f, 1.0f);
    //vec3 diffuse = light.diffuse * diff * vec3(1.0f ,1.0f, 1.0f);
    //vec3 specular = light.specular * spec * vec3(1.0f ,1.0f, 1.0f);

    // 合并
    return (ambient + diffuse + specular);
}
//-------------------------------------------

//----------------- 点光源 ------------------
struct PointLight
{
    vec3 position;  // 光源位置
    // 三个衰减系数
    float constant;
    float linear;
    float quadratic;
    // 三个光照分量
    vec3 ambient;
    vec3 diffuse;
    vec3 specular;
};

vec3 calPointLight(PointLight light, vec3 normal, vec3 fragPos, vec3 viewDir)
{
    vec3 lightDir = normalize(light.position - fragPos);
    // 环境光照
    vec3 ambient = light.ambient * vec3(texture(material.texture_diffuse1, TexCoords));
    // 漫反射光照
    float diff = max(dot(normal, lightDir), 0.0);
    vec3 diffuse = light.diffuse * diff * vec3(texture(material.texture_diffuse1, TexCoords));
    // 镜面光照
    vec3 reflectDir = reflect(-lightDir, normal);
    float spec = pow(max(dot(viewDir, reflectDir), 0.0), material.shininess);   // 材质中【反光度】决定的系数
    vec3 specular = light.specular * spec * vec3(texture(material.texture_specular1, TexCoords));
    // 合并，考虑光强衰减
    float distance = length(light.position - fragPos);
    float luminosity = 1.0 / (light.constant + light.linear * distance + light.quadratic * distance * distance);   // 光强衰减系数
    return (ambient + diffuse + specular) * luminosity;
}
//-------------------------------------------

//----------------- 聚光灯 ------------------
struct SpotLight
{
    vec3 direction;     // 光照方向
    vec3 position;      // 光源位置
    float innerCos;     // 内圆锥半角余弦值
    float outerCos;     // 外圆锥半角余弦值
    // 三个衰减系数
    float constant;
    float linear;
    float quadratic;
    // 三个光照分量
    vec3 ambient;
    vec3 diffuse;
    vec3 specular;
};

vec3 calSpotLight(SpotLight light, vec3 normal, vec3 fragPos, vec3 viewDir)
{
    vec3 lightDir = normalize(light.position - fragPos);
    // 环境光照
    vec3 ambient = light.ambient * vec3(texture(material.texture_diffuse1, TexCoords));
    // 漫反射光照
    float diff = max(dot(normal, lightDir), 0.0);
    vec3 diffuse = light.diffuse * diff * vec3(texture(material.texture_diffuse1, TexCoords));
    // 镜面光照
    vec3 reflectDir = reflect(-lightDir, normal);
    float spec = pow(max(dot(viewDir, reflectDir), 0.0), material.shininess);   // 材质中【反光度】决定的系数
    vec3 specular = light.specular * spec * vec3(texture(material.texture_specular1, TexCoords));
    // 考虑光强衰减
    float distance = length(light.position - fragPos);
    float luminosity = 1.0 / (light.constant + light.linear * distance + light.quadratic * distance * distance);   // 光强衰减系数
    // 从这里开始加入聚光灯的判断
    float theta = dot(lightDir, normalize(-light.direction));
    float epsilon = light.innerCos - light.outerCos;
    float intensity = clamp((theta - light.outerCos) / epsilon, 0.0, 1.0);    // 聚光灯决定的光强系数

    return (ambient + diffuse + specular) * luminosity * intensity;
}
//-------------------------------------------







// 这一部分是主程序

uniform DirLight dirLight;
//#define NR_POINT_LIGHTS 1
//uniform PointLight pointLights[NR_POINT_LIGHTS];
//uniform SpotLight spotLight;

uniform int isLineMesh;

void main()
{
    vec3 norm = normalize(fragNormal);              // 法向量的单位向量
    vec3 viewDir = normalize(viewPos - FragPos);    // 观察方向的单位向量
    vec3 result = vec3(0.0f, 0.0f, 0.0f);

    result += calDirLight(dirLight, norm, viewDir);
    //for(int i = 0; i < NR_POINT_LIGHTS; i++)
        //result += calPointLight(pointLights[i], norm, FragPos, viewDir);
    //result += calSpotLight(spotLight, norm, FragPos, viewDir);

    if(isLineMesh == 0)
        FragColor = vec4(result, 1.0f);
    else
        FragColor = vec4(1.0f, 1.0f, 1.0f, 1.0f);
}

