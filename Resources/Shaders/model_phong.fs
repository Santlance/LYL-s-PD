#version 430 core



// ---------- ��һ���ֶ�����һЩ����Ĳ��� ----------

// ��ǰ�洫����������
in vec3 fragNormal;     // Ƭ�η��������ǵ�λ������
in vec3 FragPos;        // Ƭ��λ��
in vec2 TexCoords;      // �����������

// Ƭ����ɫ������ɫ���
out vec4 FragColor;

// �������
struct Material
{
    // ע����������������淶
    sampler2D texture_diffuse1;
    sampler2D texture_specular1;
    float shininess;    // ����ȣ�Խ�ߣ���������Խǿ��ɢ���Խ�٣��߹��ԽС
};
uniform Material material;  // �������
uniform vec3 viewPos;       // �۲���λ��

// ----------------------------------------------------








// ��һ����ʵ���˷��Ϲ���ģ�������ֲ�ͬ��Դ�µļ��㷽��������⡢���Դ���۹�ƣ����Ը�����Ҫ���ã�
// ����������normal��viewDir�����ǵ�λ����

//----------------- ����� ------------------
struct DirLight
{
    vec3 direction;     // ���շ���
    // �������շ���
    vec3 ambient;
    vec3 diffuse;
    vec3 specular;
};

vec3 calDirLight(DirLight light, vec3 normal, vec3 viewDir)
{
    vec3 lightDir = normalize(-light.direction);
    // ��������
    vec3 ambient = light.ambient * vec3(texture(material.texture_diffuse1, TexCoords));
    // ���������
    float diff = max(dot(normal, lightDir), 0.0);
    vec3 diffuse = light.diffuse * diff * vec3(texture(material.texture_diffuse1, TexCoords));
    // �������
    vec3 reflectDir = reflect(-lightDir, normal);
    float spec = pow(max(dot(viewDir, reflectDir), 0.0), material.shininess);   // �����С�����ȡ�������ϵ��
    vec3 specular = light.specular * spec * vec3(texture(material.texture_specular1, TexCoords));
    
    // û������
    //vec3 ambient = light.ambient * vec3(1.0f ,1.0f, 1.0f);
    //vec3 diffuse = light.diffuse * diff * vec3(1.0f ,1.0f, 1.0f);
    //vec3 specular = light.specular * spec * vec3(1.0f ,1.0f, 1.0f);

    // �ϲ�
    return (ambient + diffuse + specular);
}
//-------------------------------------------

//----------------- ���Դ ------------------
struct PointLight
{
    vec3 position;  // ��Դλ��
    // ����˥��ϵ��
    float constant;
    float linear;
    float quadratic;
    // �������շ���
    vec3 ambient;
    vec3 diffuse;
    vec3 specular;
};

vec3 calPointLight(PointLight light, vec3 normal, vec3 fragPos, vec3 viewDir)
{
    vec3 lightDir = normalize(light.position - fragPos);
    // ��������
    vec3 ambient = light.ambient * vec3(texture(material.texture_diffuse1, TexCoords));
    // ���������
    float diff = max(dot(normal, lightDir), 0.0);
    vec3 diffuse = light.diffuse * diff * vec3(texture(material.texture_diffuse1, TexCoords));
    // �������
    vec3 reflectDir = reflect(-lightDir, normal);
    float spec = pow(max(dot(viewDir, reflectDir), 0.0), material.shininess);   // �����С�����ȡ�������ϵ��
    vec3 specular = light.specular * spec * vec3(texture(material.texture_specular1, TexCoords));
    // �ϲ������ǹ�ǿ˥��
    float distance = length(light.position - fragPos);
    float luminosity = 1.0 / (light.constant + light.linear * distance + light.quadratic * distance * distance);   // ��ǿ˥��ϵ��
    return (ambient + diffuse + specular) * luminosity;
}
//-------------------------------------------

//----------------- �۹�� ------------------
struct SpotLight
{
    vec3 direction;     // ���շ���
    vec3 position;      // ��Դλ��
    float innerCos;     // ��Բ׶�������ֵ
    float outerCos;     // ��Բ׶�������ֵ
    // ����˥��ϵ��
    float constant;
    float linear;
    float quadratic;
    // �������շ���
    vec3 ambient;
    vec3 diffuse;
    vec3 specular;
};

vec3 calSpotLight(SpotLight light, vec3 normal, vec3 fragPos, vec3 viewDir)
{
    vec3 lightDir = normalize(light.position - fragPos);
    // ��������
    vec3 ambient = light.ambient * vec3(texture(material.texture_diffuse1, TexCoords));
    // ���������
    float diff = max(dot(normal, lightDir), 0.0);
    vec3 diffuse = light.diffuse * diff * vec3(texture(material.texture_diffuse1, TexCoords));
    // �������
    vec3 reflectDir = reflect(-lightDir, normal);
    float spec = pow(max(dot(viewDir, reflectDir), 0.0), material.shininess);   // �����С�����ȡ�������ϵ��
    vec3 specular = light.specular * spec * vec3(texture(material.texture_specular1, TexCoords));
    // ���ǹ�ǿ˥��
    float distance = length(light.position - fragPos);
    float luminosity = 1.0 / (light.constant + light.linear * distance + light.quadratic * distance * distance);   // ��ǿ˥��ϵ��
    // �����￪ʼ����۹�Ƶ��ж�
    float theta = dot(lightDir, normalize(-light.direction));
    float epsilon = light.innerCos - light.outerCos;
    float intensity = clamp((theta - light.outerCos) / epsilon, 0.0, 1.0);    // �۹�ƾ����Ĺ�ǿϵ��

    return (ambient + diffuse + specular) * luminosity * intensity;
}
//-------------------------------------------







// ��һ������������

uniform DirLight dirLight;
//#define NR_POINT_LIGHTS 1
//uniform PointLight pointLights[NR_POINT_LIGHTS];
//uniform SpotLight spotLight;

uniform int isLineMesh;

void main()
{
    vec3 norm = normalize(fragNormal);              // �������ĵ�λ����
    vec3 viewDir = normalize(viewPos - FragPos);    // �۲췽��ĵ�λ����
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

