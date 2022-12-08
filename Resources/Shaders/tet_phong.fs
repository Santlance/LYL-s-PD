#version 430 core

in vec3 fragNormal;     // Ƭ�η��������ǵ�λ������
in vec3 FragPos;        // Ƭ��λ��

out vec4 FragColor;

uniform vec3 viewPos;       // �۲���λ��


// ����
struct Material
{
    vec3 ambient;
    vec3 diffuse;
    vec3 specular;
    float shininess;    // ����ȣ�Խ�ߣ���������Խǿ��ɢ���Խ�٣��߹��ԽС
};
uniform Material material;  // �������



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
    //vec3 ambient = light.ambient * vec3(texture(material.texture_diffuse1, TexCoords));
    // ���������
    float diff = max(dot(normal, lightDir), 0.0);
    //vec3 diffuse = light.diffuse * diff * vec3(texture(material.texture_diffuse1, TexCoords));
    // �������
    vec3 reflectDir = reflect(-lightDir, normal);
    float spec = pow(max(dot(viewDir, reflectDir), 0.0), material.shininess);   // �����С�����ȡ�������ϵ��
    //vec3 specular = light.specular * spec * vec3(texture(material.texture_specular1, TexCoords));
    
    // û������
    vec3 ambient = light.ambient * material.ambient;
    vec3 diffuse = light.diffuse * diff * material.diffuse;
    vec3 specular = light.specular * spec * material.specular;

    // �ϲ�
    return (ambient + diffuse + specular);
}


uniform DirLight dirLight;

void main()
{
    vec3 norm = normalize(fragNormal);              // �������ĵ�λ����
    vec3 viewDir = normalize(viewPos - FragPos);    // �۲췽��ĵ�λ����

    vec3 result = calDirLight(dirLight, norm, viewDir);

    FragColor = vec4(result, 1.0f);
}
