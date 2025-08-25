#version 330 core
layout(location = 0) in vec3 position;
layout(location = 1) in vec3 normal;
layout(location = 2) in vec2 uv;

out vec3 FragNormal;
out vec2 FragUV;
out vec3 FragWorldPos;

uniform mat4 model;
uniform mat4 view;
uniform mat4 projection;

void main() {
    // �⺻ ��(CPU ��Ű�� ��忡���� �״�� ���)
    vec4 pos = vec4(position, 1.0);
    vec3 nrm = normal;

    // ���� ��ȯ
    vec4 worldPos = model * pos;
    FragWorldPos = worldPos.xyz;

    // ����� model�� ������ ������ ������ ���� normal matrix ���
    mat3 normalMatrix = transpose(inverse(mat3(model)));
    FragNormal = normalize(normalMatrix * nrm);

    // ���� ��ġ
    gl_Position = projection * view * worldPos;

    FragUV = uv;
}