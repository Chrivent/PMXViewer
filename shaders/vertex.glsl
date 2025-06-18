#version 330 core
layout(location = 0) in vec3 aPos;
layout(location = 1) in vec3 aNormal;
layout(location = 2) in vec2 aUV;
layout(location = 3) in ivec4 aBoneIndices;   // �� �ε���
layout(location = 4) in vec4  aBoneWeights;   // �� ����ġ

out vec3 FragNormal;
out vec2 FragUV;

uniform mat4 model;
uniform mat4 view;
uniform mat4 projection;
uniform mat4 boneMatrices[512]; // �ִ� �� �� (�ʿ� �� �ø���)

void main() {
    // ��Ų ��Ʈ���� ��� (Linear Blend Skinning)
    mat4 skinMatrix =
        aBoneWeights[0] * boneMatrices[aBoneIndices[0]] +
        aBoneWeights[1] * boneMatrices[aBoneIndices[1]] +
        aBoneWeights[2] * boneMatrices[aBoneIndices[2]] +
        aBoneWeights[3] * boneMatrices[aBoneIndices[3]];

    // ��ġ�� ��� ��Ű�� ����
    vec4 skinnedPos = skinMatrix * vec4(aPos, 1.0);
    vec3 skinnedNormal = mat3(skinMatrix) * aNormal;

    gl_Position = projection * view * model * skinnedPos;

    FragNormal = mat3(model) * skinnedNormal; // ����� ���
    FragUV = aUV;
}