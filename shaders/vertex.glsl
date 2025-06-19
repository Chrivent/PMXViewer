#version 330 core
layout(location = 0) in vec3 position;
layout(location = 1) in vec3 normal;
layout(location = 2) in vec2 uv;
layout(location = 3) in ivec4 boneIndices;   // 본 인덱스
layout(location = 4) in vec4  boneWeights;   // 본 가중치

out vec3 FragNormal;
out vec2 FragUV;
out vec3 FragWorldPos;

uniform mat4 model;
uniform mat4 view;
uniform mat4 projection;
uniform mat4 skinMatrices[512]; // 최대 본 수 (필요 시 늘리기)

void main() {
    // 스킨 매트릭스 계산 (Linear Blend Skinning)
    mat4 skinMatrix =
        boneWeights[0] * skinMatrices[boneIndices[0]] +
        boneWeights[1] * skinMatrices[boneIndices[1]] +
        boneWeights[2] * skinMatrices[boneIndices[2]] +
        boneWeights[3] * skinMatrices[boneIndices[3]];

    // 위치와 노멀 스키닝 적용
    vec4 skinnedPos = skinMatrix * vec4(position, 1.0);
    vec3 skinnedNormal = mat3(skinMatrix) * normal;

    vec4 worldPos = model * skinnedPos;
    FragWorldPos = worldPos.xyz;

    gl_Position = projection * view * worldPos;

    FragNormal = mat3(model) * skinnedNormal; // 조명용 노멀
    FragUV = uv;
}