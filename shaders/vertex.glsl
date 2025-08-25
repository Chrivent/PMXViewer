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
    // 기본 값(CPU 스키닝 모드에서는 그대로 사용)
    vec4 pos = vec4(position, 1.0);
    vec3 nrm = normal;

    // 월드 변환
    vec4 worldPos = model * pos;
    FragWorldPos = worldPos.xyz;

    // 노멀은 model의 비정규 스케일 대응을 위해 normal matrix 사용
    mat3 normalMatrix = transpose(inverse(mat3(model)));
    FragNormal = normalize(normalMatrix * nrm);

    // 최종 위치
    gl_Position = projection * view * worldPos;

    FragUV = uv;
}