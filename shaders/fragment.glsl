#version 330 core
in vec3 FragNormal;
in vec2 FragUV;

out vec4 FragColor;

uniform sampler2D tex;
uniform sampler2D toonTex;
uniform sampler2D sphereTex;
uniform int sphereMode;

uniform bool useToon;
uniform bool useSphere;

void main() {
    // 조명 벡터 설정
    vec3 normal = normalize(FragNormal);
    vec3 lightDir = normalize(vec3(0.5, 1.0, -0.8));

    // 조명 세기
    float diffuse = max(dot(normal, lightDir), 0.0);

    // Toon 텍스처에서 음영 색 샘플링 (Y축 기준, 위가 밝음)
    vec3 toonShade = useToon
        ? texture(toonTex, vec2(0.5, 1.0 - diffuse)).rgb
        : vec3(1.0);

    // 스피어 텍스처에서 광택 샘플링 (Add 모드 기준)
    vec3 sphere = useSphere
        ? texture(sphereTex, FragUV).rgb
        : vec3(0.0);

    // 원본 텍스처
    vec4 texColor = texture(tex, FragUV);

    // 최종 색: Toon 셰이딩 × 텍스처 + 하이라이트
    vec3 finalColor = texColor.rgb * toonShade;

    // Sphere 모드 적용
    if (useSphere) {
        if (sphereMode == 1) {
            finalColor *= sphere;  // 승산
        } else if (sphereMode == 2) {
            finalColor += sphere * 0.3;  // 가산
        }
    }

    FragColor = vec4(finalColor, texColor.a);
}