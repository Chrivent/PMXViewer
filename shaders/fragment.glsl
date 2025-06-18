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
    // ���� ���� ����
    vec3 normal = normalize(FragNormal);
    vec3 lightDir = normalize(vec3(0.5, 1.0, -0.8));

    // ���� ����
    float diffuse = max(dot(normal, lightDir), 0.0);

    // Toon �ؽ�ó���� ���� �� ���ø� (Y�� ����, ���� ����)
    vec3 toonShade = useToon
        ? texture(toonTex, vec2(0.5, 1.0 - diffuse)).rgb
        : vec3(1.0);

    // ���Ǿ� �ؽ�ó���� ���� ���ø� (Add ��� ����)
    vec3 sphere = useSphere
        ? texture(sphereTex, FragUV).rgb
        : vec3(0.0);

    // ���� �ؽ�ó
    vec4 texColor = texture(tex, FragUV);

    // ���� ��: Toon ���̵� �� �ؽ�ó + ���̶���Ʈ
    vec3 finalColor = texColor.rgb * toonShade;

    // Sphere ��� ����
    if (useSphere) {
        if (sphereMode == 1) {
            finalColor *= sphere;  // �»�
        } else if (sphereMode == 2) {
            finalColor += sphere * 0.3;  // ����
        }
    }

    FragColor = vec4(finalColor, texColor.a);
}