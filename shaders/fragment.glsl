#version 330 core
in vec3 FragNormal;
in vec2 FragUV;
in vec3 FragWorldPos; // 월드 공간의 정점 위치

out vec4 FragColor;

uniform sampler2D tex;
uniform sampler2D toonTex;
uniform sampler2D sphereTex;
uniform int sphereMode;

uniform vec4 diffuse;
uniform vec3 specular;
uniform float specularPower;
uniform vec3 ambient;
uniform float Alpha;

uniform bool bUseToon;
uniform bool bUseSphere;

uniform vec3 cameraPos; // 카메라 위치
uniform vec3 lightDir;

void main() {
    vec3 normal = normalize(FragNormal);
    vec4 texColor = texture(tex, FragUV);

    vec3 baseColor = texColor.rgb * diffuse.rgb;

    float NdotL = max(dot(normal, normalize(-lightDir)), 0.0);

    vec3 lightColor;
    if (bUseToon) {
        lightColor = texture(toonTex, vec2(0.0, 1.0 - NdotL)).rgb;
    } else {
        lightColor = vec3(1.0);
    }

    vec3 finalColor = baseColor * lightColor + clamp(ambient, vec3(0.0), vec3(0.5)) - vec3(0.5);

    float finalAlpha = diffuse.a * texColor.a * Alpha;

    FragColor = vec4(finalColor, finalAlpha);
}