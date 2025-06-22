#version 330 core
in vec3 FragNormal;
in vec2 FragUV;
in vec3 FragWorldPos;

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

uniform vec3 cameraPos;
uniform vec3 lightDir;

void main() {
    vec3 N = normalize(FragNormal);
    vec3 L = normalize(-lightDir);
    vec3 V = normalize(cameraPos - FragWorldPos);
    vec3 R = reflect(-L, N);

    vec4 texColor = texture(tex, FragUV);
    vec3 base = texColor.rgb * diffuse.rgb;

    float NdotL = clamp(dot(N, L), 0.0, 1.0);
    vec3 toonCol = bUseToon ? texture(toonTex, vec2(0.0, 1.0 - NdotL)).rgb : vec3(1.0);
    vec3 diffCol = base * toonCol;

    float specB = pow(max(dot(R, V), 0.0), specularPower);
    vec3 specCol = specular * specB;

    vec3 ambCol = clamp(ambient, vec3(0.0), vec3(0.5)) - vec3(0.5);
    vec3 finalRgb = diffCol + specCol + ambCol;

    float finalAlpha = diffuse.a * texColor.a * Alpha;
    FragColor = vec4(finalRgb , finalAlpha);
}