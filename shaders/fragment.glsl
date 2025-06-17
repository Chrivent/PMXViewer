#version 330 core
in vec3 FragNormal;
in vec2 FragUV;

out vec4 FragColor;

void main() {
    vec3 lightDir = normalize(vec3(0.5, 1.0, 0.8));
    float lighting = max(dot(normalize(FragNormal), lightDir), 0.3);
    FragColor = vec4(vec3(1.0, 0.8, 0.6) * lighting, 1.0);
}