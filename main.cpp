#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <iostream>
#include <fstream>
#include <filesystem>
#include <unordered_map>
#include <fcntl.h>
#include <io.h>

#include "Pmx.h"
#include "Vmd.h"

#define GLM_ENABLE_EXPERIMENTAL
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtx/quaternion.hpp>

#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"

#define STB_IMAGE_RESIZE_IMPLEMENTATION
#include "stb_image_resize2.h"

using namespace std;

struct GLVertex {
    glm::vec3 position = glm::vec3();
    glm::vec3 normal = glm::vec3();
    glm::vec2 uv = glm::vec2();
    glm::ivec4 boneIndices = glm::ivec4(0);
    glm::vec4 boneWeights = glm::vec4(0.0f);
};

GLuint vao, vbo, ebo;
vector<GLVertex> gVertices;
vector<uint32_t> gIndices;
vector<GLuint> gTextures;

float cameraDistance = 10.0f;
float yaw = 0.0f;
float pitch = 0.0f;
bool rightMouseDown = false;
bool middleMouseDown = false;
double lastMouseX = 0.0, lastMouseY = 0.0;

glm::vec3 cameraTarget = glm::vec3(0, 5, 0);

float lightYaw = -45.0f;   // 초기 조명 방향 (degree)
float lightPitch = 45.0f;
bool leftMouseDown = false;

float test = 0;

void LoadTextures(const pmx::PmxModel& model, const string& pmxBaseDir) {
    gTextures.resize(model.texture_count, 0);

    for (int i = 0; i < model.texture_count; ++i) {
        string relPath(model.textures[i].begin(), model.textures[i].end()); // wstring → string
        filesystem::path texPath = filesystem::path(pmxBaseDir) / relPath;

        int w, h, channels;
        stbi_uc* data = stbi_load(texPath.string().c_str(), &w, &h, &channels, STBI_rgb_alpha);
        if (!data) {
            cerr << "텍스처 로딩 실패: " << texPath << "\n";
            continue;
        }

        GLuint texID;
        glGenTextures(1, &texID);
        glBindTexture(GL_TEXTURE_2D, texID);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, w, h, 0, GL_RGBA, GL_UNSIGNED_BYTE, data);
        glGenerateMipmap(GL_TEXTURE_2D);

        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

        gTextures[i] = texID;
        stbi_image_free(data);
    }
}

void ApplyMorph(const pmx::PmxModel& model, vector<GLVertex>& vertices, vector<glm::mat4>& boneMatrices, int morphIndex, float weight) {
    if (morphIndex < 0 || morphIndex >= model.morph_count) return;

    const auto& morph = model.morphs[morphIndex];

    switch (morph.morph_type) {
    case pmx::MorphType::Vertex:
        for (int i = 0; i < morph.offset_count; ++i) {
            const auto& offset = morph.vertex_offsets[i];
            int vi = offset.vertex_index;
            if (vi < 0 || vi >= static_cast<int>(vertices.size())) continue;

            vertices[vi].position += glm::vec3(
                offset.position_offset[0],
                offset.position_offset[1],
                offset.position_offset[2]
            ) * weight;
        }
        break;

    case pmx::MorphType::UV:
    case pmx::MorphType::AdditionalUV1:
    case pmx::MorphType::AdditionalUV2:
    case pmx::MorphType::AdditionalUV3:
    case pmx::MorphType::AdditionalUV4:
        for (int i = 0; i < morph.offset_count; ++i) {
            const auto& offset = morph.uv_offsets[i];
            int vi = offset.vertex_index;
            if (vi < 0 || vi >= static_cast<int>(vertices.size())) continue;

            vertices[vi].uv += glm::vec2(
                offset.uv_offset[0],
                offset.uv_offset[1]
            ) * weight;
        }
        break;

    case pmx::MorphType::Matrial:
        for (int i = 0; i < morph.offset_count; ++i) {
            const auto& offset = morph.material_offsets[i];
            int mi = offset.material_index;
            if (mi < 0 || mi >= model.material_count) continue;

            auto& mat = model.materials[mi];
            uint8_t op = offset.offset_operation;

            auto apply = [&](float* dst, const float* off, int len) {
                for (int j = 0; j < len; ++j) {
                    if (op == 0) // Mul
                        dst[j] *= (1.0f + off[j] * weight);  // 안정적 처리
                    else         // Add
                        dst[j] += off[j] * weight;
                }
                };

            apply(mat.diffuse, offset.diffuse, 4);
            apply(mat.specular, offset.specular, 3);
            mat.specularlity += offset.specularity * weight;
            apply(mat.ambient, offset.ambient, 3);
            apply(mat.edge_color, offset.edge_color, 4);
            mat.edge_size += offset.edge_size * weight;
        }
        break;

    case pmx::MorphType::Bone:
        for (int i = 0; i < morph.offset_count; ++i) {
            const auto& offset = morph.bone_offsets[i];
            int bi = offset.bone_index;
            if (bi < 0 || bi >= 512) continue;

            glm::vec3 t(offset.translation[0], offset.translation[1], offset.translation[2]);
            glm::quat r(offset.rotation[3], offset.rotation[0], offset.rotation[1], offset.rotation[2]); // wxyz

            // 모프 weight에 따라 보간된 트랜스폼 생성
            glm::mat4 trans = glm::translate(glm::mat4(1.0f), t * weight);
            glm::mat4 rot = glm::toMat4(glm::slerp(glm::quat(), r, weight));

            // 최종 행렬 적용: 기존 행렬에 추가 적용
            boneMatrices[bi] = trans * rot * boneMatrices[bi];
        }
        break;

    case pmx::MorphType::Group:
        for (int i = 0; i < morph.offset_count; ++i) {
            const auto& group = morph.group_offsets[i];
            ApplyMorph(model, vertices, boneMatrices, group.morph_index, group.morph_weight * weight);
        }
        break;

    default:
        break;
    }
}

int FindBoneIndexByName(const pmx::PmxModel& model, const string& name) {
    oguna::EncodingConverter converter;
    wstring wname;
    converter.Utf8ToUtf16(name.data(), (int)name.size(), &wname);
    for (int i = 0; i < model.bone_count; ++i)
    {
        const wstring boneName = model.bones[i].bone_name;
        if (boneName == wname) return i;
    }
    return -1;
}

int FindMorphIndexByName(const pmx::PmxModel& model, const string& name) {
    oguna::EncodingConverter converter;
    wstring wname;
    converter.Utf8ToUtf16(name.data(), (int)name.size(), &wname);
    for (int i = 0; i < model.morph_count; ++i) {
        wstring morphName = model.morphs[i].morph_name;
        if (morphName == wname) return i;
    }
    return -1;
}

class Shader {
public:
    GLuint ID;

    Shader(const char* vertexPath, const char* fragmentPath) {
        string vertexCode;
        string fragmentCode;
        ifstream vShaderFile(vertexPath);
        ifstream fShaderFile(fragmentPath);

        stringstream vShaderStream, fShaderStream;
        vShaderStream << vShaderFile.rdbuf();
        fShaderStream << fShaderFile.rdbuf();
        vertexCode = vShaderStream.str();
        fragmentCode = fShaderStream.str();

        const char* vCode = vertexCode.c_str();
        const char* fCode = fragmentCode.c_str();

        GLuint vertex = glCreateShader(GL_VERTEX_SHADER);
        glShaderSource(vertex, 1, &vCode, NULL);
        glCompileShader(vertex);

        GLuint fragment = glCreateShader(GL_FRAGMENT_SHADER);
        glShaderSource(fragment, 1, &fCode, NULL);
        glCompileShader(fragment);

        ID = glCreateProgram();
        glAttachShader(ID, vertex);
        glAttachShader(ID, fragment);
        glLinkProgram(ID);

        glDeleteShader(vertex);
        glDeleteShader(fragment);
    }

    void use() const {
        glUseProgram(ID);
    }

    void setMat4(const string& name, const glm::mat4& mat) const {
        glUniformMatrix4fv(glGetUniformLocation(ID, name.c_str()), 1, GL_FALSE, glm::value_ptr(mat));
    }
};

// 마우스 휠 콜백
void ScrollCallback(GLFWwindow* window, double xoffset, double yoffset)
{
    cameraDistance -= static_cast<float>(yoffset);
    cameraDistance = clamp(cameraDistance, 2.0f, 100.0f);
}

void MouseButtonCallback(GLFWwindow* window, int button, int action, int mods)
{
    if (button == GLFW_MOUSE_BUTTON_RIGHT)
        rightMouseDown = (action == GLFW_PRESS);

    if (button == GLFW_MOUSE_BUTTON_MIDDLE)
        middleMouseDown = (action == GLFW_PRESS);

    if (button == GLFW_MOUSE_BUTTON_LEFT)
        leftMouseDown = (action == GLFW_PRESS);

    if (action == GLFW_PRESS)
        glfwGetCursorPos(window, &lastMouseX, &lastMouseY);
}

void CursorPosCallback(GLFWwindow* window, double xpos, double ypos)
{
    float dx = static_cast<float>(xpos - lastMouseX);
    float dy = static_cast<float>(ypos - lastMouseY);
    lastMouseX = xpos;
    lastMouseY = ypos;

    if (rightMouseDown) {
        float sensitivity = 0.3f;
        yaw -= dx * sensitivity;
        pitch += dy * sensitivity;
        pitch = clamp(pitch, -89.0f, 89.0f);
    }

    if (middleMouseDown) {
        float panSpeed = 0.01f;

        // 카메라 시점 기준 좌우/상하 벡터 계산
        glm::vec3 front;
        front.x = sin(glm::radians(yaw));
        front.y = 0.0f;
        front.z = cos(glm::radians(yaw));
        front = glm::normalize(front);

        glm::vec3 cameraRight = glm::normalize(glm::cross(front, glm::vec3(0.0f, 1.0f, 0.0f)));
        glm::vec3 cameraUp = glm::vec3(0.0f, 1.0f, 0.0f);

        // 이동 적용
        cameraTarget += cameraRight * dx * panSpeed;
        cameraTarget += cameraUp * dy * panSpeed;
    }

    if (leftMouseDown) {
        float lightSensitivity = 0.3f;
        lightYaw -= dx * lightSensitivity;
        lightPitch += dy * lightSensitivity;
        lightPitch = clamp(lightPitch, -89.0f, 89.0f);

        test -= dx * lightSensitivity;
    }
}

vector<string> FindAllPMXFiles(const string& folderPath) {
    vector<string> result;

    if (!filesystem::exists(folderPath) || !filesystem::is_directory(folderPath)) {
        cerr << "폴더가 존재하지 않거나 디렉토리가 아닙니다: " << folderPath << "\n";
        return result;
    }

    for (const auto& entry : filesystem::recursive_directory_iterator(folderPath)) {
        if (entry.is_regular_file() && entry.path().extension() == ".pmx") {
            result.push_back(entry.path().string());
        }
    }

    return result;
}

vector<string> FindAllVMDFiles(const string& folderPath) {
    vector<string> result;

    if (!filesystem::exists(folderPath) || !filesystem::is_directory(folderPath)) {
        cerr << "폴더가 존재하지 않거나 디렉토리가 아닙니다: " << folderPath << "\n";
        return result;
    }

    for (const auto& entry : filesystem::recursive_directory_iterator(folderPath)) {
        if (entry.is_regular_file() && entry.path().extension() == ".vmd") {
            result.push_back(entry.path().string());
        }
    }

    return result;
}

int main()
{
    _setmode(_fileno(stdout), _O_U16TEXT);

    string modelFolder = "C:/Users/Ha Yechan/Desktop/PMXViewer/models";
    vector<string> pmxFiles = FindAllPMXFiles(modelFolder);

    if (pmxFiles.empty()) {
        cerr << "모델 폴더에 .pmx 파일이 없습니다: " << modelFolder << "\n";
        return 1;
    }

    // 목록 출력
    cerr << "[ PMX 모델 목록 ]\n";
    for (size_t i = 0; i < pmxFiles.size(); ++i) {
        cerr << i << ": " << pmxFiles[i] << "\n";
    }

    // 선택 입력
    int selected = -1;
    cerr << "\n불러올 모델 번호를 입력하세요: ";
    cin >> selected;

    if (selected < 0 || selected >= static_cast<int>(pmxFiles.size())) {
        cerr << "잘못된 선택입니다.\n";
        return 1;
    }

    string pmxPath = pmxFiles[selected];
    cerr << "선택된 PMX 파일: " << pmxPath << "\n";

    string vmdFolder = "C:/Users/Ha Yechan/Desktop/PMXViewer/motions";
    vector<string> vmdFiles = FindAllVMDFiles(vmdFolder); // 확장자 필터링 함수 개선 권장

    if (vmdFiles.empty()) {
        cerr << "모델 폴더에 .vmd 파일이 없습니다: " << modelFolder << "\n";
        return 1;
    }

    cerr << "\n[ VMD 파일 목록 ]\n";
    for (size_t i = 0; i < vmdFiles.size(); ++i) {
        cerr << i << ": " << vmdFiles[i] << "\n";
    }

    int vmdSelected = -1;
    cerr << "\n불러올 VMD 번호를 입력하세요: ";
    cin >> vmdSelected;

    if (vmdSelected < 0 || vmdSelected >= static_cast<int>(vmdFiles.size())) {
        cerr << "잘못된 선택입니다.\n";
        return 1;
    }

    unique_ptr<vmd::VmdMotion> motion;
    motion = vmd::VmdMotion::LoadFromFile(vmdFiles[vmdSelected].c_str());
    if (!motion) {
        cerr << "VMD 로딩 실패!\n";
        return 1;
    }
    // 이후 루프에서 이 `motion`을 사용

    sort(motion->bone_frames.begin(), motion->bone_frames.end(),
        [](const vmd::VmdBoneFrame& a, const vmd::VmdBoneFrame& b) {
            return a.frame < b.frame;
        });
    sort(motion->face_frames.begin(), motion->face_frames.end(),
        [](const vmd::VmdFaceFrame& a, const vmd::VmdFaceFrame& b) {
            return a.frame < b.frame;
        });

    pmx::PmxModel model;
    ifstream file(pmxPath, ios::binary);
    model.Read(&file);

    vector<glm::mat4> bindPoseMatrices(model.bone_count);
    for (int i = 0; i < model.bone_count; ++i) {
        const auto& b = model.bones[i];
        glm::vec3 t(b.position[0], b.position[1], b.position[2]);
        glm::mat4 local = glm::translate(glm::mat4(1.0f), t);

        if (b.parent_index >= 0 && b.parent_index < model.bone_count) {
            bindPoseMatrices[i] = bindPoseMatrices[b.parent_index] * local;
        }
        else {
            bindPoseMatrices[i] = local;
        }
    }
    vector<glm::mat4> inverseBindPoseMatrices(model.bone_count);
    for (int i = 0; i < model.bone_count; ++i) {
        inverseBindPoseMatrices[i] = glm::inverse(bindPoseMatrices[i]);
    }

    // GLFW 초기화
    if (!glfwInit()) {
        cerr << "GLFW 초기화 실패!\n";
        return -1;
    }

    // OpenGL 버전 3.3 Core 프로파일 사용
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

#ifdef _WIN32
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE); // macOS 호환 설정
#endif

    // 창 생성
    GLFWwindow* window = glfwCreateWindow(800, 600, "OpenGL 테스트 - PMXViewer", nullptr, nullptr);
    if (!window) {
        cerr << "윈도우 생성 실패!\n";
        glfwTerminate();
        return -1;
    }

    glfwMakeContextCurrent(window);
    glfwSetScrollCallback(window, ScrollCallback); // 🔸 마우스 휠 콜백 등록
    glfwSetMouseButtonCallback(window, MouseButtonCallback);
    glfwSetCursorPosCallback(window, CursorPosCallback);

    // GLAD 초기화
    if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress)) {
        cerr << "GLAD 초기화 실패!\n";
        return -1;
    }

    cerr << "OpenGL 버전: " << glGetString(GL_VERSION) << endl;

    string pmxFolder = filesystem::path(pmxPath).parent_path().string();
    LoadTextures(model, pmxFolder);

    Shader shader("C:/Users/Ha Yechan/Desktop/PMXViewer/shaders/vertex.glsl", "C:/Users/Ha Yechan/Desktop/PMXViewer/shaders/fragment.glsl");

    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    // VAO/VBO/EBO 초기화
    glGenVertexArrays(1, &vao);
    glGenBuffers(1, &vbo);
    glGenBuffers(1, &ebo);

    glBindVertexArray(vao);

    for (int i = 0; i < model.vertex_count; ++i) {
        const auto& src = model.vertices[i];
        GLVertex v;
        v.position = glm::vec3(src.position[0], src.position[1], src.position[2]);
        v.normal = glm::vec3(src.normal[0], src.normal[1], src.normal[2]);
        v.uv = glm::vec2(src.uv[0], src.uv[1]);

        // 본 웨이트 처리
        const auto& skin = src.skinning;
        const auto& type = src.skinning_type;

        int indices[4] = {};
        float weights[4] = {};

        switch (type) {
        case pmx::PmxVertexSkinningType::BDEF1: {
            auto* bdef = static_cast<pmx::PmxVertexSkinningBDEF1*>(skin.get());
            indices[0] = bdef->bone_index;
            weights[0] = 1.0f;
            break;
        }
        case pmx::PmxVertexSkinningType::BDEF2: {
            auto* bdef = static_cast<pmx::PmxVertexSkinningBDEF2*>(skin.get());
            indices[0] = bdef->bone_index1;
            indices[1] = bdef->bone_index2;
            weights[0] = bdef->bone_weight;
            weights[1] = 1.0f - bdef->bone_weight;
            break;
        }
        case pmx::PmxVertexSkinningType::BDEF4: {
            auto* bdef = static_cast<pmx::PmxVertexSkinningBDEF4*>(skin.get());
            indices[0] = bdef->bone_index1;
            indices[1] = bdef->bone_index2;
            indices[2] = bdef->bone_index3;
            indices[3] = bdef->bone_index4;
            weights[0] = bdef->bone_weight1;
            weights[1] = bdef->bone_weight2;
            weights[2] = bdef->bone_weight3;
            weights[3] = bdef->bone_weight4;
            break;
        }
        case pmx::PmxVertexSkinningType::SDEF: {
            auto* sdef = static_cast<pmx::PmxVertexSkinningSDEF*>(skin.get());
            indices[0] = sdef->bone_index1;
            indices[1] = sdef->bone_index2;
            weights[0] = sdef->bone_weight;
            weights[1] = 1.0f - sdef->bone_weight;
            break;
        }
        case pmx::PmxVertexSkinningType::QDEF: {
            auto* qdef = static_cast<pmx::PmxVertexSkinningQDEF*>(skin.get());
            indices[0] = qdef->bone_index1;
            indices[1] = qdef->bone_index2;
            indices[2] = qdef->bone_index3;
            indices[3] = qdef->bone_index4;
            weights[0] = qdef->bone_weight1;
            weights[1] = qdef->bone_weight2;
            weights[2] = qdef->bone_weight3;
            weights[3] = qdef->bone_weight4;
            break;
        }
        }

        for (int j = 0; j < 4; ++j) {
            v.boneIndices[j] = indices[j];
            v.boneWeights[j] = weights[j];
        }

        gVertices.push_back(v);
    }
    gIndices.assign(model.indices.get(), model.indices.get() + model.index_count);

    struct BonePose {
        glm::vec3 position;
        glm::quat rotation;
    };
    unordered_map<string, BonePose> bonePoses;
    unordered_map<string, float> morphWeights;

    vector<pmx::PmxMaterial> originalMaterials(
        model.materials.get(),
        model.materials.get() + model.material_count
    );
    vector<GLVertex> originalVertices = gVertices;

    glBindBuffer(GL_ARRAY_BUFFER, vbo);
    glBufferData(GL_ARRAY_BUFFER, gVertices.size() * sizeof(GLVertex), gVertices.data(), GL_STATIC_DRAW);

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ebo);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, gIndices.size() * sizeof(uint32_t), gIndices.data(), GL_STATIC_DRAW);

    // 속성 연결
    glEnableVertexAttribArray(0); // position
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(GLVertex), (void*)offsetof(GLVertex, position));

    glEnableVertexAttribArray(1); // normal
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(GLVertex), (void*)offsetof(GLVertex, normal));

    glEnableVertexAttribArray(2); // uv
    glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, sizeof(GLVertex), (void*)offsetof(GLVertex, uv));

    glEnableVertexAttribArray(3); // boneIndices
    glVertexAttribIPointer(3, 4, GL_INT, sizeof(GLVertex), (void*)offsetof(GLVertex, boneIndices));

    glEnableVertexAttribArray(4); // boneWeights
    glVertexAttribPointer(4, 4, GL_FLOAT, GL_FALSE, sizeof(GLVertex), (void*)offsetof(GLVertex, boneWeights));

    glBindVertexArray(0);

    // 루프
    while (!glfwWindowShouldClose(window)) {
        // 배경색 지정 및 지우기
        glClearColor(0.2f, 0.3f, 0.3f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        // 모델 스케일 적용
        glm::mat4 scale = glm::scale(glm::mat4(1.0f), glm::vec3(0.3f));

        // 카메라 위치 계산
        glm::vec3 target = cameraTarget;  // 🔄 수정: 고정값 대신 패닝 반영된 타겟 사용

        float yawRad = glm::radians(yaw);
        float pitchRad = glm::radians(pitch);
        float x = cameraDistance * cos(pitchRad) * sin(yawRad);
        float y = cameraDistance * sin(pitchRad);
        float z = cameraDistance * cos(pitchRad) * cos(yawRad);
        glm::vec3 eye = target + glm::vec3(x, y, z);

        // up 벡터 보정 (위에서 아래로 볼 때 꼬임 방지)
        glm::vec3 up = glm::vec3(0, 1, 0);
        glm::vec3 viewDir = glm::normalize(target - eye);
        if (abs(glm::dot(viewDir, up)) > 0.99f)
            up = glm::vec3(0, 0, 1);

        glm::mat4 view = glm::lookAt(eye, target, up);
        glm::mat4 projection = glm::perspective(glm::radians(45.0f), 800.f / 600.f, 0.1f, 100.f);

        // 셰이더 바인딩 및 행렬 전달
        shader.use();
        shader.setMat4("model", scale);
        shader.setMat4("view", view);
        shader.setMat4("projection", projection);

        // ✅ 라이트 방향 계산
        float lyawRad = glm::radians(lightYaw);
        float lpitchRad = glm::radians(lightPitch);
        glm::vec3 lightDir = glm::normalize(glm::vec3(
            cos(lpitchRad) * sin(lyawRad),
            sin(lpitchRad),
            cos(lpitchRad) * cos(lyawRad)
        ));
        glUniform3fv(glGetUniformLocation(shader.ID, "lightDir"), 1, glm::value_ptr(lightDir));

        // ✅ 카메라 위치 전달
        glUniform3fv(glGetUniformLocation(shader.ID, "cameraPos"), 1, glm::value_ptr(eye));

        // 🔽 sampler2D와 텍스처 유닛 연결
        glUniform1i(glGetUniformLocation(shader.ID, "tex"), 0);
        glUniform1i(glGetUniformLocation(shader.ID, "toonTex"), 1);
        glUniform1i(glGetUniformLocation(shader.ID, "sphereTex"), 2);

        // 이전 프레임 상태 초기화
        copy(originalMaterials.begin(), originalMaterials.end(), model.materials.get());
        gVertices = originalVertices;

        // 현재 시간 기반 프레임 계산
        float time = static_cast<float>(glfwGetTime());
        int currentFrame = static_cast<int>(time * 30.0f); // 30fps

        // 본 프레임 적용
        std::vector<glm::mat4> localMatrices(model.bone_count);
        for (int i = 0; i < model.bone_count; ++i) {
            const auto& bone = model.bones[i];

            glm::vec3 t(bone.position[0], bone.position[1], bone.position[2]);
            glm::quat r = glm::quat(1, 0, 0, 0);

            if (i == 11) {
                r = glm::angleAxis(glm::radians(test), glm::vec3(0, 0, 1));
            }

            localMatrices[i] = glm::translate(glm::mat4(1.0f), t) * glm::toMat4(r);
        }
        std::vector<glm::mat4> boneMatrices(model.bone_count);
        for (int i = 0; i < model.bone_count; ++i) {
            int parent = model.bones[i].parent_index;
            if (parent >= 0) {
                boneMatrices[i] = boneMatrices[parent] * localMatrices[i];
            }
            else {
                boneMatrices[i] = localMatrices[i];
            }
        }
        std::vector<glm::mat4> skinMatrices(model.bone_count);
        for (int i = 0; i < model.bone_count; ++i) {
            skinMatrices[i] = boneMatrices[i] * inverseBindPoseMatrices[i];
        }
        GLint loc = glGetUniformLocation(shader.ID, "skinMatrices");
        glUniformMatrix4fv(loc, 512, GL_FALSE, glm::value_ptr(skinMatrices[0]));

        // 모프 프레임 적용
        for (const auto& face : motion->face_frames) {
            if (face.frame > currentFrame) continue;
            std::string faceName;
            oguna::EncodingConverter converter;
            converter.Cp932ToUtf8(face.face_name.c_str(), static_cast<int>(face.face_name.length()), &faceName);
            morphWeights[faceName] = face.weight;
        }
        for (const auto& [name, weight] : morphWeights) {
            int morphIndex = FindMorphIndexByName(model, name);
            if (morphIndex >= 0) {
                ApplyMorph(model, gVertices, skinMatrices, morphIndex, weight);
            }
        }
        glBindBuffer(GL_ARRAY_BUFFER, vbo);
        glBufferData(GL_ARRAY_BUFFER, gVertices.size() * sizeof(GLVertex), gVertices.data(), GL_STATIC_DRAW);

        // 🔽 모델 그리기 (VAO 바인딩 및 그리기)
        glBindVertexArray(vao);
        int indexOffset = 0;
        for (int i = 0; i < model.material_count; ++i) {
            const auto& material = model.materials[i];
            int indexCount = material.index_count;

            // 📌 텍스처 인덱스
            int texIndex = material.diffuse_texture_index;
            int toonIndex = material.toon_texture_index;
            int sphereIndex = material.sphere_texture_index;
            int sphereMode = material.sphere_op_mode;

            // 📌 사용 여부 확인
            bool bUseToon = (toonIndex >= 0 && toonIndex < gTextures.size());
            bool bUseSphere = (sphereIndex >= 0 && sphereIndex < gTextures.size());

            // 📌 텍스처 바인딩
            glActiveTexture(GL_TEXTURE0); // Diffuse
            glBindTexture(GL_TEXTURE_2D, (texIndex >= 0 && texIndex < gTextures.size()) ? gTextures[texIndex] : 0);

            glActiveTexture(GL_TEXTURE1); // Toon
            glBindTexture(GL_TEXTURE_2D, bUseToon ? gTextures[toonIndex] : 0);

            glActiveTexture(GL_TEXTURE2); // Sphere
            glBindTexture(GL_TEXTURE_2D, bUseSphere ? gTextures[sphereIndex] : 0);

            // 📌 셰이더 uniform 설정
            // 🟡 sphereMode 전달 (0: 무효, 1: 승산, 2: 가산)
            glUniform1i(glGetUniformLocation(shader.ID, "bUseToon"), bUseToon);
            glUniform1i(glGetUniformLocation(shader.ID, "bUseSphere"), bUseSphere);
            glUniform1i(glGetUniformLocation(shader.ID, "sphereMode"), bUseSphere ? sphereMode : 0);

            // 머티리얼 속성 전달
            glUniform4fv(glGetUniformLocation(shader.ID, "diffuse"), 1, material.diffuse);
            glUniform3fv(glGetUniformLocation(shader.ID, "specular"), 1, material.specular);
            glUniform1f(glGetUniformLocation(shader.ID, "specularPower"), material.specularlity);
            glUniform3fv(glGetUniformLocation(shader.ID, "ambient"), 1, material.ambient);
            glUniform1f(glGetUniformLocation(shader.ID, "Alpha"), material.diffuse[3]); // diffuse.a

            // 드로우
            glDrawElements(GL_TRIANGLES, indexCount, GL_UNSIGNED_INT, (void*)(indexOffset * sizeof(uint32_t)));
            indexOffset += indexCount;
        }

        // 화면 표시 및 이벤트 처리
        glfwSwapBuffers(window);
        glfwPollEvents();
    }

    // 정리
    glfwDestroyWindow(window);
    glfwTerminate();
    return 0;
}
