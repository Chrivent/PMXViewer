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
#include <glm/gtx/string_cast.hpp>
#include <glm/gtx/euler_angles.hpp>

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

float lightYaw = 45.0f;   // 초기 조명 방향 (degree)
float lightPitch = -45.0f;
bool leftMouseDown = false;

void LoadTextures(const pmx::PmxModel& model, const string& pmxBaseDir) {
    gTextures.resize(model.texture_count, 0);

    for (int i = 0; i < model.texture_count; ++i) {
        string relPath(model.textures[i].begin(), model.textures[i].end()); // wstring → string
        filesystem::path texPath = filesystem::path(pmxBaseDir) / relPath;

        int w, h, channels;
        stbi_uc* data = stbi_load(texPath.string().c_str(), &w, &h, &channels, STBI_rgb_alpha);
        if (!data) {
            wcerr << "텍스처 로딩 실패: " << texPath << "\n";
            continue;
        }

        GLuint texID;
        glGenTextures(1, &texID);
        glBindTexture(GL_TEXTURE_2D, texID);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, w, h, 0, GL_RGBA, GL_UNSIGNED_BYTE, data);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

        gTextures[i] = texID;
        stbi_image_free(data);
    }
}

void ApplyMorph(const pmx::PmxModel& model, vector<GLVertex>& vertices, vector<glm::mat4>& localMatrices, int morphIndex, float weight) {
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
            if (bi < 0 || bi >= static_cast<int>(localMatrices.size())) continue;

            const auto& bone = model.bones[bi];
            glm::vec3 pivot(bone.position[0], bone.position[1], bone.position[2]);

            glm::vec3 t(offset.translation[0], offset.translation[1], offset.translation[2]);
            glm::quat r(offset.rotation[3], offset.rotation[0], offset.rotation[1], offset.rotation[2]); // wxyz

            // 보간된 트랜스폼
            glm::mat4 trans = glm::translate(glm::mat4(1.0f), t * weight);
            glm::mat4 rot = glm::toMat4(glm::slerp(glm::quat(), r, weight));

            // 피벗 기준으로 회전·이동 적용: Tpivot * (T * R) * Tinv
            glm::mat4 morphMat = glm::translate(glm::mat4(1.0f), pivot)
                * trans * rot
                * glm::translate(glm::mat4(1.0f), -pivot);

            // 기존 local 행렬에 적용
            localMatrices[bi] = morphMat * localMatrices[bi];
        }
        break;

    case pmx::MorphType::Group:
        for (int i = 0; i < morph.offset_count; ++i) {
            const auto& group = morph.group_offsets[i];
            ApplyMorph(model, vertices, localMatrices, group.morph_index, group.morph_weight * weight);
        }
        break;

    default:
        break;
    }
}

float BezierInterpolate(float x, glm::vec2 a, glm::vec2 b, int iter = 10, float epsilon = 1e-5f) {
    if (a.x == a.y && b.x == b.y)
        return x;

    float t = x;
    float k0 = 1 + 3 * a.x - 3 * b.x;
    float k1 = 3 * b.x - 6 * a.x;
    float k2 = 3 * a.x;

    for (int i = 0; i < iter; ++i) {
        float ft = k0 * t * t * t + k1 * t * t + k2 * t - x;
        if (fabs(ft) < epsilon) break;
        t -= ft / 2.0f;  // 근사 방식
    }

    float r = 1 - t;
    return t * t * t + 3 * t * t * r * b.y + 3 * t * r * r * a.y;
}

struct BonePose {
    glm::vec3 position;
    glm::quat orientation;
};

void Decompose(const glm::mat4& mat, glm::vec3& outPos, glm::vec3& outEuler) {
    outPos = glm::vec3(mat[3]);

    glm::quat rot = glm::quat_cast(mat);
    outEuler = glm::eulerAngles(rot);

    std::wcerr << L"[Decompose] position: (" << outPos.x << L", " << outPos.y << L", " << outPos.z << L")"
        << L" | euler: (" << outEuler.x << L", " << outEuler.y << L", " << outEuler.z << L")" << std::endl;
}

glm::mat4 ComposeTransform(const glm::vec3& pos, const glm::vec3& euler) {
    std::wcerr << L"[Compose] position: (" << pos.x << L", " << pos.y << L", " << pos.z << L")"
        << L" | euler: (" << euler.x << L", " << euler.y << L", " << euler.z << L")" << std::endl;

    glm::mat4 T = glm::translate(glm::mat4(1.0f), pos);
    glm::mat4 R = glm::yawPitchRoll(euler.y, euler.x, euler.z);

    return T * R;
}

class IKSolver {
private:
    std::wstring _ikName;
    int _effectorIndex;
    int _targetIndex;
    int _loopCount;
    float _angleLimit;

    struct Chain {
        int boneIndex;
        bool hasLimit;
        glm::vec3 limitMin;
        glm::vec3 limitMax;
    };
    std::vector<Chain> _chains;

public:
    IKSolver(const std::wstring& ikName, int effectorIdx, int targetIdx, int loopCount, float angleLimit) {
        _ikName = ikName;
        _effectorIndex = effectorIdx;
        _targetIndex = targetIdx;
        _loopCount = loopCount;
        _angleLimit = angleLimit;
    }

    const std::wstring& GetName() const {
        return _ikName;
    }

    void AddIKChain(int boneIndex, bool hasLimit, glm::vec3 limitMin, glm::vec3 limitMax) {
        _chains.push_back({ boneIndex, hasLimit, limitMin, limitMax });
    }

    void Solve(std::vector<glm::mat4>& localMatrices) {
        std::wcerr << L"[IKSolver] Solving IK: " << _ikName
            << L" | effector: " << _effectorIndex
            << L" -> target: " << _targetIndex
            << L" | loop: " << _loopCount
            << L" | chains: " << _chains.size() << std::endl;

        for (size_t i = 0; i < _chains.size(); ++i) {
            const auto& chain = _chains[i];
            std::wcerr << L"  └─ Chain[" << i << L"] boneIndex: " << chain.boneIndex
                << L", hasLimit: " << (chain.hasLimit ? L"true" : L"false")
                << L", limitMin: (" << chain.limitMin.x << L", " << chain.limitMin.y << L", " << chain.limitMin.z << L")"
                << L", limitMax: (" << chain.limitMax.x << L", " << chain.limitMax.y << L", " << chain.limitMax.z << L")"
                << std::endl;
        }

        if (_effectorIndex < 0 || _targetIndex < 0 || _chains.empty())
            return;

        std::vector<glm::mat4> worldMatrices(localMatrices.size());

        auto UpdateWorldMatrices = [&]() {
            for (size_t i = 0; i < localMatrices.size(); ++i) {
                int parent = (i < _chains.size()) ? _chains[i].boneIndex : -1;
                if (parent >= 0 && parent < (int)localMatrices.size())
                    worldMatrices[i] = worldMatrices[parent] * localMatrices[i];
                else
                    worldMatrices[i] = localMatrices[i];
            }
            };

        float lastDistance = std::numeric_limits<float>::max();

        for (int iter = 0; iter < _loopCount; ++iter) {
            UpdateWorldMatrices();

            SolveCore(iter, localMatrices, worldMatrices);

            glm::vec3 effectorPos = glm::vec3(worldMatrices[_effectorIndex][3]);
            glm::vec3 targetPos = glm::vec3(worldMatrices[_targetIndex][3]);
            float dist = glm::length(targetPos - effectorPos);

            if (dist < lastDistance) {
                lastDistance = dist;
            }
            else {
                break;
            }
        }
    }

    enum class SolveAxis
    {
        X,
        Y,
        Z
    };

    void SolveCore(int iteration, std::vector<glm::mat4>& local, const std::vector<glm::mat4>& world) {
        std::wcerr << L"    [SolveCore] iteration: " << iteration << std::endl;

        for (size_t i = 0; i < _chains.size(); ++i) {
            const auto& chain = _chains[i];
            std::wcerr << L"      └─ Chain[" << i << L"] boneIndex: " << chain.boneIndex
                << L" | hasLimit: " << (chain.hasLimit ? L"true" : L"false") << std::endl;

            int boneIndex = chain.boneIndex;

            if (boneIndex < 0 || boneIndex >= (int)local.size())
                continue;

            glm::mat4 invChainWorld = glm::inverse(world[boneIndex]);
            glm::vec3 effectorPos = glm::vec3(invChainWorld * world[_effectorIndex][3]);
            glm::vec3 targetPos = glm::vec3(invChainWorld * world[_targetIndex][3]);

            glm::vec3 axis = glm::cross(effectorPos, targetPos);
            float dot = glm::dot(glm::normalize(effectorPos), glm::normalize(targetPos));
            dot = glm::clamp(dot, -1.0f, 1.0f);
            float angle = std::acos(dot);

            if (glm::length(axis) < 1e-4 || angle < 1e-4f) {
                std::wcerr << L"        > Skipping due to small axis/angle" << std::endl;
                continue;
            }

            angle = glm::min(angle, _angleLimit);

            glm::quat deltaRot = glm::angleAxis(angle, glm::normalize(axis));
            glm::mat4 R = glm::toMat4(deltaRot);

            local[boneIndex] = local[boneIndex] * R;

            std::wcerr << L"        > effectorPos: (" << effectorPos.x << L", " << effectorPos.y << L", " << effectorPos.z << L")"
                << L" | targetPos: (" << targetPos.x << L", " << targetPos.y << L", " << targetPos.z << L")"
                << L" | angle: " << angle << L" | axisLen: " << glm::length(axis) << std::endl;

            if (chain.hasLimit) {
                bool limitX = chain.limitMin.x != chain.limitMax.x;
                bool limitY = chain.limitMin.y != chain.limitMax.y;
                bool limitZ = chain.limitMin.z != chain.limitMax.z;

                int axisCount = (limitX ? 1 : 0) + (limitY ? 1 : 0) + (limitZ ? 1 : 0);

                if (axisCount == 1) {
                    if (limitX) {
                        SolvePlane(iteration, i, local, world, SolveAxis::X);
                        continue;
                    }
                    if (limitY) {
                        SolvePlane(iteration, i, local, world, SolveAxis::Y);
                        continue;
                    }
                    if (limitZ) {
                        SolvePlane(iteration, i, local, world, SolveAxis::Z);
                        continue;
                    }
                }
            }
        }
    }

    void SolvePlane(int iteration,
        int chainIndex,
        std::vector<glm::mat4>& local,
        const std::vector<glm::mat4>& world,
        SolveAxis axis)
    {
        std::wcerr << L"[SolvePlane] iteration: " << iteration
            << L" | chainIndex: " << chainIndex
            << L" | axis: ";
        switch (axis) {
        case SolveAxis::X: std::wcerr << L"X"; break;
        case SolveAxis::Y: std::wcerr << L"Y"; break;
        case SolveAxis::Z: std::wcerr << L"Z"; break;
        default:           std::wcerr << L"(unknown)"; break;
        }
        std::wcerr << std::endl;

        if (chainIndex < 0 || chainIndex >= (int)_chains.size())
            return;

        int boneIndex = _chains[chainIndex].boneIndex;
        if (boneIndex < 0 || boneIndex >= (int)local.size())
            return;

        glm::mat4 invWorld = glm::inverse(world[boneIndex]);
        glm::vec3 effector = glm::vec3(invWorld * world[_effectorIndex][3]);
        glm::vec3 target = glm::vec3(invWorld * world[_targetIndex][3]);

        if (axis == SolveAxis::X) {
            effector.x = 0;
            target.x = 0;
        }
        else if (axis == SolveAxis::Y) {
            effector.y = 0;
            target.y = 0;
        }
        else if (axis == SolveAxis::Z) {
            effector.z = 0;
            target.z = 0;
        }

        if (glm::length(effector) < 1e-5f || glm::length(target) < 1e-5f)
            return;

        effector = glm::normalize(effector);
        target = glm::normalize(target);

        float cosAngle = glm::clamp(glm::dot(effector, target), -1.0f, 1.0f);
        float angle = std::acos(cosAngle);

        float sign = glm::sign(effector.x * target.y - effector.y * target.x); // Z축 밖의 경우
        angle *= sign;

        angle = glm::clamp(angle, -_angleLimit, _angleLimit);

        glm::vec3 pos, euler;
        Decompose(local[boneIndex], pos, euler);

        if (axis == SolveAxis::X) euler.x += angle;
        if (axis == SolveAxis::Y) euler.y += angle;
        if (axis == SolveAxis::Z) euler.z += angle;

        local[boneIndex] = ComposeTransform(pos, euler);
    }

    bool IsEnabledAtFrame(int frameNo, const std::vector<const vmd::VmdIkFrame*>& keyframes) const {
        const vmd::VmdIkFrame* lastFrame = nullptr;
        for (const auto* f : keyframes) {
            if (f->frame > frameNo) break;
            for (const auto& ik : f->ik_enable) {
                std::wstring name;
                oguna::EncodingConverter{}.Cp932ToUtf16(ik.ik_name.c_str(), (int)ik.ik_name.length(), &name);
                if (name == _ikName) {
                    lastFrame = f;
                    break;
                }
            }
        }

        if (!lastFrame) return true;

        for (const auto& ik : lastFrame->ik_enable) {
            std::wstring name;
            oguna::EncodingConverter{}.Cp932ToUtf16(ik.ik_name.c_str(), (int)ik.ik_name.length(), &name);
            if (name == _ikName)
                return ik.enable;
        }

        return true;
    }
};

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
    }
}

vector<wstring> FindAllPMXFiles(const wstring& folderPath) {
    vector<wstring> result;

    if (!filesystem::exists(folderPath) || !filesystem::is_directory(folderPath)) {
        wcerr << "폴더가 존재하지 않거나 디렉토리가 아닙니다: " << folderPath << "\n";
        return result;
    }

    for (const auto& entry : filesystem::recursive_directory_iterator(folderPath)) {
        if (entry.is_regular_file() && entry.path().extension() == ".pmx") {
            result.push_back(entry.path().wstring());
        }
    }

    return result;
}

vector<wstring> FindAllVMDFiles(const wstring& folderPath) {
    vector<wstring> result;

    if (!filesystem::exists(folderPath) || !filesystem::is_directory(folderPath)) {
        wcerr << L"폴더가 존재하지 않거나 디렉토리가 아닙니다: " << folderPath << "\n";
        return result;
    }

    for (const auto& entry : filesystem::recursive_directory_iterator(folderPath)) {
        if (entry.is_regular_file() && entry.path().extension() == ".vmd") {
            result.push_back(entry.path().wstring());
        }
    }

    return result;
}

int main()
{
    _setmode(_fileno(stderr), _O_U16TEXT);

    wstring modelFolder = L"C:/Users/Ha Yechan/Desktop/PMXViewer/models";
    vector<wstring> pmxFiles = FindAllPMXFiles(modelFolder);

    if (pmxFiles.empty()) {
        wcerr << L"모델 폴더에 .pmx 파일이 없습니다: " << modelFolder << "\n";
        return 1;
    }

    // 목록 출력
    wcerr << L"[ PMX 모델 목록 ]\n";
    for (size_t i = 0; i < pmxFiles.size(); ++i) {
        wcerr << i << ": " << pmxFiles[i] << "\n";
    }

    // 선택 입력
    int selected = -1;
    wcerr << L"\n불러올 모델 번호를 입력하세요: ";
    cin >> selected;

    if (selected < 0 || selected >= static_cast<int>(pmxFiles.size())) {
        wcerr << L"잘못된 선택입니다.\n";
        return 1;
    }

    wstring pmxPath = pmxFiles[selected];
    wcerr << L"선택된 PMX 파일: " << pmxPath << "\n";

    wstring vmdFolder = L"C:/Users/Ha Yechan/Desktop/PMXViewer/motions";
    vector<wstring> vmdFiles = FindAllVMDFiles(vmdFolder); // 확장자 필터링 함수 개선 권장

    if (vmdFiles.empty()) {
        wcerr << L"모델 폴더에 .vmd 파일이 없습니다: " << modelFolder << "\n";
        return 1;
    }

    wcerr << L"\n[ VMD 파일 목록 ]\n";
    for (size_t i = 0; i < vmdFiles.size(); ++i) {
        wcerr << i << L": " << vmdFiles[i] << "\n";
    }

    int vmdSelected = -1;
    wcerr << L"\n불러올 VMD 번호를 입력하세요: ";
    cin >> vmdSelected;

    if (vmdSelected < 0 || vmdSelected >= static_cast<int>(vmdFiles.size())) {
        wcerr << L"잘못된 선택입니다.\n";
        return 1;
    }

    unique_ptr<vmd::VmdMotion> motion;
    motion = vmd::VmdMotion::LoadFromFile(vmdFiles[vmdSelected].c_str());
    if (!motion) {
        wcerr << L"VMD 로딩 실패!\n";
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
    sort(motion->ik_frames.begin(), motion->ik_frames.end(),
        [](const vmd::VmdIkFrame& a, const vmd::VmdIkFrame& b) {
            return a.frame < b.frame;
        });

    pmx::PmxModel model;
    ifstream file(pmxPath, ios::binary);
    model.Read(&file);

    // GLFW 초기화
    if (!glfwInit()) {
        wcerr << L"GLFW 초기화 실패!\n";
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
        wcerr << L"윈도우 생성 실패!\n";
        glfwTerminate();
        return -1;
    }

    glfwMakeContextCurrent(window);
    glfwSetScrollCallback(window, ScrollCallback); // 🔸 마우스 휠 콜백 등록
    glfwSetMouseButtonCallback(window, MouseButtonCallback);
    glfwSetCursorPosCallback(window, CursorPosCallback);

    // GLAD 초기화
    if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress)) {
        wcerr << L"GLAD 초기화 실패!\n";
        return -1;
    }

    wcerr << L"OpenGL 버전: " << glGetString(GL_VERSION) << endl;

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

    unordered_map<wstring, BonePose> bonePoses;
    unordered_map<wstring, float> morphWeights;

    vector<pmx::PmxMaterial> originalMaterials(
        model.materials.get(),
        model.materials.get() + model.material_count
    );
    vector<GLVertex> originalVertices = gVertices;

    unordered_map<wstring, vector<const vmd::VmdBoneFrame*>> boneKeyframes;
    unordered_map<wstring, vector<const vmd::VmdFaceFrame*>> faceKeyframes;
    unordered_map<wstring, vector<const vmd::VmdIkFrame*>> ikKeyframes;

    for (const auto& f : motion->bone_frames) {
        wstring name;
        oguna::EncodingConverter{}.Cp932ToUtf16(f.name.c_str(), (int)f.name.length(), &name);
        boneKeyframes[name].push_back(&f);
    }
    for (const auto& f : motion->face_frames) {
        wstring name;
        oguna::EncodingConverter{}.Cp932ToUtf16(f.face_name.c_str(), (int)f.face_name.length(), &name);
        faceKeyframes[name].push_back(&f);
    }
    for (const auto& f : motion->ik_frames) {
        for (const auto& ik : f.ik_enable) {
            wstring name;
            oguna::EncodingConverter{}.Cp932ToUtf16(ik.ik_name.c_str(), (int)ik.ik_name.length(), &name);
            ikKeyframes[name].push_back(&f);
        }
    }

    vector<glm::mat4> globalMatrices(model.bone_count);
    vector<glm::mat4> localMatrices(model.bone_count);

    unordered_map<wstring, int> morphNameToIndex;
    for (int i = 0; i < model.morph_count; ++i)
        morphNameToIndex[model.morphs[i].morph_name] = i;

    std::vector<std::unique_ptr<IKSolver>> ikSolvers;
    for (int i = 0; i < model.bone_count; ++i) {
        const auto& bone = model.bones[i];

        if ((bone.bone_flag & 0x0020) != 0) {
            auto solver = std::make_unique<IKSolver>(
                bone.bone_name,
                i,
                bone.ik_target_bone_index,
                bone.ik_loop,
                bone.ik_loop_angle_limit
            );

            for (int j = 0; j < bone.ik_link_count; ++j) {
                const auto& link = bone.ik_links[j];

                solver->AddIKChain(
                    link.link_target,
                    link.angle_lock != 0,  // uint8_t → bool
                    glm::make_vec3(link.min_radian),
                    glm::make_vec3(link.max_radian)
                );
            }

            ikSolvers.push_back(std::move(solver));
        }
    }

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

        // 현재 시간 기반 프레임 계산 (애니메이션은 30fps 기준 시간축 사용)
        float time = static_cast<float>(glfwGetTime());
        float frameTime = time * 30.0f;                    // ⬅️ 실수형 시간
        int currentFrame = static_cast<int>(frameTime);    // ⬅️ 정수 프레임 (예: 0~N)
        float fraction = frameTime - currentFrame;         // ⬅️ 보간 잔여

        // 이전 프레임 상태 초기화
        copy(originalMaterials.begin(), originalMaterials.end(), model.materials.get());
        gVertices = originalVertices;

        // 본 프레임 적용
        bonePoses.clear();
        for (const auto& [name, frames] : boneKeyframes) {
            if (frames.empty()) continue;
            int prev = -1, next = -1;
            for (int i = 0; i < frames.size(); ++i) {
                if (frames[i]->frame > currentFrame) {
                    next = i;
                    break;
                }
                prev = i;
            }

            if (prev < 0 || next < 0) continue;
            const auto* f1 = frames[prev];
            const auto* f2 = frames[next];

            float rawT = (frameTime - f1->frame) / float(f2->frame - f1->frame);

            const uint8_t* interp = reinterpret_cast<const uint8_t*>(&f2->interpolation);

            glm::vec2 p1x(interp[0], interp[1]);
            glm::vec2 p2x(interp[8], interp[9]);
            float tx = BezierInterpolate(rawT, p1x / 127.0f, p2x / 127.0f, 12);

            glm::vec2 p1y(interp[16], interp[17]);
            glm::vec2 p2y(interp[24], interp[25]);
            float ty = BezierInterpolate(rawT, p1y / 127.0f, p2y / 127.0f, 12);

            glm::vec2 p1z(interp[32], interp[33]);
            glm::vec2 p2z(interp[40], interp[41]);
            float tz = BezierInterpolate(rawT, p1z / 127.0f, p2z / 127.0f, 12);

            glm::vec2 p1r(interp[48], interp[49]);
            glm::vec2 p2r(interp[56], interp[57]);
            float tr = BezierInterpolate(rawT, p1r / 127.0f, p2r / 127.0f, 12);

            // 위치 보간
            glm::vec3 p1(f1->position[0], f1->position[1], f1->position[2]);
            glm::vec3 p2(f2->position[0], f2->position[1], f2->position[2]);
            glm::vec3 pos = glm::vec3(
                glm::mix(p1.x, p2.x, glm::clamp(tx, 0.0f, 1.0f)),
                glm::mix(p1.y, p2.y, glm::clamp(ty, 0.0f, 1.0f)),
                glm::mix(p1.z, p2.z, glm::clamp(tz, 0.0f, 1.0f))
            );

            // 회전 보간
            glm::quat q1(f1->orientation[3], f1->orientation[0], f1->orientation[1], f1->orientation[2]);
            glm::quat q2(f2->orientation[3], f2->orientation[0], f2->orientation[1], f2->orientation[2]);
            glm::quat rot = glm::slerp(q1, q2, glm::clamp(tr, 0.0f, 1.0f));

            BonePose pose;
            pose.position = pos;
            pose.orientation = rot;

            bonePoses[name] = pose;
        }

        for (int i = 0; i < model.bone_count; ++i) {
            const auto& bone = model.bones[i];
            glm::vec3 pivot(bone.position[0], bone.position[1], bone.position[2]);
            glm::mat4 Trest = glm::translate(glm::mat4(1.0f), pivot);

            auto it = bonePoses.find(bone.bone_name);
            if (it != bonePoses.end()) {
                glm::mat4 Tanim = glm::translate(glm::mat4(1.0f), it->second.position);
                glm::mat4 Ranim = glm::toMat4(it->second.orientation);
                localMatrices[i] = Trest * Tanim * Ranim * glm::inverse(Trest);
            }
            else {
                localMatrices[i] = glm::mat4(1.0f);
            }
        }

        for (int i = 0; i < model.bone_count; ++i) {
            const auto& bone = model.bones[i];

            if ((bone.bone_flag & 0x0100) && (bone.bone_flag & (0x0200 | 0x0400))) {
                int parentIdx = bone.grant_parent_index;
                float weight = bone.grant_weight;

                glm::vec3 pos, euler;
                Decompose(localMatrices[i], pos, euler);

                if (bone.bone_flag & 0x0200) {
                    glm::quat parentRot = glm::quat_cast(localMatrices[parentIdx]);
                    glm::quat selfRot = glm::quat_cast(localMatrices[i]);
                    glm::quat mixed = glm::slerp(selfRot, parentRot * selfRot, weight);
                    glm::mat4 rot = glm::toMat4(mixed);

                    localMatrices[i] = glm::translate(glm::mat4(1.0f), pos) * rot;
                }

                if (bone.bone_flag & 0x0400) {
                    glm::vec3 appendPos = glm::vec3(localMatrices[parentIdx][3]);
                    pos += appendPos * weight;

                    glm::quat rot = glm::quat_cast(localMatrices[i]);
                    localMatrices[i] = glm::translate(glm::mat4(1.0f), pos) * glm::toMat4(rot);
                }
            }
        }

        for (auto& solver : ikSolvers) {
            const auto& keyframes = ikKeyframes[solver->GetName()];
            if (solver->IsEnabledAtFrame(currentFrame, keyframes)) {
                solver->Solve(localMatrices);
            }
        }

        // 모프 프레임 적용
        morphWeights.clear();
        for (const auto& [name, frames] : faceKeyframes) {
            if (frames.empty()) continue;
            int idx = -1;
            for (int i = 0; i < frames.size(); ++i) {
                if (frames[i]->frame > currentFrame) break;
                idx = i;
            }
            if (idx >= 0) {
                morphWeights[name] = frames[idx]->weight;
            }
        }
        for (const auto& [name, weight] : morphWeights) {
            auto it = morphNameToIndex.find(name);
            if (it != morphNameToIndex.end()) {
                ApplyMorph(model, gVertices, localMatrices, it->second, weight);
            }
        }

        for (int i = 0; i < model.bone_count; ++i) {
            int parent = model.bones[i].parent_index;
            if (parent >= 0)
                globalMatrices[i] = globalMatrices[parent] * localMatrices[i];
            else
                globalMatrices[i] = localMatrices[i];
        }

        GLint loc = glGetUniformLocation(shader.ID, "boneMatrices");
        glUniformMatrix4fv(loc, 512, GL_FALSE, glm::value_ptr(globalMatrices[0]));

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
