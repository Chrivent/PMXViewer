#define GLM_ENABLE_EXPERIMENTAL

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
#include "IKSolver.h"
#include "PMXActor.h"

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtx/quaternion.hpp>
#include <glm/gtx/string_cast.hpp>

#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"

#define STB_IMAGE_RESIZE_IMPLEMENTATION
#include "stb_image_resize2.h"

using namespace std;

GLuint vao, vbo, ebo;

float cameraDistance = 10.0f;
float yaw = 0.0f;
float pitch = 0.0f;
bool rightMouseDown = false;
bool middleMouseDown = false;
double lastMouseX = 0.0, lastMouseY = 0.0;

glm::vec3 cameraTarget = glm::vec3(0, 5, 0);

float lightYaw = 45.0f;   // 초기 조명 방향 (degree)
float lightPitch = -45.0f;
glm::vec3 lightDir = glm::vec3(1, -1, -1);

bool leftMouseDown = false;

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

    lightDir = glm::vec3(
        cos(glm::radians(lightPitch)) * cos(glm::radians(lightYaw)),
        sin(glm::radians(lightPitch)),
        cos(glm::radians(lightPitch)) * sin(glm::radians(lightYaw))
    );
    lightDir = glm::normalize(lightDir);
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

    pmx::PmxModel model;
    ifstream file(pmxPath, ios::binary);
    model.Read(&file);

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

    sort(motion->bone_frames.begin(), motion->bone_frames.end(),
        [](const vmd::VmdBoneFrame& a, const vmd::VmdBoneFrame& b) {
            return a.frame < b.frame;
        });
    sort(motion->face_frames.begin(), motion->face_frames.end(),
        [](const vmd::VmdFaceFrame& a, const vmd::VmdFaceFrame& b) {
            return a.frame < b.frame;
        });
    // GLFW 초기화
    if (!glfwInit()) {
        wcerr << L"GLFW 초기화 실패!\n";
        return -1;
    }

    // OpenGL 버전 3.3 Core 프로파일 사용
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 5);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

#ifdef _WIN32
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE); // macOS 호환 설정
#endif
    // 창 생성
    GLFWwindow* window =
        glfwCreateWindow(800, 600, "PMX Viewer (OpenGL)", nullptr, nullptr);
    if (!window) { wcerr << L"윈도우 생성 실패!\n"; return -1; }
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

    /* ---- ③ PMXActor 생성·초기화 ---- */
    PMXActor actor;
    if (!actor.Initialize(model, fs::path(pmxPath), motion.get())) {
        wcerr << L"PMXActor 초기화 실패!\n"; return -1;
    }

    /* ---- ④ 메인 루프 ---- */
    double last = glfwGetTime();
    while (!glfwWindowShouldClose(window)) {
        /* dt 계산 */
        double now = glfwGetTime();
        float dt = static_cast<float>(now - last);
        last = now;

        /* 카메라 행렬 계산 (기존 코드 재사용) */
        glm::vec3 target = cameraTarget;
        float yawRad = glm::radians(yaw);
        float pitchRad = glm::radians(pitch);
        glm::vec3 eye(
            cameraDistance* cos(pitchRad)* sin(yawRad),
            cameraDistance* sin(pitchRad),
            cameraDistance* cos(pitchRad)* cos(yawRad));
        eye += target;

        glm::vec3 up(0, 1, 0);
        glm::vec3 viewDir = glm::normalize(target - eye);
        if (abs(glm::dot(viewDir, up)) > 0.99f) up = glm::vec3(0, 0, 1);

        glm::mat4 view = glm::lookAt(eye, target, up);
        glm::mat4 proj = glm::perspective(
            glm::radians(45.0f), 800.f / 600.f, 0.1f, 100.f);

        /* 업데이트 + 렌더 */
        actor.Update(dt);

        glClearColor(0.2f, 0.3f, 0.3f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        actor.Draw(view, proj, eye, lightDir);

        glfwSwapBuffers(window);
        glfwPollEvents();
    }

    /* ---- ⑤ 종료 ---- */
    actor.Destroy();
    glfwDestroyWindow(window);
    glfwTerminate();
    return 0;
}
