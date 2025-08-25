#ifdef _WIN32
#include <io.h>
#include <fcntl.h>
#endif

#include <iostream>
#include <filesystem>

#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <glm/gtc/type_ptr.hpp>

#include "Pmx.h"
#include "Vmd.h"
#include "PMXActor.h" 

// ====== (카메라/입력 상태 전역은 기존 그대로 유지) ======
float cameraDistance = 10.0f;
float yaw = 0.0f;
float pitch = 0.0f;
bool rightMouseDown = false;
bool middleMouseDown = false;
bool leftMouseDown = false;
double lastMouseX = 0.0, lastMouseY = 0.0;

glm::vec3 cameraTarget = glm::vec3(0, 5, 0);
float lightYaw = 45.0f;   // 초기 조명 방향 (degree)
float lightPitch = -45.0f;

// ====== 입력 콜백들 (네 코드 그대로) ======
void ScrollCallback(GLFWwindow* window, double xoffset, double yoffset)
{
    cameraDistance -= static_cast<float>(yoffset);
    cameraDistance = std::clamp(cameraDistance, 2.0f, 100.0f);
}
void MouseButtonCallback(GLFWwindow* window, int button, int action, int mods)
{
    if (button == GLFW_MOUSE_BUTTON_RIGHT)  rightMouseDown = (action == GLFW_PRESS);
    if (button == GLFW_MOUSE_BUTTON_MIDDLE) middleMouseDown = (action == GLFW_PRESS);
    if (button == GLFW_MOUSE_BUTTON_LEFT)   leftMouseDown = (action == GLFW_PRESS);
    if (action == GLFW_PRESS) glfwGetCursorPos(window, &lastMouseX, &lastMouseY);
}
void CursorPosCallback(GLFWwindow* window, double xpos, double ypos)
{
    float dx = static_cast<float>(xpos - lastMouseX);
    float dy = static_cast<float>(ypos - lastMouseY);
    lastMouseX = xpos; lastMouseY = ypos;

    if (rightMouseDown) {
        float s = 0.3f;
        yaw -= dx * s;
        pitch += dy * s;
        pitch = std::clamp(pitch, -89.0f, 89.0f);
    }
    if (middleMouseDown) {
        float panSpeed = 0.01f;
        glm::vec3 front = glm::normalize(glm::vec3(sin(glm::radians(yaw)), 0.0f, cos(glm::radians(yaw))));
        glm::vec3 cameraRight = glm::normalize(glm::cross(front, glm::vec3(0.0f, 1.0f, 0.0f)));
        glm::vec3 cameraUp(0.0f, 1.0f, 0.0f);
        cameraTarget += cameraRight * dx * panSpeed;
        cameraTarget += cameraUp * dy * panSpeed;
    }
    if (leftMouseDown) {
        float s = 0.3f;
        lightYaw -= dx * s;
        lightPitch += dy * s;
        lightPitch = std::clamp(lightPitch, -89.0f, 89.0f);
    }
}

// ====== 파일 탐색 (네 코드 그대로) ======
std::vector<std::wstring> FindAllPMXFiles(const std::wstring& folderPath) {
    std::vector<std::wstring> result;
    if (!std::filesystem::exists(folderPath) || !std::filesystem::is_directory(folderPath)) {
        std::wcerr << "폴더가 존재하지 않거나 디렉토리가 아닙니다: " << folderPath << "\n";
        return result;
    }
    for (const auto& entry : std::filesystem::recursive_directory_iterator(folderPath)) {
        if (entry.is_regular_file() && entry.path().extension() == ".pmx") {
            result.push_back(entry.path().wstring());
        }
    }
    return result;
}
std::vector<std::wstring> FindAllVMDFiles(const std::wstring& folderPath) {
    std::vector<std::wstring> result;
    if (!std::filesystem::exists(folderPath) || !std::filesystem::is_directory(folderPath)) {
        std::wcerr << L"폴더가 존재하지 않거나 디렉토리가 아닙니다: " << folderPath << "\n";
        return result;
    }
    for (const auto& entry : std::filesystem::recursive_directory_iterator(folderPath)) {
        if (entry.is_regular_file() && entry.path().extension() == ".vmd") {
            result.push_back(entry.path().wstring());
        }
    }
    return result;
}

// ====== 간단 셰이더 래퍼 (네 코드 그대로 사용) ======
class Shader {
public:
    GLuint ID;
    Shader(const char* vertexPath, const char* fragmentPath) {
        std::string vertexCode, fragmentCode;
        std::ifstream vShaderFile(vertexPath), fShaderFile(fragmentPath);
        std::stringstream vss, fss;
        vss << vShaderFile.rdbuf(); fss << fShaderFile.rdbuf();
        vertexCode = vss.str(); fragmentCode = fss.str();
        const char* vCode = vertexCode.c_str(); const char* fCode = fragmentCode.c_str();
        GLuint vs = glCreateShader(GL_VERTEX_SHADER);
        glShaderSource(vs, 1, &vCode, NULL); glCompileShader(vs);
        GLuint fs = glCreateShader(GL_FRAGMENT_SHADER);
        glShaderSource(fs, 1, &fCode, NULL); glCompileShader(fs);
        ID = glCreateProgram(); glAttachShader(ID, vs); glAttachShader(ID, fs); glLinkProgram(ID);
        glDeleteShader(vs); glDeleteShader(fs);
    }
    void use() const { glUseProgram(ID); }
    void setMat4(const std::string& name, const glm::mat4& m) const {
        glUniformMatrix4fv(glGetUniformLocation(ID, name.c_str()), 1, GL_FALSE, glm::value_ptr(m));
    }

    // Shader helper에 추가
    void setVec3(const std::string& name, const glm::vec3& v) const {
        glUniform3fv(glGetUniformLocation(ID, name.c_str()), 1, glm::value_ptr(v));
    }
    void setInt(const std::string& name, int value) const {
        glUniform1i(glGetUniformLocation(ID, name.c_str()), value);
    }
};

// === 셰이더 유틸 함수 ===
void SetupStaticSamplers(const Shader& shader) {
    shader.use();
    shader.setInt("tex", 0);
    shader.setInt("toonTex", 1);
    shader.setInt("sphereTex", 2);
}

void SetupFrameUniforms(const Shader& shader,
    const glm::vec3& lightDir,
    const glm::vec3& cameraPos) {
    shader.use();
    shader.setVec3("lightDir", lightDir);
    shader.setVec3("cameraPos", cameraPos);
}

int main()
{
#ifdef _WIN32
    _setmode(_fileno(stderr), _O_U16TEXT);
#endif

    // 1) 파일 선택
    std::wstring modelFolder = L"C:/Users/Ha Yechan/Desktop/PMXViewer/models";
    auto pmxFiles = FindAllPMXFiles(modelFolder);
    if (pmxFiles.empty()) { std::wcerr << L"모델 폴더에 .pmx 파일이 없습니다: " << modelFolder << L"\n"; return 1; }

    std::wcerr << L"[ PMX 모델 목록 ]\n";
    for (size_t i = 0; i < pmxFiles.size(); ++i) std::wcerr << i << L": " << pmxFiles[i] << L"\n";
    int selected = -1; std::wcerr << L"\n불러올 모델 번호를 입력하세요: "; std::cin >> selected;
    if (selected < 0 || selected >= (int)pmxFiles.size()) { std::wcerr << L"잘못된 선택입니다.\n"; return 1; }
    std::wstring pmxPathW = pmxFiles[selected];

    std::wstring vmdFolder = L"C:/Users/Ha Yechan/Desktop/PMXViewer/motions";
    auto vmdFiles = FindAllVMDFiles(vmdFolder);
    if (vmdFiles.empty()) { std::wcerr << L"모션 폴더에 .vmd 파일이 없습니다: " << vmdFolder << L"\n"; return 1; }
    std::wcerr << L"\n[ VMD 파일 목록 ]\n";
    for (size_t i = 0; i < vmdFiles.size(); ++i) std::wcerr << i << L": " << vmdFiles[i] << L"\n";
    int vmdSelected = -1; std::wcerr << L"\n불러올 VMD 번호를 입력하세요: "; std::cin >> vmdSelected;
    if (vmdSelected < 0 || vmdSelected >= (int)vmdFiles.size()) { std::wcerr << L"잘못된 선택입니다.\n"; return 1; }

    // 2) 모델/모션 로드 (간단 확인용. 실제 Read는 PMXActor 내부 LoadModel/LoadMotion에서도 가능)
    std::unique_ptr<vmd::VmdMotion> motion(vmd::VmdMotion::LoadFromFile(vmdFiles[vmdSelected].c_str()));
    if (!motion) { std::wcerr << L"VMD 로딩 실패!\n"; return 1; }

    // 3) GLFW/GLAD
    if (!glfwInit()) { std::wcerr << L"GLFW 초기화 실패!\n"; return -1; }
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
#ifdef __APPLE__
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
#endif
    GLFWwindow* window = glfwCreateWindow(800, 600, "OpenGL 테스트 - PMXViewer", nullptr, nullptr);
    if (!window) { std::wcerr << L"윈도우 생성 실패!\n"; glfwTerminate(); return -1; }

    glfwMakeContextCurrent(window);
    glfwSetScrollCallback(window, ScrollCallback);
    glfwSetMouseButtonCallback(window, MouseButtonCallback);
    glfwSetCursorPosCallback(window, CursorPosCallback);

    if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress)) { std::wcerr << L"GLAD 초기화 실패!\n"; return -1; }
    std::wcerr << L"OpenGL 버전: " << glGetString(GL_VERSION) << std::endl;

    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    // 4) 셰이더
    Shader shader("C:/Users/Ha Yechan/Desktop/PMXViewer/shaders/vertex.glsl",
        "C:/Users/Ha Yechan/Desktop/PMXViewer/shaders/fragment.glsl");

    // 5) PMXActor 초기화 (네 인터페이스에 맞춤)
    PMXActor actor;
    if (!actor.LoadModel(pmxPathW)) { std::wcerr << L"PMX 로딩 실패!\n"; return -1; }
    if (!actor.LoadMotion(vmdFiles[vmdSelected])) { std::wcerr << L"VMD 적용 실패!\n"; return -1; }
    if (!actor.InitGL()) { std::wcerr << L"PMXActor InitGL 실패!\n"; return -1; }
    actor.SetModelScale(glm::vec3(0.3f, 0.3f, -0.3f)); // 필요 시 스케일 조정

    // 초기화 시 한 번만 실행
    SetupStaticSamplers(shader);

    // 6) 루프
    while (!glfwWindowShouldClose(window)) {
        glClearColor(0.2f, 0.3f, 0.3f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        // — 카메라
        float yawRad = glm::radians(yaw), pitchRad = glm::radians(pitch);
        glm::vec3 target = cameraTarget;
        glm::vec3 eye = target + glm::vec3(
            cameraDistance * cos(pitchRad) * sin(yawRad),
            cameraDistance * sin(pitchRad),
            cameraDistance * cos(pitchRad) * cos(yawRad));
        glm::vec3 up = glm::vec3(0, 1, 0);
        glm::vec3 viewDir = glm::normalize(target - eye);
        if (fabs(glm::dot(viewDir, up)) > 0.99f) up = glm::vec3(0, 0, 1);

        glm::mat4 view = glm::lookAt(eye, target, up);
        glm::mat4 proj = glm::perspective(glm::radians(45.0f), 800.f / 600.f, 0.1f, 100.f);

        // — 조명
        float ly = glm::radians(lightYaw), lp = glm::radians(lightPitch);
        glm::vec3 lightDir = glm::normalize(glm::vec3(
            cos(lp) * sin(ly), sin(lp), cos(lp) * cos(ly)));

        // — 셰이더
        shader.use();
        shader.setMat4("model", actor.GetModelMatrix());
        shader.setMat4("view", view);
        shader.setMat4("projection", proj);

        // 프레임마다 바뀌는 유니폼만 갱신
        SetupFrameUniforms(shader, lightDir, eye);

        // — 프레임 시간(30fps 기준)
        float t = static_cast<float>(glfwGetTime());
        float frameTime = t * 30.0f;

        // — 업데이트 & 드로우 (모든 모델 로직은 PMXActor 내부)
        actor.Update(frameTime);
        actor.Draw(shader.ID);

        glfwSwapBuffers(window);
        glfwPollEvents();
    }

    // 정리
    // PMXActor 소멸자에서 GL 리소스 정리한다면 아래는 생략 가능
    // actor.~PMXActor(); (자동 호출)
    glfwDestroyWindow(window);
    glfwTerminate();
    return 0;
}
