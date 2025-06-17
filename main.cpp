#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <iostream>
#include <fstream>
#include <filesystem>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include "Pmx.h"

using namespace std;

struct GLVertex {
    glm::vec3 position;
    glm::vec3 normal;
    glm::vec2 uv;
};

GLuint vao, vbo, ebo;
vector<GLVertex> gVertices;
vector<uint32_t> gIndices;

float cameraDistance = 10.0f;
float yaw = 0.0f;
float pitch = 0.0f;
bool rightMouseDown = false;
bool middleMouseDown = false;
double lastMouseX = 0.0, lastMouseY = 0.0;

glm::vec3 cameraTarget = glm::vec3(0, 5, 0);

class Shader {
public:
    GLuint ID;

    Shader(const char* vertexPath, const char* fragmentPath) {
        std::string vertexCode;
        std::string fragmentCode;
        std::ifstream vShaderFile(vertexPath);
        std::ifstream fShaderFile(fragmentPath);

        std::stringstream vShaderStream, fShaderStream;
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

    void setMat4(const std::string& name, const glm::mat4& mat) const {
        glUniformMatrix4fv(glGetUniformLocation(ID, name.c_str()), 1, GL_FALSE, glm::value_ptr(mat));
    }
};

// 마우스 휠 콜백
void ScrollCallback(GLFWwindow* window, double xoffset, double yoffset)
{
    cameraDistance -= static_cast<float>(yoffset);
    cameraDistance = std::clamp(cameraDistance, 2.0f, 100.0f);
}

void MouseButtonCallback(GLFWwindow* window, int button, int action, int mods)
{
    if (button == GLFW_MOUSE_BUTTON_RIGHT)
        rightMouseDown = (action == GLFW_PRESS);

    if (button == GLFW_MOUSE_BUTTON_MIDDLE)
        middleMouseDown = (action == GLFW_PRESS);

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
        pitch = std::clamp(pitch, -89.0f, 89.0f);
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

int main()
{
    string modelFolder = "C:/Users/Ha Yechan/Desktop/PMXViewer/models";
    vector<string> pmxFiles = FindAllPMXFiles(modelFolder);

    if (pmxFiles.empty()) {
        cerr << "모델 폴더에 .pmx 파일이 없습니다: " << modelFolder << "\n";
        return 1;
    }

    // 목록 출력
    cout << "[ PMX 모델 목록 ]\n";
    for (size_t i = 0; i < pmxFiles.size(); ++i) {
        cout << i << ": " << pmxFiles[i] << "\n";
    }

    // 선택 입력
    int selected = -1;
    cout << "\n불러올 모델 번호를 입력하세요: ";
    cin >> selected;

    if (selected < 0 || selected >= static_cast<int>(pmxFiles.size())) {
        cerr << "잘못된 선택입니다.\n";
        return 1;
    }

    string pmxPath = pmxFiles[selected];
    cout << "선택된 PMX 파일: " << pmxPath << "\n";

    pmx::PmxModel model;
    std::ifstream file(pmxPath, std::ios::binary);
    model.Read(&file);

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

    cout << "OpenGL 버전: " << glGetString(GL_VERSION) << endl;

    Shader shader("C:/Users/Ha Yechan/Desktop/PMXViewer/shaders/vertex.glsl", "C:/Users/Ha Yechan/Desktop/PMXViewer/shaders/fragment.glsl");

    glEnable(GL_DEPTH_TEST);

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
        gVertices.push_back(v);
    }
    gIndices.assign(model.indices.get(), model.indices.get() + model.index_count);

    glBindBuffer(GL_ARRAY_BUFFER, vbo);
    glBufferData(GL_ARRAY_BUFFER, gVertices.size() * sizeof(GLVertex), gVertices.data(), GL_STATIC_DRAW);

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ebo);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, gIndices.size() * sizeof(uint32_t), gIndices.data(), GL_STATIC_DRAW);

    // 속성 연결
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(GLVertex), (void*)offsetof(GLVertex, position));

    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(GLVertex), (void*)offsetof(GLVertex, normal));

    glEnableVertexAttribArray(2);
    glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, sizeof(GLVertex), (void*)offsetof(GLVertex, uv));

    glBindVertexArray(0);

    // 루프
    while (!glfwWindowShouldClose(window)) {
        // 배경색 지정 및 지우기
        glClearColor(0.2f, 0.3f, 0.3f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        // 모델 스케일 적용
        glm::mat4 model = glm::scale(glm::mat4(1.0f), glm::vec3(0.3f));

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
        shader.setMat4("model", model);
        shader.setMat4("view", view);
        shader.setMat4("projection", projection);

        // 🔽 모델 그리기 (VAO 바인딩 및 그리기)
        glBindVertexArray(vao);
        glDrawElements(GL_TRIANGLES, static_cast<GLsizei>(gIndices.size()), GL_UNSIGNED_INT, 0);

        // 화면 표시 및 이벤트 처리
        glfwSwapBuffers(window);
        glfwPollEvents();
    }

    // 정리
    glfwDestroyWindow(window);
    glfwTerminate();
    return 0;
}
