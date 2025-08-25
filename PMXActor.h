#pragma once

#include <future>
#include <glad/glad.h>
#include <glm/gtc/quaternion.hpp>

#include "Pmx.h"
#include "Vmd.h"
#include "NodeManager.h"

struct GLVertex {
    glm::vec3 position{};
    glm::vec3 normal{};
    glm::vec2 uv{};
};

struct SkinningRange {
    unsigned startIndex = 0;
    unsigned vertexCount = 0;
};

class PMXActor {
public:
    PMXActor() = default;
    ~PMXActor();

    // 파일 로드 & GPU 리소스 준비
    bool LoadModel(const std::wstring& pmxPath);
    bool LoadMotion(const std::wstring& vmdPath);
    bool InitGL(); // VAO/VBO/EBO, 텍스처 등

    // 프레임별 업데이트/드로우
    void Update(float frameTime30fps);  // NodeManager, BonePalette, CPU 스키닝, VBO 업데이트
    void Draw(GLuint shaderProgram);    // 머티리얼 루프 + glDrawElements

    // 카메라 외부에서 세팅한 행렬/유니폼은 main에서 셰이더에 넣고, Draw는 지오메트리만.
    // 필요하면 여기서 model 행렬만 넘겨주도록 해도 됨.

    // 액터의 모델 행렬(스케일만 걸고 있음)
    glm::mat4 GetModelMatrix() const { return _modelScale; }
    void SetModelScale(const glm::vec3& s);

private:
    // 내부 헬퍼
    void buildInitialVertices();
    void buildSkinningPartitions(size_t vertexCount);
    void buildBonePalette();
    static void skinningByRange(
        const pmx::PmxModel& model,
        const std::vector<glm::mat4>& bonePalette,
        std::vector<GLVertex>& outVertices,
        unsigned start,
        unsigned count);

    void uploadVertexBufferOrphaned();

    bool loadAllTextures(const std::string& pmxBaseDir);
    GLuint createTextureFromFile(const std::filesystem::path& path);

private:
    // CPU 데이터
    pmx::PmxModel _model;
    std::unique_ptr<vmd::VmdMotion> _motion;
    NodeManager _nodeManager;

    std::vector<GLVertex> _vertices;   // 스키닝된 결과가 매 프레임 들어감
    std::vector<uint32_t> _indices;
    std::vector<glm::mat4> _bonePalette;

    // 병렬 스키닝
    std::vector<SkinningRange> _ranges;
    std::vector<std::future<void>> _futures;

    // GPU 리소스
    GLuint _vao = 0, _vbo = 0, _ebo = 0;
    std::vector<GLuint> _textures; // PMX 텍스처 인덱스대로

    // 상태
    bool _glReady = false;
    glm::mat4 _modelScale = glm::scale(glm::mat4(1.0f), glm::vec3(0.3f, 0.3f, -0.3f));
};