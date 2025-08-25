#pragma once

#include <future>
#include <glad/glad.h>
#include <glm/gtc/quaternion.hpp>

#include "Pmx.h"
#include "Vmd.h"
#include "NodeManager.h"
#include "MorphManager.h"

// 정점 포맷
struct GLVertex {
    glm::vec3 position;
    glm::vec3 normal;
    glm::vec2 uv;
};

// 병렬 스키닝을 위한 범위
struct VertexRange {
    unsigned startIndex = 0;
    unsigned vertexCount = 0;
};

struct Affine34 { float m[12]; };

struct EffectiveMaterial {
    uint8_t opType;
    glm::vec4 diffuse;
    glm::vec3 specular;
    float     specularPower;
    glm::vec3 ambient;
    glm::vec4 edgeColor;
    float     edgeSize;
    glm::vec4 texFactor;
    glm::vec4 sphereFactor;
    glm::vec4 toonFactor;
};

class PMXActor {
public:
    PMXActor() = default;
    ~PMXActor();

    // 로드
    bool LoadModel(const std::wstring& pmxPath);
    bool LoadMotion(const std::wstring& vmdPath);

    // GL 초기화/드로우
    bool InitGL();
    void Update(float frameTime30);
    void Draw(GLuint shader);

    glm::mat4 GetModelMatrix() const { return _modelScale; }
    void SetModelScale(const glm::vec3& s);

private:
    // 초기 버텍스 만들기 (비스키닝, 스키닝 입력용)
    void buildInitialVertices();
    // 파티션 구성 (스레드별 범위)
    void buildSkinningPartitions(size_t vertexCount);

    static Affine34 toA34(const glm::mat4& M);
    static glm::vec3 transformPoint(const Affine34& A, const glm::vec3& v);
    static glm::vec3 transformDir(const Affine34& A, const glm::vec3& v);
    static glm::vec3 fastNormalize(const glm::vec3& v);

    // 본 팔레트 구성
    void buildBonePalette();
    void buildBonePaletteA34();

    // 스키닝 (범위별)
    void skinningByRange(
        const pmx::PmxModel& model,
        const std::vector<Affine34>& bonePaletteA34,
        std::vector<GLVertex>& out,
        unsigned start, unsigned count);

    // VBO 업로드 (맵/언맵 + invalidate + unsync)
    void uploadVertexBufferFast();

    // 텍스처
    bool   loadAllTextures(const std::string& base);
    GLuint createTextureFromFile(const std::filesystem::path& path);

    void MorphMaterial();
    void MorphBone();

private:
    // 데이터
    pmx::PmxModel _model;
    std::unique_ptr<vmd::VmdMotion> _motion;
    NodeManager _nodeManager;
    MorphManager _morphManager;

    std::vector<GLVertex>  _vertices;  // CPU-side skinned vertices
    std::vector<uint32_t>  _indices;
    std::vector<glm::mat4> _bonePalette;
    std::vector<Affine34>   _bonePaletteA34;

    // GL 자원
    GLuint _vao = 0;
    GLuint _vbo = 0;
    GLuint _ebo = 0;
    bool   _glReady = false;

    // 텍스처
    std::vector<GLuint> _textures;

    // 병렬 스키닝
    std::vector<VertexRange> _ranges;

    // 드로우용 상태 캐시
    glm::mat4 _modelScale = glm::mat4(1.0f);

    // 텍스처 바인딩 최소화 캐시
    GLint _lastTex0 = -1, _lastTex1 = -1, _lastTex2 = -1;

    std::vector<EffectiveMaterial> _matCache;
};
