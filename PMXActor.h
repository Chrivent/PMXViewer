#pragma once

#include <glad/glad.h>
#include <vector>
#include <glm/glm.hpp>
#include <filesystem>
#include "Pmx.h"
#include "Vmd.h"
#include "NodeManager.h"

// ���������� ���� ���� ����������
struct PMXVertex {
    glm::vec3 position, normal;
    glm::vec2 uv;
    glm::vec4 additionalUV[4]{};
    pmx::PmxVertexSkinningType weightType;
    glm::ivec4 boneIndices{};
    glm::vec4  boneWeights{};
    glm::vec3  sdefC{}, sdefR0{}, sdefR1{};
    float      edgeMag = 0.f;
};

// ���������� PMXActor Ŭ���� ����������
class PMXActor {
public:
    bool Initialize(pmx::PmxModel& model, const std::filesystem::path& pmxPath);
    void InitAnimation(const vmd::VmdMotion* motion = nullptr);
    void Update(float dt);
    void Draw(const glm::mat4& view, const glm::mat4& proj, const glm::vec3& eye, const glm::vec3& lightDir) const;
    void UpdateAnimation(float dt);
    void Destroy();

private:
    pmx::PmxModel* mModel;

    /* GPU ���ҽ� */
    unsigned mVAO{}, mVBO{}, mEBO{};
    unsigned mUBOTransform{};
    unsigned mProgram{};                 // ���̴� ���α׷�
    std::vector<unsigned> mTextures;
    std::vector<unsigned> mToonTextures;

    /* ����޽á���� */
    struct Submesh { uint32_t ofs, cnt, mat; };
    std::vector<Submesh>   mSubmeshes;
    std::vector<glm::mat4> mLocal, mSkin;
    std::vector<pmx::PmxMaterial> mMaterials;

    /* �ִϸ��̼� */
    const vmd::VmdMotion* mMotion = nullptr;
    float mCurrentFrame = 0.f;

    /* ���� ���� */
    void buildBuffers(const pmx::PmxModel&);
    void buildMaterials(const pmx::PmxModel&);
    void buildTextures(const pmx::PmxModel&);
    void uploadBones() const;
    static unsigned createProgram();                 // VS��FS ������

    std::filesystem::path mModelDir;

    GLint locVP = -1;
    GLint locEye = -1;
    GLint locLight = -1;
    GLint locDiff = -1;
    GLint locSpec = -1;
    GLint locAmb = -1;

    NodeManager _nodeManager;
    float _elapsedTime = 0.0f;
};