#pragma once

#include <glad/glad.h>
#include <vector>
#include <glm/glm.hpp>
#include <filesystem>
#include "Pmx.h"
#include "Vmd.h"

namespace fs = std::filesystem;

// ���������� ���� ���� ����������
struct PMXVertex {
    glm::vec3 position, normal;
    glm::vec2 uv;
    glm::vec4 addUV[4]{};
    glm::ivec4 boneIdx{};
    glm::vec4  boneWeight{};
    glm::vec3  sdefC{}, sdefR0{}, sdefR1{};
    float      edgeMag = 0.f;
};

// ���������� PMXActor Ŭ���� ����������
class PMXActor {
public:
    bool Initialize(const pmx::PmxModel& model,
        const fs::path& pmxPath,
        const vmd::VmdMotion* motion = nullptr);
    void Update(float dt);
    void Draw(const glm::mat4& view,
        const glm::mat4& proj,
        const glm::vec3& eye,
        const glm::vec3& lightDir) const;
    void Destroy();

private:
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

    fs::path mModelDir;
    GLuint fallbackToon = 0;

    GLuint createFallbackToon();

    GLint locVP = -1;
    GLint locEye = -1;
    GLint locLight = -1;
    GLint locDiff = -1;
    GLint locSpec = -1;
    GLint locAmb = -1;
};