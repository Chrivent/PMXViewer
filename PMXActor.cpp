#include "PMXActor.h"

#include <cstring>
#include <iostream>
#include <type_traits>
#include <array>
#include "stb_image.h"
#include "EncodingHelper.h"
#include "BoneNode.h"
#include "NodeManager.h"
#include "Pmx.h"
#include "Vmd.h"

/* ---------- 셰이더 소스 ---------- */
static const char* kVS = R"GLSL(
#version 330 core
layout(location=0)in vec3 aPos; layout(location=1)in vec3 aNrm;
layout(location=2)in vec2 aUV;  layout(location=7)in ivec4 aBi;
layout(location=8)in vec4 aBw;

layout(std140,binding=0)uniform Bones{mat4 uB[128];};
uniform mat4 uVP;
uniform vec3 uEye;
uniform vec3 uLightDir;

out VSOUT{
    vec2 uv;
    vec3 nrmW;
    vec3 viewW;
} vs;

void main(){
    mat4 skin = aBw.x*uB[aBi.x] + aBw.y*uB[aBi.y]
              + aBw.z*uB[aBi.z] + aBw.w*uB[aBi.w];
    vec4 wp = skin * vec4(aPos,1.);
    vs.nrmW  = mat3(skin)*aNrm;
    vs.viewW = normalize(uEye - wp.xyz);
    vs.uv    = aUV;
    gl_Position = uVP * wp;
}
)GLSL";

static const char* kFS = R"GLSL(
#version 330 core
in VSOUT{
    vec2 uv;
    vec3 nrmW;
    vec3 viewW;
} fs;
out vec4 FragColor;
layout(binding = 2) uniform sampler2D uTex;
layout(binding = 3) uniform sampler2D uToon;

uniform vec4  kDiffuse;
uniform vec4  kSpec;
uniform vec3  kAmb;

uniform vec3  uLightDir;
uniform vec3  uEye;

uniform bool  bUseToon;

void main(){
    vec3 base = texture(uTex, fs.uv).rgb * kDiffuse.rgb;

    vec3 N = normalize(fs.nrmW);
    vec3 L = normalize(-uLightDir);
    vec3 V = normalize(fs.viewW);
    vec3 H = normalize(L + V);

    float NdotL = clamp(dot(N, L), 0.0, 1.0);
    vec3 toonCol = bUseToon
        ? texture(uToon, vec2(0.0, 1.0 - NdotL)).rgb
        : vec3(1.0);

    vec3 diffCol = base * toonCol;

    float specB = pow(max(dot(N, H), 0.0), kSpec.a);
    vec3 specCol = kSpec.rgb * specB;

    vec3 ambCol = clamp(kAmb, vec3(0.0), vec3(0.5)) - vec3(0.5);
    vec3 finalRgb = diffCol + specCol + ambCol;

    FragColor = vec4(finalRgb , kDiffuse.a);
}
)GLSL";

/* ---------- 유틸 ---------- */
static unsigned compile(GLenum t, const char* s) {
    unsigned id = glCreateShader(t);
    glShaderSource(id, 1, &s, nullptr); glCompileShader(id);
    GLint ok; glGetShaderiv(id, GL_COMPILE_STATUS, &ok);
    if (!ok) {
        char log[512]; glGetShaderInfoLog(id, 512, nullptr, log);
        std::wcerr << L"shader err:\n" << log << L'\n';
    }
    return id;
}
unsigned PMXActor::createProgram() {
    unsigned vs = compile(GL_VERTEX_SHADER, kVS);
    unsigned fs = compile(GL_FRAGMENT_SHADER, kFS);
    unsigned p = glCreateProgram();
    glAttachShader(p, vs); glAttachShader(p, fs); glLinkProgram(p);
    glDeleteShader(vs); glDeleteShader(fs);
    return p;
}

/* ---------- 정점 변환 ---------- */
static void convertVertices(const pmx::PmxModel& m,
    std::vector<PMXVertex>& v,
    std::vector<uint32_t>& i)
{
    v.resize(m.vertex_count);

    for (uint32_t n = 0; n < m.vertex_count; ++n)
    {
        const auto& src = m.vertices[n];
        PMXVertex& dst = v[n];

        /* ── 위치·법선·UV ── */
        dst.position = { src.position[0], src.position[1], src.position[2] };
        dst.normal = { src.normal[0] , src.normal[1] , src.normal[2] };
        dst.uv = { src.uv[0]     , src.uv[1] };

        /* ── 스킨 가중치 ── */
        glm::ivec4 idx(0);
        glm::vec4  w(0.0f);

        const auto* sk = src.skinning.get();           // actual type 확인
        using ST = pmx::PmxVertexSkinningType;

        if (auto* s = dynamic_cast<const pmx::PmxVertexSkinningBDEF1*>(sk))
        {
            dst.weightType = pmx::PmxVertexSkinningType::BDEF1;
            idx.x = s->bone_index;
            w.x = 1.0f;
        }
        else if (auto* s = dynamic_cast<const pmx::PmxVertexSkinningBDEF2*>(sk))
        {
            dst.weightType = pmx::PmxVertexSkinningType::BDEF2;
            idx.x = s->bone_index1;
            idx.y = s->bone_index2;
            w.x = 1.0f - s->bone_weight;   // bone1
            w.y = s->bone_weight;          // bone2
        }
        else if (auto* s = dynamic_cast<const pmx::PmxVertexSkinningBDEF4*>(sk))
        {
            dst.weightType = pmx::PmxVertexSkinningType::BDEF4;
            idx = { s->bone_index1, s->bone_index2,
                    s->bone_index3, s->bone_index4 };
            w = { s->bone_weight1, s->bone_weight2,
                    s->bone_weight3, s->bone_weight4 };
        }
        else if (auto* s = dynamic_cast<const pmx::PmxVertexSkinningSDEF*>(sk))
        {
            dst.weightType = pmx::PmxVertexSkinningType::SDEF;
            idx.x = s->bone_index1;
            idx.y = s->bone_index2;
            w.x = 1.0f - s->bone_weight;
            w.y = s->bone_weight;

            /* SDEF 전용 파라미터 */
            dst.sdefC = { s->sdef_c[0], s->sdef_c[1], s->sdef_c[2] };
            dst.sdefR0 = { s->sdef_r0[0], s->sdef_r0[1], s->sdef_r0[2] };
            dst.sdefR1 = { s->sdef_r1[0], s->sdef_r1[1], s->sdef_r1[2] };
        }
        else if (auto* s = dynamic_cast<const pmx::PmxVertexSkinningQDEF*>(sk))
        {
            dst.weightType = pmx::PmxVertexSkinningType::QDEF;
            idx = { s->bone_index1, s->bone_index2,
                    s->bone_index3, s->bone_index4 };
            w = { s->bone_weight1, s->bone_weight2,
                    s->bone_weight3, s->bone_weight4 };
        }
        else
        {
            /* 예외‧미지정 → 단일본으로 취급 */
            dst.weightType = pmx::PmxVertexSkinningType::BDEF1;
            idx.x = 0;  w.x = 1.0f;
        }

        dst.boneIndices = idx;
        dst.boneWeights = w;

        /* ── 기타 속성 ── */
        dst.edgeMag = src.edge;
        std::memset(dst.additionalUV, 0, sizeof(dst.additionalUV));
    }

    /* 인덱스 복사 */
    i.assign(m.indices.get(), m.indices.get() + m.index_count);
}

/* ---------- Initialize ---------- */
bool PMXActor::Initialize(pmx::PmxModel& m, const std::filesystem::path& pmxPath) {
    mModel = &m;
    mProgram = createProgram();
    mModelDir = pmxPath.parent_path();

    locVP = glGetUniformLocation(mProgram, "uVP");
    locEye = glGetUniformLocation(mProgram, "uEye");
    locLight = glGetUniformLocation(mProgram, "uLightDir");
    locDiff = glGetUniformLocation(mProgram, "kDiffuse");
    locSpec = glGetUniformLocation(mProgram, "kSpec");
    locAmb = glGetUniformLocation(mProgram, "kAmb");

    buildBuffers(m);
    buildTextures(m);
    buildMaterials(m);

    glCreateBuffers(1, &mUBOTransform);
    glNamedBufferStorage(mUBOTransform,
        sizeof(glm::mat4) * m.bone_count, nullptr, GL_DYNAMIC_STORAGE_BIT);

    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    std::vector<const pmx::PmxBone*> boneVec;
    boneVec.reserve(m.bone_count);
    for (uint32_t i = 0; i < m.bone_count; ++i) {
        boneVec.push_back(&m.bones[i]);  // 포인터만 저장
    }
    _nodeManager.Init(boneVec);

    return true;
}

void PMXActor::InitAnimation(const vmd::VmdMotion* motion)
{
    if (!motion) return;

    for (const auto& frame : motion->bone_frames)
    {
        std::wstring boneName;
        oguna::EncodingConverter{}.Cp932ToUtf16(frame.name.c_str(), (int)frame.name.length(), &boneName);

        BoneNode* boneNode = _nodeManager.GetBoneNodeByName(boneName);
        if (!boneNode) continue;

        glm::quat quaternion(frame.orientation[3], frame.orientation[0], frame.orientation[1], frame.orientation[2]);
        glm::vec3 offset(frame.position[0], frame.position[1], frame.position[2]);

        glm::vec2 p1 = {
            static_cast<float>(frame.interpolation[0][1][0]) / 127.0f,
            static_cast<float>(frame.interpolation[0][1][1]) / 127.0f
        };

        glm::vec2 p2 = {
            static_cast<float>(frame.interpolation[0][1][2]) / 127.0f,
            static_cast<float>(frame.interpolation[0][1][3]) / 127.0f
        };

        boneNode->AddMotionKey(frame.frame, quaternion, offset, p1, p2);
    }

    _nodeManager.SortKey();
}

/* ---------- Update ---------- */
void PMXActor::Update(float dt) {

}

/* ---------- Draw ---------- */
void PMXActor::Draw(const glm::mat4& view, const glm::mat4& proj, const glm::vec3& eye, const glm::vec3& lightDir) const {
    glUseProgram(mProgram);
    glProgramUniform3fv(mProgram, glGetUniformLocation(mProgram, "uEye"), 1, &eye.x);
    glProgramUniform3fv(mProgram, glGetUniformLocation(mProgram, "uLightDir"), 1, &lightDir.x);
    glm::mat4 vp = proj * view;
    glUniformMatrix4fv(glGetUniformLocation(mProgram, "uVP"), 1, GL_FALSE, &vp[0][0]);

    glBindVertexArray(mVAO);
    glBindBufferBase(GL_UNIFORM_BUFFER, 0, mUBOTransform);

    for (auto& s : mSubmeshes) {
        const auto& mt = mMaterials[s.mat];
        float diff[4] = { mt.diffuse[0],  mt.diffuse[1], mt.diffuse[2], mt.diffuse[3] };
        float spec[4] = { mt.specular[0], mt.specular[1], mt.specular[2], mt.specularlity };
        float amb[3] = { mt.ambient[0],  mt.ambient[1],  mt.ambient[2] };
        glProgramUniform4fv(mProgram, locDiff, 1, diff);
        glProgramUniform4fv(mProgram, locSpec, 1, spec);
        glProgramUniform3fv(mProgram, locAmb, 1, amb);
        glBindTextureUnit(2, mTextures[s.mat]);
        bool hasToon = mToonTextures[s.mat] != 0;
        glUniform1i(glGetUniformLocation(mProgram, "bUseToon"), hasToon);
        if (hasToon)
            glBindTextureUnit(3, mToonTextures[s.mat]);
        else
            glBindTextureUnit(3, 0);
        glDrawElementsBaseVertex(GL_TRIANGLES, s.cnt, GL_UNSIGNED_INT, (void*)(sizeof(uint32_t) * s.ofs), 0);
    }
}

/* ---------- Update Animation ---------- */
void PMXActor::UpdateAnimation(float dt) {
    _elapsedTime += dt;
    unsigned int frameNo = static_cast<unsigned int>(_elapsedTime * 30.0f);  // 30fps 기준

    if (frameNo > 300) {
        _elapsedTime = 0.0f;
        frameNo = 0;
    }

    _nodeManager.UpdateAnimation(frameNo);

    for (int i = 0; i < mSkin.size(); ++i) {
        const BoneNode* bn = _nodeManager.GetBoneNodeByIndex(i);
        /* ✅ ① 글로벌 * 바인드-포즈(초기행렬)의 역행렬 */
        mSkin[i] = bn->GetGlobalTransform() * bn->GetInitInverseTransform();
    }

    if (!mSkin.empty()) uploadBones();
}

/* ---------- Destroy ---------- */
void PMXActor::Destroy() {
    glDeleteProgram(mProgram);
    glDeleteBuffers(1, &mVBO); glDeleteBuffers(1, &mEBO);
    glDeleteVertexArrays(1, &mVAO);
    if (!mTextures.empty())
        glDeleteTextures((GLsizei)mTextures.size(), mTextures.data());
    mTextures.clear(); mSubmeshes.clear(); mLocal.clear(); mSkin.clear();
}

/* ---------- buildBuffers ---------- */
void PMXActor::buildBuffers(const pmx::PmxModel& m) {
    std::vector<PMXVertex> v;
    std::vector<uint32_t> idx;
    convertVertices(m, v, idx);

    glCreateBuffers(1, &mVBO);
    glNamedBufferStorage(mVBO, v.size() * sizeof(PMXVertex),
        v.data(), GL_DYNAMIC_STORAGE_BIT);
    glCreateBuffers(1, &mEBO);
    glNamedBufferStorage(mEBO, idx.size() * 4, idx.data(), 0);

    glCreateVertexArrays(1, &mVAO);
    glVertexArrayVertexBuffer(mVAO, 0, mVBO, 0, sizeof(PMXVertex));
    glVertexArrayElementBuffer(mVAO, mEBO);

    auto add = [&](GLuint loc, int comp, GLenum t, std::size_t ofs, bool intAttr = false) {
        if (intAttr) glVertexArrayAttribIFormat(mVAO, loc, comp, t, ofs);
        else        glVertexArrayAttribFormat(mVAO, loc, comp, t, GL_FALSE, ofs);
        glEnableVertexArrayAttrib(mVAO, loc); glVertexArrayAttribBinding(mVAO, loc, 0);
        };
    add(0, 3, GL_FLOAT, offsetof(PMXVertex, position));
    add(1, 3, GL_FLOAT, offsetof(PMXVertex, normal));
    add(2, 2, GL_FLOAT, offsetof(PMXVertex, uv));
    add(7, 4, GL_INT, offsetof(PMXVertex, boneIndices), true);
    add(8, 4, GL_FLOAT, offsetof(PMXVertex, boneWeights));

    uint32_t cur = 0;
    for (uint32_t i = 0; i < m.material_count; ++i) {
        uint32_t icnt = m.materials[i].index_count;
        mSubmeshes.push_back({ cur, icnt, i });
        cur += icnt;
    }
    mLocal.resize(m.bone_count, glm::mat4(1));
    mSkin = mLocal;
}

/* ---------- buildMaterials ---------- */
void PMXActor::buildMaterials(const pmx::PmxModel& m) {
    mMaterials.assign(m.materials.get(), m.materials.get() + m.material_count);
}

/* ---------- buildTextures (diffuse만) ---------- */
void PMXActor::buildTextures(const pmx::PmxModel& m) {
    oguna::EncodingConverter conv;
    mTextures.resize(m.material_count);          // diffuse
    mToonTextures.resize(m.material_count);      // 새 배열

    for (uint32_t i = 0; i < m.material_count; ++i)
    {
        auto load = [&](int idx)->GLuint {
            if (idx < 0 || idx >= m.texture_count) return 0;
            std::filesystem::path abs = mModelDir / m.textures[idx];
            std::string utf8; conv.Utf16ToUtf8(abs.c_str(),
                (int)abs.wstring().length(), &utf8);
            int w, h, n; stbi_uc* data = stbi_load(utf8.c_str(), &w, &h, &n, 4);
            if (!data) { std::wcerr << L"[stb] " << abs << L" 실패\n"; return 0; }
            GLuint tex; glCreateTextures(GL_TEXTURE_2D, 1, &tex);
            glTextureStorage2D(tex, 1, GL_RGBA8, w, h);
            glTextureSubImage2D(tex, 0, 0, 0, w, h, GL_RGBA, GL_UNSIGNED_BYTE, data);
            glTextureParameteri(tex, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
            glTextureParameteri(tex, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
            glTextureParameteri(tex, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
            glTextureParameteri(tex, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
            stbi_image_free(data); return tex;
            };

        mTextures[i] = load(m.materials[i].diffuse_texture_index);
        mToonTextures[i] = load(m.materials[i].toon_texture_index);
    }
}

/* ---------- uploadBones ---------- */
void PMXActor::uploadBones()const {
    glNamedBufferSubData(mUBOTransform, 0,
        mSkin.size() * sizeof(glm::mat4), mSkin.data());
}