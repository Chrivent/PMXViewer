#include "PMXActor.h"

#include <GLFW/glfw3.h>
#include <cstring>
#include <iostream>
#include <type_traits>
#include <array>
#include "stb_image.h"
#include "EncodingHelper.h"

using namespace pmx;
using oguna::EncodingConverter;

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

    float specB = pow(max(dot(N, H), 0.0), kSpec.w);
    vec3 specCol = kSpec.rgb * specB;

    vec3 ambCol = base + clamp(kAmb, vec3(0.0), vec3(0.5)) - vec3(0.5);
    vec3 finalRgb = diffCol + specCol + ambCol;

    FragColor = vec4(finalRgb, kDiffuse.a);
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
    std::vector<uint32_t>& i) {
    v.resize(m.vertex_count);

    for (uint32_t n = 0; n < m.vertex_count; ++n)
    {
        const auto& src = m.vertices[n];
        PMXVertex& dst = v[n];

        /* 위치·법선·UV */
        dst.position = { src.position[0], src.position[1], src.position[2] };
        dst.normal = { src.normal[0],   src.normal[1],   src.normal[2] };
        dst.uv = { src.uv[0],       src.uv[1] };

        /* -------- 스킨 가중치 처리 -------- */
        glm::ivec4 idx(0);
        glm::vec4  w(0.0f);

        const auto* sk = src.skinning.get();      // unique_ptr<PmxVertexSkinning>

        if (const auto* b1 = dynamic_cast<const PmxVertexSkinningBDEF1*>(sk))
        {
            idx.x = b1->bone_index;
            w.x = 1.0f;
        }
        else if (const auto* b2 = dynamic_cast<const PmxVertexSkinningBDEF2*>(sk))
        {
            idx.x = b2->bone_index1;
            idx.y = b2->bone_index2;
            w.x = 1.0f - b2->bone_weight;   // PMX:  weight = bone2
            w.y = b2->bone_weight;          //        1-weight = bone1
        }
        else if (const auto* b4 = dynamic_cast<const PmxVertexSkinningBDEF4*>(sk))
        {
            idx = { b4->bone_index1, b4->bone_index2,
                    b4->bone_index3, b4->bone_index4 };
            w = { b4->bone_weight1, b4->bone_weight2,
                    b4->bone_weight3, b4->bone_weight4 };
        }
        else if (const auto* sd = dynamic_cast<const PmxVertexSkinningSDEF*>(sk))
        {
            idx.x = sd->bone_index1;
            idx.y = sd->bone_index2;
            w.x = 1.0f - sd->bone_weight;
            w.y = sd->bone_weight;

            /* SDEF 전용 파라미터 */
            dst.sdefC = { sd->sdef_c[0], sd->sdef_c[1], sd->sdef_c[2] };
            dst.sdefR0 = { sd->sdef_r0[0], sd->sdef_r0[1], sd->sdef_r0[2] };
            dst.sdefR1 = { sd->sdef_r1[0], sd->sdef_r1[1], sd->sdef_r1[2] };
        }
        else if (const auto* q4 = dynamic_cast<const PmxVertexSkinningQDEF*>(sk))
        {
            idx = { q4->bone_index1, q4->bone_index2,
                    q4->bone_index3, q4->bone_index4 };
            w = { q4->bone_weight1, q4->bone_weight2,
                    q4->bone_weight3, q4->bone_weight4 };
        }
        else {
            // 예외 처리(잘못된 포인터) – 전부 0
        }

        dst.boneIdx = idx;
        dst.boneWeight = w;

        /* 에지 배율 */
        dst.edgeMag = src.edge;

        /* addUV 초기화 (필요 없다면 유지) */
        std::memset(dst.addUV, 0, sizeof(dst.addUV));
    }

    /* 인덱스 복사 */
    i.assign(m.indices.get(), m.indices.get() + m.index_count);
}

/* ---------- Initialize ---------- */
bool PMXActor::Initialize(const pmx::PmxModel& m, const fs::path& pmxPath, const vmd::VmdMotion* mot) {
    mMotion = mot;
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

    if (fallbackToon == 0)
        fallbackToon = createFallbackToon();

    glCreateBuffers(1, &mUBOTransform);
    glNamedBufferStorage(mUBOTransform,
        sizeof(glm::mat4) * m.bone_count, nullptr, GL_DYNAMIC_STORAGE_BIT);

    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    return true;
}

/* ---------- Update ---------- */
void PMXActor::Update(float dt) {
    mCurrentFrame += dt * 30.f;
    /* TODO: 키프레임 → mLocal,  IK → mSkin 계산 */
    if (!mSkin.empty()) uploadBones();
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

/* ---------- Destroy ---------- */
void PMXActor::Destroy() {
    glDeleteProgram(mProgram);
    glDeleteBuffers(1, &mVBO); glDeleteBuffers(1, &mEBO);
    glDeleteVertexArrays(1, &mVAO);
    if (!mTextures.empty())
        glDeleteTextures((GLsizei)mTextures.size(), mTextures.data());
    mTextures.clear(); mSubmeshes.clear(); mLocal.clear(); mSkin.clear();
    if (fallbackToon) {
        glDeleteTextures(1, &fallbackToon);
        fallbackToon = 0;
    }
}

/* ---------- buildBuffers ---------- */
void PMXActor::buildBuffers(const pmx::PmxModel& m) {
    std::vector<PMXVertex> v; std::vector<uint32_t> idx;
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
    add(7, 4, GL_INT, offsetof(PMXVertex, boneIdx), true);
    add(8, 4, GL_FLOAT, offsetof(PMXVertex, boneWeight));

    uint32_t cur = 0;
    for (uint32_t i = 0; i < m.material_count; ++i) {
        uint32_t icnt = m.materials[i].index_count;
        mSubmeshes.push_back({ cur, icnt, i });
        cur += icnt;
    }
    mLocal.resize(m.bone_count, glm::mat4(1)); mSkin = mLocal;
}

/* ---------- buildMaterials ---------- */
void PMXActor::buildMaterials(const pmx::PmxModel& m) {
    mMaterials.assign(m.materials.get(),
        m.materials.get() + m.material_count);
}

/* ---------- buildTextures (diffuse만) ---------- */
void PMXActor::buildTextures(const pmx::PmxModel& m) {
    EncodingConverter conv;
    mTextures.resize(m.material_count);          // diffuse
    mToonTextures.resize(m.material_count);      // 새 배열

    for (uint32_t i = 0; i < m.material_count; ++i)
    {
        auto load = [&](int idx)->GLuint {
            if (idx < 0 || idx >= m.texture_count) return 0;
            fs::path abs = mModelDir / m.textures[idx];
            std::string utf8; conv.Utf16ToUtf8(abs.c_str(),
                (int)abs.wstring().length(), &utf8);
            int w, h, n; stbi_uc* data = stbi_load(utf8.c_str(), &w, &h, &n, 4);
            if (!data) { std::wcerr << L"[stb] " << abs << L" 실패\n"; return 0; }
            GLuint tex; glCreateTextures(GL_TEXTURE_2D, 1, &tex);
            glTextureStorage2D(tex, 1, GL_SRGB8_ALPHA8, w, h);
            glTextureSubImage2D(tex, 0, 0, 0, w, h, GL_RGBA, GL_UNSIGNED_BYTE, data);
            glTextureParameteri(tex, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
            glTextureParameteri(tex, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
            glTextureParameteri(tex, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
            glTextureParameteri(tex, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
            stbi_image_free(data); return tex;
            };

        mTextures[i] = load(m.materials[i].diffuse_texture_index);
        mToonTextures[i] = load(m.materials[i].toon_texture_index);   // PMX 필드 참조 :contentReference[oaicite:3]{index=3}
        if (!mToonTextures[i]) mToonTextures[i] = fallbackToon;        // 기본 램프
    }
}

/* ---------- uploadBones ---------- */
void PMXActor::uploadBones()const {
    glNamedBufferSubData(mUBOTransform, 0,
        mSkin.size() * sizeof(glm::mat4), mSkin.data());
}

GLuint PMXActor::createFallbackToon()
{
    // 1×256 그라데이션 (흰→검, 위에서 아래로)
    const int W = 1, H = 256;
    std::array<unsigned char, W* H> data;
    for (int y = 0; y < H; ++y)
        data[y] = static_cast<unsigned char>(255 - y);   // 상단 밝음

    GLuint tex;
    glCreateTextures(GL_TEXTURE_2D, 1, &tex);
    glTextureStorage2D(tex, 1, GL_R8, W, H);             // 단일 채널
    glTextureSubImage2D(tex, 0, 0, 0, W, H,
        GL_RED, GL_UNSIGNED_BYTE, data.data());
    glTextureParameteri(tex, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTextureParameteri(tex, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTextureParameteri(tex, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTextureParameteri(tex, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    return tex;
}