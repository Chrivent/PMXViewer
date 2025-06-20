#include "PMXActor.h"

#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <cstring>
#include <iostream>
#include <type_traits>
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

layout(std140,binding=0)uniform Bones{mat4 uB[512];};
uniform mat4 uVP;

out VSOUT{vec2 uv; vec3 nrm;} vs;
void main(){
    mat4 skin = aBw.x*uB[aBi.x]+aBw.y*uB[aBi.y]+aBw.z*uB[aBi.z]+aBw.w*uB[aBi.w];
    vec4 wp   = skin*vec4(aPos,1);
    vs.nrm    = mat3(skin)*aNrm;
    vs.uv     = aUV;
    gl_Position = uVP*wp;
}
)GLSL";

static const char* kFS = R"GLSL(
#version 330 core
in VSOUT{vec2 uv; vec3 nrm;} fs;
out vec4 FragColor;
layout(binding=2)uniform sampler2D uTex;
void main(){
    vec3 base = texture(uTex,fs.uv).rgb;
    vec3 L = normalize(vec3(-0.3,1,0.4));
    float NdotL = max(dot(normalize(fs.nrm),L),0);
    FragColor = vec4(base*0.25 + base*0.75*NdotL,1);
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
    mMotion = mot; mProgram = createProgram(); mModelDir = pmxPath.parent_path();
    buildBuffers(m);  buildTextures(m);  buildMaterials(m);

    glCreateBuffers(1, &mUBOTransform);
    glNamedBufferStorage(mUBOTransform,
        sizeof(glm::mat4) * m.bone_count, nullptr, GL_DYNAMIC_STORAGE_BIT);

    glCreateBuffers(1, &mUBOMaterial);
    glNamedBufferStorage(mUBOMaterial,
        sizeof(glm::vec4) * m.material_count * 3, nullptr, GL_DYNAMIC_STORAGE_BIT);

    return true;
}

/* ---------- Update ---------- */
void PMXActor::Update(float dt) {
    mCurrentFrame += dt * 30.f;
    /* TODO: 키프레임 → mLocal,  IK → mSkin 계산 */
    if (!mSkin.empty()) uploadBones();
}

/* ---------- Draw ---------- */
void PMXActor::Draw(const glm::mat4& view, const glm::mat4& proj)const {
    glUseProgram(mProgram);
    glm::mat4 vp = proj * view;
    glUniformMatrix4fv(glGetUniformLocation(mProgram, "uVP"),
        1, GL_FALSE, &vp[0][0]);

    glBindVertexArray(mVAO);
    glBindBufferBase(GL_UNIFORM_BUFFER, 0, mUBOTransform);
    glBindBufferBase(GL_UNIFORM_BUFFER, 1, mUBOMaterial);
    for (auto& s : mSubmeshes) {
        glBindTextureUnit(2, mTextures[s.mat]);
        glDrawElementsBaseVertex(GL_TRIANGLES, s.cnt, GL_UNSIGNED_INT,
            (void*)(sizeof(uint32_t) * s.ofs), 0);
    }
}

/* ---------- Destroy ---------- */
void PMXActor::Destroy() {
    glDeleteProgram(mProgram);
    glDeleteBuffers(1, &mVBO); glDeleteBuffers(1, &mEBO);
    glDeleteVertexArrays(1, &mVAO);
    glDeleteBuffers(1, &mUBOTransform); glDeleteBuffers(1, &mUBOMaterial);
    if (!mTextures.empty())
        glDeleteTextures((GLsizei)mTextures.size(), mTextures.data());
    mTextures.clear(); mSubmeshes.clear(); mLocal.clear(); mSkin.clear();
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
    struct M { glm::vec4 d, s, a; };
    std::vector<M> buf(m.material_count);
    for (uint32_t i = 0; i < m.material_count; ++i) {
        auto& s = m.materials[i];
        buf[i].d = { s.diffuse[0],s.diffuse[1],s.diffuse[2],s.diffuse[3] };
        buf[i].s = { s.specular[0],s.specular[1],s.specular[2],s.specularlity };
        buf[i].a = { s.ambient[0],s.ambient[1],s.ambient[2],1.0f };
    }
    glNamedBufferSubData(mUBOMaterial, 0, buf.size() * sizeof(M), buf.data());
}

/* ---------- buildTextures (diffuse만) ---------- */
void PMXActor::buildTextures(const pmx::PmxModel& m) {
    EncodingConverter conv;
    mTextures.resize(m.material_count, 0);
    for (uint32_t i = 0; i < m.material_count; ++i) {
        int tid = m.materials[i].diffuse_texture_index;
        if (tid < 0 || tid >= m.texture_count) continue;
        fs::path abs = mModelDir / m.textures[tid];
        std::string pathUtf8;
        conv.Utf16ToUtf8(abs.c_str(),
            static_cast<int>(abs.wstring().length()),
            &pathUtf8);
        int w, h, n;
        stbi_uc* data = stbi_load(pathUtf8.c_str(), &w, &h, &n, 4);
        if (!data) {
            std::wcerr << L"[stb] load fail: " << abs.wstring() << L'\n';
            continue;
        }
        glCreateTextures(GL_TEXTURE_2D, 1, &mTextures[i]);
        glTextureStorage2D(mTextures[i], 1, GL_RGBA8, w, h);
        glTextureSubImage2D(mTextures[i], 0, 0, 0, w, h,
            GL_RGBA, GL_UNSIGNED_BYTE, data);
        glGenerateTextureMipmap(mTextures[i]);
        glTextureParameteri(mTextures[i],
            GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
        glTextureParameteri(mTextures[i],
            GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        stbi_image_free(data);
    }
}

/* ---------- uploadBones ---------- */
void PMXActor::uploadBones()const {
    glNamedBufferSubData(mUBOTransform, 0,
        mSkin.size() * sizeof(glm::mat4), mSkin.data());
}