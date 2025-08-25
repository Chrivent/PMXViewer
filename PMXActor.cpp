#include "PMXActor.h"
\
#include <filesystem>
#include <execution>
#include <algorithm>

#include <chrono>
#include <iomanip>

#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtx/quaternion.hpp>

// ★ STB 구현은 '딱 한 곳'에서만: 여기서 구현하세요
#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"

#include "BoneNode.h"

struct MaterialUniforms {
    GLint loc_bUseToon, loc_bUseSphere, loc_sphereMode;
    GLint loc_diffuse, loc_specular, loc_specularPower;
    GLint loc_ambient, loc_Alpha;
};
static std::unordered_map<GLuint, MaterialUniforms> sCache;

static const MaterialUniforms& getUniforms(GLuint prog) {
    auto it = sCache.find(prog);
    if (it != sCache.end()) return it->second;
    MaterialUniforms u{
        glGetUniformLocation(prog, "bUseToon"),
        glGetUniformLocation(prog, "bUseSphere"),
        glGetUniformLocation(prog, "sphereMode"),
        glGetUniformLocation(prog, "diffuse"),
        glGetUniformLocation(prog, "specular"),
        glGetUniformLocation(prog, "specularPower"),
        glGetUniformLocation(prog, "ambient"),
        glGetUniformLocation(prog, "Alpha"),
    };
    return sCache.emplace(prog, u).first->second;
}

PMXActor::~PMXActor() {
    if (_ebo) glDeleteBuffers(1, &_ebo);
    if (_vbo) glDeleteBuffers(1, &_vbo);
    if (_vao) glDeleteVertexArrays(1, &_vao);
    for (auto t : _textures) if (t) glDeleteTextures(1, &t);
}

bool PMXActor::LoadModel(const std::wstring& pmxPath) {
    std::ifstream file(pmxPath, std::ios::binary);
    if (!file) return false;
    _model.Read(&file);

    // 노드 매니저 세팅
    _nodeManager.Init(_model.bones, _model.bone_count);

    // CPU 버텍스/인덱스 준비
    buildInitialVertices();
    _indices.assign(_model.indices.get(), _model.indices.get() + _model.index_count);

    // 텍스처
    std::string base = std::filesystem::path(pmxPath).parent_path().string();
    (void)loadAllTextures(base); // 실패해도 하드 실패 X

    // 스키닝 파티션
    buildSkinningPartitions(_vertices.size());
    return true;
}

bool PMXActor::LoadMotion(const std::wstring& vmdPath) {
#ifdef _WIN32
    // Vmd.h가 Windows에서 wide 경로를 받는 오버로드가 있다면 이게 정답
    _motion = vmd::VmdMotion::LoadFromFile(vmdPath.c_str());
#else
    // 리눅스/맥: UTF-8로 변환해 좁은 문자열 오버로드 호출
    std::string u8 = std::filesystem::path(vmdPath).u8string();
    _motion = vmd::VmdMotion::LoadFromFile(u8.c_str());
#endif
    if (!_motion) return false;

    // VMD bone frames -> NodeManager
    oguna::EncodingConverter conv;
    for (auto& boneFrame : _motion->bone_frames) {
        std::wstring name;
        conv.Cp932ToUtf16(boneFrame.name.c_str(),
            (int)boneFrame.name.size(), &name);
        if (auto boneNode = _nodeManager.GetBoneNodeByName(name)) {
            boneNode->_motionKeys.emplace_back(boneFrame);
        }
    }
    _nodeManager.SortKey();
    return true;
}

bool PMXActor::InitGL() {
    glGenVertexArrays(1, &_vao);
    glGenBuffers(1, &_vbo);
    glGenBuffers(1, &_ebo);

    glBindVertexArray(_vao);

    glBindBuffer(GL_ARRAY_BUFFER, _vbo);
    glBufferData(GL_ARRAY_BUFFER, _vertices.size() * sizeof(GLVertex),
        _vertices.data(), GL_DYNAMIC_DRAW);

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, _ebo);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, _indices.size() * sizeof(uint32_t),
        _indices.data(), GL_STATIC_DRAW);

    // 정점 속성
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(GLVertex), (void*)offsetof(GLVertex, position));

    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(GLVertex), (void*)offsetof(GLVertex, normal));

    glEnableVertexAttribArray(2);
    glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, sizeof(GLVertex), (void*)offsetof(GLVertex, uv));

    glBindVertexArray(0);

    _glReady = true;
    return true;
}

void PMXActor::SetModelScale(const glm::vec3& s) {
    _modelScale = glm::scale(glm::mat4(1.0f), s);
}

// --- per-frame ---

void PMXActor::Update(float frameTime30) {
    if (!_glReady) return;

    // 1) 애니메이션 업데이트
    _nodeManager.UpdateAnimation(frameTime30);

    // 2) 본 팔레트 갱신
    buildBonePalette();

    // 3) CPU 스키닝 (표준 병렬 알고리즘 사용: 공용 스레드풀 활용)
    std::for_each(std::execution::par, _ranges.begin(), _ranges.end(),
        [this](const VertexRange& r) {
            if (!r.vertexCount) return;
            skinningByRange(_model, _bonePalette, _vertices, r.startIndex, r.vertexCount);
        });

    // 4) VBO 업데이트 (맵/언맵 + INVALIDATE + UNSYNC)
    uploadVertexBufferFast();

    // 텍스처 바인딩 캐시 초기화
    _lastTex0 = _lastTex1 = _lastTex2 = -1;
}

void PMXActor::Draw(GLuint shader) {
    if (!_glReady) return;

    const auto& U = getUniforms(shader);
    glBindVertexArray(_vao);

    int indexOffset = 0;

    for (int i = 0; i < _model.material_count; ++i) {
        const auto& mat = _model.materials[i];
        const int idxCount = mat.index_count;

        const int texIndex = mat.diffuse_texture_index;
        const int toonIndex = mat.toon_texture_index;
        const int sphereIndex = mat.sphere_texture_index;
        const int sphereMode = mat.sphere_op_mode;

        const bool hasDiffuse = (texIndex >= 0 && texIndex < (int)_textures.size() && _textures[texIndex]);
        const bool useToon = (toonIndex >= 0 && toonIndex < (int)_textures.size() && _textures[toonIndex]);
        const bool useSphere = (sphereIndex >= 0 && sphereIndex < (int)_textures.size() && _textures[sphereIndex]);

        // 바인딩 최소화 (상태 캐시)
        GLint want0 = hasDiffuse ? (GLint)_textures[texIndex] : 0;
        GLint want1 = useToon ? (GLint)_textures[toonIndex] : 0;
        GLint want2 = useSphere ? (GLint)_textures[sphereIndex] : 0;

        if (_lastTex0 != want0) { glActiveTexture(GL_TEXTURE0); glBindTexture(GL_TEXTURE_2D, want0); _lastTex0 = want0; }
        if (_lastTex1 != want1) { glActiveTexture(GL_TEXTURE1); glBindTexture(GL_TEXTURE_2D, want1); _lastTex1 = want1; }
        if (_lastTex2 != want2) { glActiveTexture(GL_TEXTURE2); glBindTexture(GL_TEXTURE_2D, want2); _lastTex2 = want2; }

        glUniform1i(U.loc_bUseToon, useToon);
        glUniform1i(U.loc_bUseSphere, useSphere);
        glUniform1i(U.loc_sphereMode, useSphere ? sphereMode : 0);

        glUniform4fv(U.loc_diffuse, 1, mat.diffuse);
        glUniform3fv(U.loc_specular, 1, mat.specular);
        glUniform1f(U.loc_specularPower, mat.specularlity);
        glUniform3fv(U.loc_ambient, 1, mat.ambient);
        glUniform1f(U.loc_Alpha, mat.diffuse[3]);

        glDrawElements(GL_TRIANGLES, idxCount, GL_UNSIGNED_INT,
            (void*)(uintptr_t)(indexOffset * sizeof(uint32_t)));
        indexOffset += idxCount;
    }

    glBindVertexArray(0);
}

// --- helpers ---

void PMXActor::buildInitialVertices() {
    _vertices.clear();
    _vertices.resize(_model.vertex_count);
    for (int i = 0; i < _model.vertex_count; ++i) {
        const auto& src = _model.vertices[i];
        GLVertex v{};
        v.position = { src.position[0], src.position[1], src.position[2] };
        v.normal = { src.normal[0],   src.normal[1],   src.normal[2] };
        v.uv = { src.uv[0],       src.uv[1] };
        _vertices[i] = v;
    }
}

void PMXActor::buildSkinningPartitions(size_t vertexCount) {
    unsigned hw = (std::max)(1u, std::thread::hardware_concurrency());
    unsigned threadCount = (std::min)(8u, (std::max)(2u, hw));
    threadCount = (unsigned)std::min<size_t>(threadCount, std::max<size_t>(1, vertexCount));
    _ranges.resize(threadCount);

    unsigned div = (unsigned)vertexCount / threadCount;
    unsigned rem = (unsigned)vertexCount % threadCount;
    unsigned start = 0;
    for (unsigned i = 0; i < threadCount; ++i) {
        unsigned cnt = div + (i < rem ? 1u : 0u);
        _ranges[i] = { start, cnt };
        start += cnt;
    }
}

void PMXActor::buildBonePalette() {
    _bonePalette.resize(_model.bone_count);
    for (int bi = 0; bi < _model.bone_count; ++bi) {
        if (auto* bn = _nodeManager.GetBoneNodeByIndex(bi)) {
            _bonePalette[bi] = bn->_globalTransform * bn->_inverseInitTransform;
        }
        else {
            _bonePalette[bi] = glm::mat4(1.0f);
        }
    }
}

void PMXActor::skinningByRange(
    const pmx::PmxModel& model,
    const std::vector<glm::mat4>& bonePalette,
    std::vector<GLVertex>& out,
    unsigned start, unsigned count)
{
    const unsigned end = std::min<unsigned>(start + count, (unsigned)model.vertex_count);
    for (unsigned i = start; i < end; ++i) {
        const auto& v = model.vertices[i];
        glm::vec4 pos(v.position[0], v.position[1], v.position[2], 1.f);
        glm::vec3 nrm(v.normal[0], v.normal[1], v.normal[2]);

        glm::mat4 blended(0.0f);
        bool useBlended = true;

        switch (v.skinning_type) {
        case pmx::PmxVertexSkinningType::BDEF1: {
            auto* b = static_cast<pmx::PmxVertexSkinningBDEF1*>(v.skinning.get());
            blended = (b->bone_index >= 0 && b->bone_index < (int)bonePalette.size())
                ? bonePalette[b->bone_index] : glm::mat4(1.0f);
            break;
        }
        case pmx::PmxVertexSkinningType::BDEF2: {
            auto* b = static_cast<pmx::PmxVertexSkinningBDEF2*>(v.skinning.get());
            float w0 = b->bone_weight, w1 = 1.f - w0;
            glm::mat4 m0 = (b->bone_index1 >= 0 && b->bone_index1 < (int)bonePalette.size()) ? bonePalette[b->bone_index1] : glm::mat4(1);
            glm::mat4 m1 = (b->bone_index2 >= 0 && b->bone_index2 < (int)bonePalette.size()) ? bonePalette[b->bone_index2] : glm::mat4(1);
            blended = m0 * w0 + m1 * w1;
            break;
        }
        case pmx::PmxVertexSkinningType::BDEF4: {
            auto* b = static_cast<pmx::PmxVertexSkinningBDEF4*>(v.skinning.get());
            const int id[4] = { b->bone_index1,b->bone_index2,b->bone_index3,b->bone_index4 };
            const float w[4] = { b->bone_weight1,b->bone_weight2,b->bone_weight3,b->bone_weight4 };
            blended = glm::mat4(0.0f);
            for (int k = 0; k < 4; ++k) {
                if (w[k] == 0.0f || id[k] < 0 || id[k] >= (int)bonePalette.size()) continue;
                blended += bonePalette[id[k]] * w[k];
            }
            break;
        }
        case pmx::PmxVertexSkinningType::SDEF: {
            useBlended = false;
            auto* s = static_cast<pmx::PmxVertexSkinningSDEF*>(v.skinning.get());
            float w0 = s->bone_weight, w1 = 1.f - w0;
            if (s->bone_index1 < 0 || s->bone_index2 < 0 ||
                s->bone_index1 >= (int)bonePalette.size() || s->bone_index2 >= (int)bonePalette.size()) break;
            glm::mat4 M0 = bonePalette[s->bone_index1];
            glm::mat4 M1 = bonePalette[s->bone_index2];
            glm::quat q0 = glm::quat_cast(M0), q1 = glm::quat_cast(M1);
            glm::quat q = glm::slerp(q0, q1, w1);
            glm::mat4 R = glm::toMat4(q);
            glm::vec3 C(s->sdef_c[0], s->sdef_c[1], s->sdef_c[2]);
            glm::vec3 R0(s->sdef_r0[0], s->sdef_r0[1], s->sdef_r0[2]);
            glm::vec3 R1(s->sdef_r1[0], s->sdef_r1[1], s->sdef_r1[2]);
            glm::vec3 rw = R0 * w0 + R1 * w1;
            glm::vec3 r0 = C + R0 - rw;
            glm::vec3 r1 = C + R1 - rw;
            glm::vec3 cr0 = 0.5f * (C + r0);
            glm::vec3 cr1 = 0.5f * (C + r1);
            glm::vec3 pOut = glm::vec3(R * glm::vec4(glm::vec3(pos) - C, 1.0f))
                + glm::vec3(M0 * glm::vec4(cr0, 1.0f)) * w0
                + glm::vec3(M1 * glm::vec4(cr1, 1.0f)) * w1;
            glm::vec3 nOut = glm::mat3(R) * nrm;
            out[i].position = pOut;
            out[i].normal = glm::normalize(nOut);
            out[i].uv = { v.uv[0], v.uv[1] };
            continue;
        }
        case pmx::PmxVertexSkinningType::QDEF: {
            auto* qd = static_cast<pmx::PmxVertexSkinningQDEF*>(v.skinning.get());
            const int ids[4] = { qd->bone_index1,qd->bone_index2,qd->bone_index3,qd->bone_index4 };
            float w[4] = { qd->bone_weight1,qd->bone_weight2,qd->bone_weight3,qd->bone_weight4 };
            float sum = w[0] + w[1] + w[2] + w[3]; if (sum > 0) { float inv = 1.f / sum; for (int k = 0; k < 4; ++k) w[k] *= inv; }
            glm::quat accR(1, 0, 0, 0), accD(0, 0, 0, 0); bool has = false; glm::quat ref(1, 0, 0, 0);
            for (int k = 0; k < 4; ++k) {
                if (w[k] <= 0 || ids[k] < 0 || ids[k] >= (int)bonePalette.size()) continue;
                glm::mat4 M = bonePalette[ids[k]];
                glm::quat qr = glm::normalize(glm::quat_cast(M));
                glm::vec3 t(M[3]);
                glm::quat dq = 0.5f * (glm::quat(0, t.x, t.y, t.z) * qr);
                if (!has) { ref = qr; has = true; }
                else if (glm::dot(qr, ref) < 0) { qr = -qr; dq = -dq; }
                accR += qr * w[k]; accD += dq * w[k];
            }
            float len = glm::length(accR);
            if (len < 1e-8f) {
                int b0 = ids[0];
                if (b0 >= 0 && b0 < (int)bonePalette.size()) {
                    glm::mat4 M = bonePalette[b0];
                    out[i].position = glm::vec3(M * pos);
                    out[i].normal = glm::normalize(glm::mat3(M) * nrm);
                    out[i].uv = { v.uv[0], v.uv[1] };
                }
                continue;
            }
            accR /= len; accD /= len;
            glm::quat tq = accD * glm::conjugate(accR) * 2.0f;
            glm::vec3 tf(tq.x, tq.y, tq.z);
            glm::mat4 R = glm::mat4_cast(accR); R[3] = glm::vec4(tf, 1.0f);
            out[i].position = glm::vec3(R * pos);
            out[i].normal = glm::normalize(glm::mat3_cast(accR) * nrm);
            out[i].uv = { v.uv[0], v.uv[1] };
            continue;
        }
        }

        if (useBlended) {
            glm::vec3 pOut = glm::vec3(blended * pos);
            glm::vec3 nOut = glm::normalize(glm::mat3(blended) * nrm);
            out[i].position = pOut;
            out[i].normal = nOut;
            out[i].uv = { v.uv[0], v.uv[1] };
        }
    }
}

// 버텍스 버퍼 업로드 최적화:
// glMapBufferRange(GL_MAP_WRITE_BIT | GL_MAP_INVALIDATE_BUFFER_BIT | GL_MAP_UNSYNCHRONIZED_BIT)
// 로 드라이버 동기화 비용을 피함.
void PMXActor::uploadVertexBufferFast() {
    const GLsizeiptr bytes = (GLsizeiptr)(_vertices.size() * sizeof(GLVertex));
    glBindBuffer(GL_ARRAY_BUFFER, _vbo);

    // orphan + map(unsync) 조합이 가장 호환성 좋고 빠른 편
    glBufferData(GL_ARRAY_BUFFER, bytes, nullptr, GL_DYNAMIC_DRAW); // orphan
    void* p = glMapBufferRange(GL_ARRAY_BUFFER, 0, bytes,
        GL_MAP_WRITE_BIT | GL_MAP_INVALIDATE_BUFFER_BIT | GL_MAP_UNSYNCHRONIZED_BIT);
    if (p) {
        std::memcpy(p, _vertices.data(), (size_t)bytes);
        glUnmapBuffer(GL_ARRAY_BUFFER);
    }
    else {
        // fallback
        glBufferSubData(GL_ARRAY_BUFFER, 0, bytes, _vertices.data());
    }
}

bool PMXActor::loadAllTextures(const std::string& base) {
    _textures.resize(_model.texture_count, 0);
    for (int i = 0; i < _model.texture_count; ++i) {
        const std::wstring& rel = _model.textures[i];
        auto path = std::filesystem::path(base) / rel;
        _textures[i] = createTextureFromFile(path);
    }
    return true;
}

GLuint PMXActor::createTextureFromFile(const std::filesystem::path& path) {
    FILE* fp = _wfopen(path.c_str(), L"rb");
    if (!fp) return 0;
    int w = 0, h = 0, ch = 0;
    stbi_uc* data = stbi_load_from_file(fp, &w, &h, &ch, STBI_rgb_alpha);
    fclose(fp);
    if (!data) return 0;
    GLuint tex = 0;
    glGenTextures(1, &tex);
    glBindTexture(GL_TEXTURE_2D, tex);
    // sRGB 질감일 경우(확장자 기반 추정) GL_SRGB8_ALPHA8 사용해도 좋음. 여기선 일반 RGBA.
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, w, h, 0, GL_RGBA, GL_UNSIGNED_BYTE, data);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glGenerateMipmap(GL_TEXTURE_2D);
    stbi_image_free(data);
    return tex;
}
