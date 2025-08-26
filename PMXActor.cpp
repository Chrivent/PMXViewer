#include "PMXActor.h"
\
#include <filesystem>
#include <execution>
#include <algorithm>
#include <chrono>

#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtx/quaternion.hpp>
#include <glm/gtc/type_ptr.hpp>

// STB 구현은 '딱 한 곳'에서만: 여기서 구현하세요
#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"

#include "BoneNode.h"
#include "PhysicsManager.h"

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

    PhysicsManager::Destroy();
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
    _motion = vmd::VmdMotion::LoadFromFile(vmdPath.c_str());
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

    _morphManager.Init(
        _model.morphs.get(),                 // 포인터
        _model.morph_count,                  // 개수
        _motion->face_frames,                // 그대로 vector
        static_cast<unsigned>(_model.vertex_count),
        static_cast<unsigned>(_model.material_count),
        static_cast<unsigned>(_model.bone_count)
    );

    PhysicsManager::Create();
    InitPhysics();
    ResetPhysics();

    return true;
}

void PMXActor::InitPhysics()
{
    PhysicsManager::ActivePhysics(false);

    if (_model.rigid_body_count > 0 && _model.rigid_bodies) {
        for (int i = 0; i < _model.rigid_body_count; ++i) {
            const pmx::PmxRigidBody& pmxRigidBody = _model.rigid_bodies[i];

            RigidBody* rigidBody = new RigidBody();
            _rigidBodies.emplace_back(std::move(rigidBody));

            BoneNode* boneNode = nullptr;
            if (pmxRigidBody.target_bone != -1)
            {
                boneNode = _nodeManager.GetBoneNodeByIndex(pmxRigidBody.target_bone);
            }

            if (rigidBody->Init(pmxRigidBody, &_nodeManager, boneNode) == false)
            {
                OutputDebugStringA("Create Rigid Body Fail");
                continue;
            }
            PhysicsManager::AddRigidBody(rigidBody);
        }
    }

    if (_model.joint_count > 0 && _model.joints) {
        for (int i = 0; i < _model.joint_count; ++i) {
            const pmx::PmxJoint& pmxJoint = _model.joints[i];

            if (pmxJoint.param.rigid_body1 != -1 &&
                pmxJoint.param.rigid_body2 != -1 &&
                pmxJoint.param.rigid_body1 != pmxJoint.param.rigid_body2)
            {
                Joint* joint = new Joint();
                _joints.emplace_back(std::move(joint));

                RigidBody* rigidBodyA = nullptr;
                if (_rigidBodies.size() <= pmxJoint.param.rigid_body1)
                {
                    OutputDebugStringA("Create Joint Fail");
                    continue;
                }

                rigidBodyA = _rigidBodies[pmxJoint.param.rigid_body1].get();

                RigidBody* rigidBodyB = nullptr;
                if (_rigidBodies.size() <= pmxJoint.param.rigid_body2)
                {
                    OutputDebugStringA("Create Joint Fail");
                    continue;
                }

                rigidBodyB = _rigidBodies[pmxJoint.param.rigid_body2].get();

                if (rigidBodyA->_rigidBody->isStaticOrKinematicObject() == true &&
                    rigidBodyB->_rigidBody->isStaticOrKinematicObject() == true)
                {
                    int d = 0;
                }

                if (joint->CreateJoint(pmxJoint, rigidBodyA, rigidBodyB) == false)
                {
                    OutputDebugStringA("Create Joint Fail");
                    continue;
                }

                PhysicsManager::AddJoint(joint);
            }
        }
    }

    PhysicsManager::ActivePhysics(true);
}

void PMXActor::ResetPhysics()
{
    for (auto& rigidBody : _rigidBodies)
    {
        rigidBody->SetActive(false);
        rigidBody->ResetTransform();
    }

    for (auto& rigidBody : _rigidBodies)
    {
        rigidBody->ReflectGlobalTransform();
    }

    for (auto& rigidBody : _rigidBodies)
    {
        rigidBody->CalcLocalTransform();
    }

    const auto& nodes = _nodeManager._sortedNodes;
    for (const auto& node : nodes)
    {
        if (node->_parentBoneNode == nullptr)
        {
            node->UpdateGlobalTransform();
        }
    }

    btDiscreteDynamicsWorld* world = PhysicsManager::_world.get();
    for (auto& rigidBody : _rigidBodies)
    {
        rigidBody->Reset(world);
    }
}

void PMXActor::UpdatePhysicsAnimation()
{
    for (auto& rigidBody : _rigidBodies)
    {
        rigidBody->SetActive(true);
    }

    for (auto& rigidBody : _rigidBodies)
    {
        rigidBody->ReflectGlobalTransform();
    }

    for (auto& rigidBody : _rigidBodies)
    {
        rigidBody->CalcLocalTransform();
    }

    const auto& nodes = _nodeManager._sortedNodes;
    for (const auto& node : nodes)
    {
        if (node->_parentBoneNode == nullptr)
        {
            node->UpdateGlobalTransform();
        }
    }
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

    using clock = std::chrono::steady_clock;
    using dsec = std::chrono::duration<double>;
    auto t0 = clock::now();

    // --- 1) 애니메이션/모프 업데이트 (분리 계측, 호출 순서 유지) ---
    auto a_all_0 = clock::now();

    // 1-1) 애니메이션 프리 (본 트리 준비)
    auto a_pre_0 = clock::now();
    _nodeManager.BeforeUpdateAnimation();
    auto a_pre_1 = clock::now();

    // 1-2) 모프: Animate (표정/UV 등)
    auto mAnim0 = clock::now();
    _morphManager.Animate(frameTime30);
    auto mAnim1 = clock::now();

    // 1-3) 애니메이션 본 업데이트
    auto a_upd_0 = clock::now();
    _nodeManager.EvaluateAnimation(frameTime30);
    _nodeManager.UpdateAnimation();
    UpdatePhysicsAnimation();
    _nodeManager.UpdateAnimationAfterPhysics();
    auto a_upd_1 = clock::now();

    // 1-4) 모프: Material
    auto mMat0 = clock::now();
    MorphMaterial();
    auto mMat1 = clock::now();

    // 1-5) 모프: Bone
    auto mBone0 = clock::now();
    MorphBone();
    auto mBone1 = clock::now();

    auto a_all_1 = clock::now();

    // --- 2) 본 팔레트 갱신 ---
    auto b0 = clock::now();
    buildBonePalette();
    buildBonePaletteA34();
    auto b1 = clock::now();

    // --- 3) CPU 스키닝 ---
    auto s0 = clock::now();
    std::for_each(std::execution::par, _ranges.begin(), _ranges.end(),
        [this](const VertexRange& r) {
            if (!r.vertexCount) return;
            skinningByRange(_model, _bonePaletteA34, _vertices, r.startIndex, r.vertexCount);
        });
    auto s1 = clock::now();

    // --- 4) VBO 업데이트 ---
    auto u0 = clock::now();
    uploadVertexBufferFast();
    auto u1 = clock::now();

    _lastTex0 = _lastTex1 = _lastTex2 = -1;

    auto t1 = clock::now();

    // ================== 프로파일 누적 & 1초마다 출력 ==================
    static double accAnimPre = 0.0; // ★ BeforeUpdateAnimation
    static double accAnimUpd = 0.0; // ★ UpdateAnimation
    static double accAnimOnly = 0.0; // Before+Update 합계(기존)
    static double accMorphAnim = 0.0;
    static double accMorphMat = 0.0;
    static double accMorphBone = 0.0;
    static double accAnimAll = 0.0;
    static double accBone = 0.0;
    static double accSkin = 0.0;
    static double accUpload = 0.0;
    static double accTotal = 0.0;
    static int    frames = 0;
    static auto   lastPrint = clock::now();

    // 분리 집계
    const double dAnimPre = std::chrono::duration_cast<dsec>(a_pre_1 - a_pre_0).count();
    const double dAnimUpd = std::chrono::duration_cast<dsec>(a_upd_1 - a_upd_0).count();
    const double dAnimOnly = dAnimPre + dAnimUpd; // 합계(기존 지표)
    const double dMorphAnim = std::chrono::duration_cast<dsec>(mAnim1 - mAnim0).count();
    const double dMorphMat = std::chrono::duration_cast<dsec>(mMat1 - mMat0).count();
    const double dMorphBone = std::chrono::duration_cast<dsec>(mBone1 - mBone0).count();
    const double dAnimAll = std::chrono::duration_cast<dsec>(a_all_1 - a_all_0).count();
    const double dBonePal = std::chrono::duration_cast<dsec>(b1 - b0).count();
    const double dSkin = std::chrono::duration_cast<dsec>(s1 - s0).count();
    const double dUpload = std::chrono::duration_cast<dsec>(u1 - u0).count();
    const double dTotal = std::chrono::duration_cast<dsec>(t1 - t0).count();

    accAnimPre += dAnimPre;
    accAnimUpd += dAnimUpd;
    accAnimOnly += dAnimOnly;
    accMorphAnim += dMorphAnim;
    accMorphMat += dMorphMat;
    accMorphBone += dMorphBone;
    accAnimAll += dAnimAll;
    accBone += dBonePal;
    accSkin += dSkin;
    accUpload += dUpload;
    accTotal += dTotal;
    frames += 1;

    auto now = clock::now();
    if (std::chrono::duration_cast<dsec>(now - lastPrint).count() >= 1.0) {
        const double secs = std::chrono::duration_cast<dsec>(now - lastPrint).count();
        const double fps = frames / secs;
        auto avg_ms = [](double acc, int f) { return (acc / f) * 1000.0; };

        std::wcerr << std::fixed << std::setprecision(2)
            << L"[PMXActor] FPS: " << fps
            << L" | avg ms  anim(pre):" << avg_ms(accAnimPre, frames)   // ★ 추가
            << L"  anim(update):" << avg_ms(accAnimUpd, frames)   // ★ 추가
            << L"  anim(bones):" << avg_ms(accAnimOnly, frames)   // 합계(기존)
            << L"  morph(anim):" << avg_ms(accMorphAnim, frames)
            << L"  morph(material):" << avg_ms(accMorphMat, frames)
            << L"  morph(bone):" << avg_ms(accMorphBone, frames)
            << L"  anim+Morph:" << avg_ms(accAnimAll, frames)
            << L"  bonePal:" << avg_ms(accBone, frames)
            << L"  skin:" << avg_ms(accSkin, frames)
            << L"  vbo:" << avg_ms(accUpload, frames)
            << L"  total:" << avg_ms(accTotal, frames)
            << L" (" << frames << L" frames / " << secs << L"s)\n";

        accAnimPre = accAnimUpd = accAnimOnly =
            accMorphAnim = accMorphMat = accMorphBone =
            accAnimAll = accBone = accSkin = accUpload = accTotal = 0.0;
        frames = 0;
        lastPrint = now;
    }
}

void PMXActor::Draw(GLuint shader) {
    if (!_glReady) return;

    const auto& U = getUniforms(shader);
    glBindVertexArray(_vao);

    struct Batch {
        int matIndex;
        int indexOffset;
        int indexCount;
        bool transparent;
    };
    std::vector<Batch> batches;
    batches.reserve(_model.material_count);

    // 1) 머테리얼 → 인덱스 오프셋/카운트/투명 플래그로 변환
    int runningOffset = 0;
    for (int i = 0; i < _model.material_count; ++i) {
        const auto& m = _model.materials[i];
        const float alpha = (_matCache.empty() ? m.diffuse[3] : _matCache[i].diffuse.a);
        const bool isTransparent = (alpha < 0.999f);
        batches.push_back(Batch{ i, runningOffset, m.index_count, isTransparent });
        runningOffset += m.index_count;
    }

    // 2) 정렬: 불투명 먼저, 투명 나중 (내부 순서는 유지)
    auto mid = std::stable_partition(batches.begin(), batches.end(),
        [](const Batch& b) { return !b.transparent; });

    // (원하면 투명 그룹 내부를 추가 정렬 가능:
    // std::stable_sort(mid, batches.end(), [](const Batch& a, const Batch& b){
    //     // 예: 알파값 오름차순/내림차순 등 (정확한 투명 처리엔 카메라-기반 깊이 정렬이 필요)
    //     return aAlpha < bAlpha;
    // });

    // 텍스처 캐시 초기화
    _lastTex0 = _lastTex1 = _lastTex2 = -1;

    // 3) 드로우 루프 (상태 변경 없음: 정렬만 적용)
    for (const auto& b : batches) {
        const auto& mat = _model.materials[b.matIndex];
        const EffectiveMaterial& em = _matCache[b.matIndex];

        const int texIndex = mat.diffuse_texture_index;
        const int toonIndex = mat.toon_texture_index;
        const int sphereIndex = mat.sphere_texture_index;
        const int sphereMode = mat.sphere_op_mode;

        const bool hasDiffuse = (texIndex >= 0 && texIndex < (int)_textures.size() && _textures[texIndex]);
        const bool useToon = (toonIndex >= 0 && toonIndex < (int)_textures.size() && _textures[toonIndex]);
        const bool useSphere = (sphereIndex >= 0 && sphereIndex < (int)_textures.size() && _textures[sphereIndex]);

        GLint want0 = hasDiffuse ? (GLint)_textures[texIndex] : 0;
        GLint want1 = useToon ? (GLint)_textures[toonIndex] : 0;
        GLint want2 = useSphere ? (GLint)_textures[sphereIndex] : 0;

        if (_lastTex0 != want0) { glActiveTexture(GL_TEXTURE0); glBindTexture(GL_TEXTURE_2D, want0); _lastTex0 = want0; }
        if (_lastTex1 != want1) { glActiveTexture(GL_TEXTURE1); glBindTexture(GL_TEXTURE_2D, want1); _lastTex1 = want1; }
        if (_lastTex2 != want2) { glActiveTexture(GL_TEXTURE2); glBindTexture(GL_TEXTURE_2D, want2); _lastTex2 = want2; }

        glUniform1i(U.loc_bUseToon, useToon);
        glUniform1i(U.loc_bUseSphere, useSphere);
        glUniform1i(U.loc_sphereMode, useSphere ? sphereMode : 0);

        // 모프 반영된 머티리얼 값 세팅
        glUniform4fv(U.loc_diffuse, 1, glm::value_ptr(em.diffuse));
        glUniform3fv(U.loc_specular, 1, glm::value_ptr(em.specular));
        glUniform1f(U.loc_specularPower, em.specularPower);
        glUniform3fv(U.loc_ambient, 1, glm::value_ptr(em.ambient));
        glUniform1f(U.loc_Alpha, em.diffuse.a);

        glDrawElements(GL_TRIANGLES, b.indexCount, GL_UNSIGNED_INT,
            (void*)(uintptr_t)(b.indexOffset * sizeof(uint32_t)));
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

Affine34 PMXActor::toA34(const glm::mat4& M) {
    Affine34 A;
    const float* p = glm::value_ptr(M); // column-major
    // row 0
    A.m[0] = p[0];  A.m[1] = p[4];  A.m[2] = p[8];  A.m[3] = p[12];
    // row 1
    A.m[4] = p[1];  A.m[5] = p[5];  A.m[6] = p[9];  A.m[7] = p[13];
    // row 2
    A.m[8] = p[2];  A.m[9] = p[6];  A.m[10] = p[10]; A.m[11] = p[14];
    return A;
}

glm::vec3 PMXActor::transformPoint(const Affine34& A, const glm::vec3& v) {
    return {
        A.m[0] * v.x + A.m[1] * v.y + A.m[2] * v.z + A.m[3],
        A.m[4] * v.x + A.m[5] * v.y + A.m[6] * v.z + A.m[7],
        A.m[8] * v.x + A.m[9] * v.y + A.m[10] * v.z + A.m[11]
    };
}

glm::vec3 PMXActor::transformDir(const Affine34& A, const glm::vec3& v) {
    return {
        A.m[0] * v.x + A.m[1] * v.y + A.m[2] * v.z,
        A.m[4] * v.x + A.m[5] * v.y + A.m[6] * v.z,
        A.m[8] * v.x + A.m[9] * v.y + A.m[10] * v.z
    };
}

glm::vec3 PMXActor::fastNormalize(const glm::vec3& v) {
    float d = v.x * v.x + v.y * v.y + v.z * v.z;
    if (d <= 0.f) return glm::vec3(0, 0, 1);
    float inv = 1.0f / glm::sqrt(d); // /fp:fast or -ffast-math이면 더 빠름
    return { v.x * inv, v.y * inv, v.z * inv };
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

void PMXActor::buildBonePaletteA34()
{
    _bonePaletteA34.resize(_model.bone_count);
    for (int bi = 0; bi < _model.bone_count; ++bi) {
        const glm::mat4& M = (bi < (int)_bonePalette.size()) ? _bonePalette[bi] : glm::mat4(1.0f);
        _bonePaletteA34[bi] = toA34(M);
    }
}

void PMXActor::skinningByRange(
    const pmx::PmxModel& model,
    const std::vector<Affine34>& bonePaletteA34,
    std::vector<GLVertex>& out,
    unsigned start, unsigned count)
{
    const unsigned end = std::min<unsigned>(start + count, (unsigned)model.vertex_count);
    if (start >= end) return;

    const Affine34* __restrict BP34 = bonePaletteA34.data();
    const int bpSize = (int)bonePaletteA34.size();
    GLVertex* __restrict outBase = out.data();

    auto getA = [&](int idx) -> const Affine34& {
        static const Affine34 I = toA34(glm::mat4(1.0f));
        return (idx >= 0 && idx < bpSize) ? BP34[idx] : I;
        };

    for (int i = (int)start; i < (int)end; ++i)
    {
        const auto& v = model.vertices[i];

        // --- 원본 값 ---
        glm::vec3 basePos(v.position[0], v.position[1], v.position[2]);
        glm::vec3 nrm(v.normal[0], v.normal[1], v.normal[2]);
        glm::vec2 baseUv(v.uv[0], v.uv[1]);

        // --- 모프 델타 적용 (네 API 직접 사용) ---
        const glm::vec3 dpos = _morphManager.GetMorphVertexPosition((unsigned)i); // 위치 모프
        const glm::vec4 muv4 = _morphManager.GetMorphUV((unsigned)i);             // UV 모프(vec4) → xy 사용
        const glm::vec2 duv = glm::vec2(muv4.x, muv4.y);

        // 위치 모핑은 스키닝 "입력"에 더한다
        const glm::vec3 pos = basePos + dpos;

        GLVertex& dst = outBase[i];
        // UV 모핑은 스키닝과 무관하므로 결과에 더하기만 하면 됨
        dst.uv = baseUv + duv;

        switch (v.skinning_type)
        {
        case pmx::PmxVertexSkinningType::BDEF1:
        {
            const auto* b = static_cast<pmx::PmxVertexSkinningBDEF1*>(v.skinning.get());
            const Affine34& A = getA(b->bone_index);
            dst.position = transformPoint(A, pos);
            dst.normal = fastNormalize(transformDir(A, nrm));
            break;
        }
        case pmx::PmxVertexSkinningType::BDEF2:
        {
            const auto* b = static_cast<pmx::PmxVertexSkinningBDEF2*>(v.skinning.get());
            const float w0 = b->bone_weight, w1 = 1.0f - w0;

            const Affine34& A0 = getA(b->bone_index1);
            const Affine34& A1 = getA(b->bone_index2);

            const glm::vec3 p0 = transformPoint(A0, pos);
            const glm::vec3 p1 = transformPoint(A1, pos);
            const glm::vec3 n0 = transformDir(A0, nrm);
            const glm::vec3 n1 = transformDir(A1, nrm);

            dst.position = p0 * w0 + p1 * w1;
            dst.normal = fastNormalize(n0 * w0 + n1 * w1);
            break;
        }
        case pmx::PmxVertexSkinningType::BDEF4:
        {
            const auto* b = static_cast<pmx::PmxVertexSkinningBDEF4*>(v.skinning.get());
            const int   id[4] = { b->bone_index1, b->bone_index2, b->bone_index3, b->bone_index4 };
            const float w[4] = { b->bone_weight1, b->bone_weight2, b->bone_weight3, b->bone_weight4 };

            glm::vec3 pAcc(0.0f), nAcc(0.0f);
            for (int k = 0; k < 4; ++k) {
                if (w[k] == 0.0f) continue;
                const Affine34& Ak = getA(id[k]);
                pAcc += transformPoint(Ak, pos) * w[k];
                nAcc += transformDir(Ak, nrm) * w[k];
            }
            dst.position = pAcc;
            dst.normal = fastNormalize(nAcc);
            break;
        }
        case pmx::PmxVertexSkinningType::SDEF:
        {
            // SDEF/QDEF는 기존 구현 유지.
            const auto* s = static_cast<pmx::PmxVertexSkinningSDEF*>(v.skinning.get());
            const int i0 = s->bone_index1, i1 = s->bone_index2;

            const float w0 = s->bone_weight, w1 = 1.0f - w0;
            const glm::mat4& M0 = _bonePalette[i0];
            const glm::mat4& M1 = _bonePalette[i1];

            const glm::quat q0 = glm::quat_cast(M0);
            const glm::quat q1 = glm::quat_cast(M1);
            const glm::quat q = glm::slerp(q0, q1, w1);
            const glm::mat4 R = glm::toMat4(q);

            const glm::vec3 C(s->sdef_c[0], s->sdef_c[1], s->sdef_c[2]);
            const glm::vec3 R0(s->sdef_r0[0], s->sdef_r0[1], s->sdef_r0[2]);
            const glm::vec3 R1(s->sdef_r1[0], s->sdef_r1[1], s->sdef_r1[2]);

            const glm::vec3 rw = R0 * w0 + R1 * w1;
            const glm::vec3 r0 = C + R0 - rw;
            const glm::vec3 r1 = C + R1 - rw;
            const glm::vec3 cr0 = 0.5f * (C + r0);
            const glm::vec3 cr1 = 0.5f * (C + r1);

            // pos는 basePos + dpos (모핑 적용)임
            const glm::vec3 pOut = glm::vec3(R * glm::vec4(pos - C, 1.0f))
                + glm::vec3(M0 * glm::vec4(cr0, 1.0f)) * w0
                + glm::vec3(M1 * glm::vec4(cr1, 1.0f)) * w1;

            dst.position = pOut;
            dst.normal = fastNormalize(glm::mat3(R) * nrm);
            break;
        }
        case pmx::PmxVertexSkinningType::QDEF:
        {
            // 네가 이미 작성한 QDEF 구현 그대로. pos만 바뀐 점 유의.
            const auto* qd = static_cast<pmx::PmxVertexSkinningQDEF*>(v.skinning.get());
            int   ids[4] = { qd->bone_index1, qd->bone_index2, qd->bone_index3, qd->bone_index4 };
            float w[4] = { qd->bone_weight1, qd->bone_weight2, qd->bone_weight3, qd->bone_weight4 };

            float sum = w[0] + w[1] + w[2] + w[3];
            if (sum > 0.0f) { float inv = 1.0f / sum; for (int k = 0; k < 4; ++k) w[k] *= inv; }

            glm::quat accR(1, 0, 0, 0), accD(0, 0, 0, 0); bool has = false; glm::quat ref(1, 0, 0, 0);
            for (int k = 0; k < 4; ++k) {
                if (w[k] <= 0.0f) continue;
                int id = ids[k]; if (id < 0 || id >= (int)_bonePalette.size()) continue;
                const glm::mat4& M = _bonePalette[id];
                glm::quat qr = glm::normalize(glm::quat_cast(M));
                const glm::vec3 t(M[3]);
                glm::quat dq = 0.5f * (glm::quat(0, t.x, t.y, t.z) * qr);
                if (!has) { ref = qr; has = true; }
                else if (glm::dot(qr, ref) < 0) { qr = -qr; dq = -dq; }
                accR += qr * w[k];
                accD += dq * w[k];
            }
            float len = glm::length(accR);
            if (len < 1e-8f) {
                const glm::mat4& M = (ids[0] >= 0 && ids[0] < (int)_bonePalette.size()) ? _bonePalette[ids[0]] : glm::mat4(1);
                dst.position = glm::vec3(M * glm::vec4(pos, 1.0f));        // pos 사용
                dst.normal = fastNormalize(glm::mat3(M) * nrm);
                break;
            }
            accR /= len; accD /= len;
            glm::quat tq = accD * glm::conjugate(accR) * 2.0f;
            const glm::vec3 tf(tq.x, tq.y, tq.z);
            glm::mat4 R = glm::mat4_cast(accR);
            R[3] = glm::vec4(tf, 1.0f);
            dst.position = glm::vec3(R * glm::vec4(pos, 1.0f));            // pos 사용
            dst.normal = fastNormalize(glm::mat3_cast(accR) * nrm);
            break;
        }
        default:
            dst.position = pos;                    // 모핑만 적용
            dst.normal = fastNormalize(nrm);
            break;
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
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glGenerateMipmap(GL_TEXTURE_2D);
    stbi_image_free(data);
    return tex;
}

// PMXActor.cpp
static glm::vec3 mixMul(const glm::vec3& base, const glm::vec3& factor, float w) {
    // base * lerp(1, factor, w)
    return base * (glm::vec3(1.0f) + (factor - glm::vec3(1.0f)) * w);
}
static glm::vec4 mixMul4(const glm::vec4& base, const glm::vec4& factor, float w) {
    return base * (glm::vec4(1.0f) + (factor - glm::vec4(1.0f)) * w);
}
static float mixMul1(float base, float factor, float w) {
    return base * (1.0f + (factor - 1.0f) * w);
}

void PMXActor::MorphMaterial()
{
    // 모프 결과 캐시 크기 보정
    if (_matCache.size() != static_cast<size_t>(_model.material_count))
        _matCache.resize(_model.material_count);

    auto lerp = [](float a, float b, float t) { return a + (b - a) * t; };
    auto lerp3 = [&](const glm::vec3& a, const glm::vec3& b, float t) {
        return glm::vec3(lerp(a.x, b.x, t), lerp(a.y, b.y, t), lerp(a.z, b.z, t));
        };
    auto lerp4 = [&](const glm::vec4& a, const glm::vec4& b, float t) {
        return glm::vec4(lerp(a.x, b.x, t), lerp(a.y, b.y, t), lerp(a.z, b.z, t), lerp(a.w, b.w, t));
        };

    for (int i = 0; i < _model.material_count; ++i)
    {
        const pmx::PmxMaterial& base = _model.materials[i];

        // PMX 기본값을 glm으로 변환
        glm::vec4 baseDiffuse(base.diffuse[0], base.diffuse[1], base.diffuse[2], base.diffuse[3]);
        glm::vec3 baseSpecular(base.specular[0], base.specular[1], base.specular[2]);
        float     baseSpecPow = base.specularlity;
        glm::vec3 baseAmbient(base.ambient[0], base.ambient[1], base.ambient[2]);
        glm::vec4 baseEdgeCol(base.edge_color[0], base.edge_color[1], base.edge_color[2], base.edge_color[3]);
        float     baseEdgeSz = base.edge_size;

        // 텍스처 계수(알베도/스피어/툰)는 기본 1
        const glm::vec4 one4(1.0f);
        const glm::vec4 zero4(0.0f);

        // 모프 값 가져오기 (사용자 정의 구조)
        const MaterialMorphData& mm = _morphManager.GetMorphMaterial(i);
        const float w = mm.weight;
        const uint8_t op = mm.opType; // 0: Add, 1: Mul (가정)

        EffectiveMaterial eff{};
        eff.opType = op;

        if (w <= 0.0f) {
            // 모프가 없으면 원본 그대로
            eff.diffuse = baseDiffuse;
            eff.specular = baseSpecular;
            eff.specularPower = baseSpecPow;
            eff.ambient = baseAmbient;
            eff.edgeColor = baseEdgeCol;
            eff.edgeSize = baseEdgeSz;
            eff.texFactor = one4;
            eff.sphereFactor = one4;
            eff.toonFactor = one4;
            _matCache[i] = eff;
            continue;
        }

        if (op == 0 /* Add */)
        {
            eff.diffuse = baseDiffuse + mm.diffuse * w;
            eff.specular = baseSpecular + mm.specular * w;
            eff.specularPower = baseSpecPow + mm.specularPower * w;
            eff.ambient = baseAmbient + mm.ambient * w;
            eff.edgeColor = baseEdgeCol + mm.edgeColor * w;
            eff.edgeSize = baseEdgeSz + mm.edgeSize * w;

            // 텍스처 계수(Add 모드) : 보통 (1 + delta*w) 형태로 두고 셰이더에서 곱해줌
            eff.texFactor = one4 + mm.textureFactor * w;
            eff.sphereFactor = one4 + mm.sphereTextureFactor * w;
            eff.toonFactor = one4 + mm.toonTextureFactor * w;
        }
        else /* Multiply */
        {
            // 색/스칼라: base * lerp(1, morph, w)
            eff.diffuse = baseDiffuse * lerp4(glm::vec4(1), mm.diffuse, w);
            eff.specular = baseSpecular * lerp3(glm::vec3(1), mm.specular, w);
            eff.specularPower = baseSpecPow * lerp(1.0f, mm.specularPower, w);
            eff.ambient = baseAmbient * lerp3(glm::vec3(1), mm.ambient, w);
            eff.edgeColor = baseEdgeCol * lerp4(glm::vec4(1), mm.edgeColor, w);
            eff.edgeSize = baseEdgeSz * lerp(1.0f, mm.edgeSize, w);

            // 텍스처 계수(Mul 모드) : 계수 자체를 lerp(1, morphFactor, w)로 만들어서 곱셈에 쓰기
            eff.texFactor = lerp4(one4, mm.textureFactor, w);
            eff.sphereFactor = lerp4(one4, mm.sphereTextureFactor, w);
            eff.toonFactor = lerp4(one4, mm.toonTextureFactor, w);
        }

        // (선택) 기본적인 클램프 – 알파/색이 음수로 가는 걸 방지하고 싶다면 사용
        auto clamp01 = [](float v) { return glm::clamp(v, 0.0f, 1.0f); };
        eff.diffuse = glm::vec4(clamp01(eff.diffuse.r),
            clamp01(eff.diffuse.g),
            clamp01(eff.diffuse.b),
            clamp01(eff.diffuse.a));
        eff.specular = glm::max(eff.specular, glm::vec3(0.0f));
        eff.ambient = glm::max(eff.ambient, glm::vec3(0.0f));
        eff.edgeColor = glm::vec4(glm::max(eff.edgeColor.r, 0.0f),
            glm::max(eff.edgeColor.g, 0.0f),
            glm::max(eff.edgeColor.b, 0.0f),
            glm::clamp(eff.edgeColor.a, 0.0f, 1.0f));
        eff.edgeSize = glm::max(eff.edgeSize, 0.0f);

        _matCache[i] = eff;
    }
}

void PMXActor::MorphBone()
{
    const auto& nodes = _nodeManager._sortedNodes;

    for (BoneNode* bn : nodes)
    {
        const int bi = bn->_boneIndex;

        const BoneMorphData md = _morphManager.GetMorphBone(bi);
        const float w = md.weight;

        // 1) 위치: 0 -> md.position 선형보간(추가 오프셋)
        bn->_morphPosition = glm::mix(glm::vec3(0.0f), md.position, w);

        // 2) 회전: 절대 모프 해석이면 (지금처럼) animR <-> morphQ slerp
        glm::quat animR = bn->_animateRotation;          // 이미 quat
        glm::quat morphQ = glm::normalize(md.quaternion); // 방어적 정규화

        glm::quat blendedQ = glm::slerp(animR, morphQ, w);
        bn->_animateRotation = glm::normalize(blendedQ);
    }
}
