#pragma once

#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <iostream>
#include <fstream>
#include <filesystem>
#include <unordered_map>
#include <fcntl.h>
#include <io.h>

#include "Pmx.h"
#include "Vmd.h"

#ifndef GLM_ENABLE_EXPERIMENTAL        // ① 아직 안 켜졌다면
#define GLM_ENABLE_EXPERIMENTAL
#endif

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtx/quaternion.hpp>
#include <glm/gtx/string_cast.hpp>

using namespace std;

enum class SolveAxis
{
    X,
    Y,
    Z
};

struct IKChain
{
    int        boneIndex = -1;           // 링크가 가리키는 본 인덱스
    bool       enableAxisLimit = false;

    glm::vec3  limitMin{};
    glm::vec3  limitMax{};

    glm::vec3  prevAngle{};                    // Euler 누적각
    glm::quat  saveIKRotation{ 1,0,0,0 };        // 롤백용 (Identity)
    float      planeModeAngle = 0.0f;

    IKChain(int idx,
        bool axisLimit,
        const glm::vec3& mn,
        const glm::vec3& mx)
        : boneIndex(idx)
        , enableAxisLimit(axisLimit)
        , limitMin(mn)
        , limitMax(mx)
    {
    }
};

class IKSolver
{
public:
    /* ikIndex = 루트(회전 적용) 본,  targetIndex = 목표 본 */
    IKSolver(int ikIndex, int targetIndex,
        unsigned iterationCount, float limitAngleRad)
        : _ikIndex(ikIndex)
        , _targetIndex(targetIndex)
        , _ikIterationCount(iterationCount)
        , _ikLimitAngle(limitAngleRad) {
    }

    /* 외부 호출: PMX 메타 + 런타임 행렬 버퍼 전달 */
    void Solve(const pmx::PmxBone& ikBone,
        const pmx::PmxBone* bones,
        int                  boneCount,
        std::vector<glm::mat4>& localMats);

    /* 체인 링크 추가 */
    void AddIKChain(int linkIndex, bool axisLimit,
        glm::vec3& limitMin,
        glm::vec3& limitMax)
    {
        for (int a = 0; a < 3; ++a)
            if (limitMin[a] > limitMax[a]) std::swap(limitMin[a], limitMax[a]);

        _ikChains.emplace_back(linkIndex, axisLimit, limitMin, limitMax);
    }

    /* 단순 Getter */
    bool        IsEnabled()        const { return _enable; }
    void        SetEnabled(bool e) { _enable = e; }
    int         GetIKIndex()       const { return _ikIndex; }
    int         GetTargetIndex()   const { return _targetIndex; }
    unsigned    GetIterationCount()const { return _ikIterationCount; }
    float       GetLimitAngle()    const { return _ikLimitAngle; }
    const std::vector<IKChain>& GetIKChains() const { return _ikChains; }

private:
    /* 내부: CCD 한 바퀴, Plane 축 전용, 유틸 */
    void SolveCore(unsigned iter,
        const pmx::PmxBone& ikBone,
        const pmx::PmxBone* bones,
        std::vector<glm::mat4>& local,
        std::vector<glm::mat4>& world);

    void SolvePlane(unsigned iter, std::size_t chainIdx, SolveAxis axis,
        const pmx::PmxBone* bones,
        std::vector<glm::mat4>& local,
        std::vector<glm::mat4>& world);

    static glm::vec3 DecomposeEuler(const glm::mat4& m, const glm::vec3& prev);
    static float     NormalizeAngle(float a);           // 0‥2π
    static float     DiffAngle(float a, float b);       // (-π, π]

private:
    /* 상태 */
    bool                      _enable = true;
    int                       _ikIndex = -1;  // 루트 본 인덱스
    int                       _targetIndex = -1;  // 목표 본 인덱스
    unsigned                  _ikIterationCount = 15;
    float                     _ikLimitAngle = glm::radians(4.0f);

    std::vector<IKChain>      _ikChains;                // 링크 목록
};

void IKSolver::Solve(const pmx::PmxBone& ikBone,
    const pmx::PmxBone* bones,
    int                  boneCount,
    std::vector<glm::mat4>& localMats)
{
    if (ikBone.ik_link_count == 0) return;

    /* 0) IK 델타 초기화 ---------------------------------------- */
    auto Reset = [&](int idx)
        {
            glm::vec3 t = glm::vec3(localMats[idx][3]);
            glm::quat qa = glm::quat_cast(localMats[idx]);       // 애니 회전
            localMats[idx] = glm::translate(glm::mat4(1), t) *
                glm::toMat4(qa);                    // IK = I
        };
    int rootIdx = int(&ikBone - bones);
    Reset(rootIdx);
    for (int l = 0; l < ikBone.ik_link_count; ++l)
        Reset(ikBone.ik_links[l].link_target);

    /* 1) 월드 행렬 첫 계산 -------------------------------------- */
    std::vector<glm::mat4> world(boneCount);
    auto UpdateWorld = [&]()
        {
            for (int i = 0; i < boneCount; ++i) {
                int p = bones[i].parent_index;
                world[i] = (p >= 0) ? world[p] * localMats[i]
                    : localMats[i];
            }
        };
    UpdateWorld();

    /* 2) CCD 반복 ---------------------------------------------- */
    float bestDist = std::numeric_limits<float>::max();
    int   targetIdx = ikBone.ik_target_bone_index;

    for (unsigned iter = 0; iter < _ikIterationCount; ++iter)
    {
        SolveCore(iter, ikBone, bones, localMats, world);    // ← 링크 회전

        float dist = glm::length(glm::vec3(world[targetIdx][3]) -
            glm::vec3(world[rootIdx][3]));

        if (dist < bestDist) {                               // 개선!
            bestDist = dist;
            for (auto& c : _ikChains)
                c.saveIKRotation = glm::quat_cast(localMats[c.boneIndex]);
        }
        else {                                             // 악화 → 롤백
            for (auto& c : _ikChains) {
                glm::vec3 t = glm::vec3(localMats[c.boneIndex][3]);
                localMats[c.boneIndex] =
                    glm::translate(glm::mat4(1), t) *
                    glm::toMat4(c.saveIKRotation);
            }
            UpdateWorld();
            break;
        }
    }
}

void IKSolver::SolveCore(unsigned                      iter,
    const pmx::PmxBone& ikBone,
    const pmx::PmxBone* bones,
    std::vector<glm::mat4>& local,
    std::vector<glm::mat4>& world)
{
    const glm::vec3 rootPos = glm::vec3(world[_ikIndex][3]);
    const glm::vec3 tgtPos = glm::vec3(world[_targetIndex][3]);

    /* 체인 링크를 루트 → Effector 순으로 처리 */
    for (std::size_t chainIdx = 0; chainIdx < _ikChains.size(); ++chainIdx)
    {
        IKChain& chain = _ikChains[chainIdx];
        int      idx = chain.boneIndex;
        if (idx < 0) continue;                         // 안전 검사

        /* ── (A) 단일축 제한이면 SolvePlane()으로 특화 처리 ── */
        if (chain.enableAxisLimit)
        {
            bool xOnly = (chain.limitMin.x || chain.limitMax.x) &&
                !chain.limitMin.y && !chain.limitMax.y &&
                !chain.limitMin.z && !chain.limitMax.z;
            bool yOnly = (chain.limitMin.y || chain.limitMax.y) &&
                !chain.limitMin.x && !chain.limitMax.x &&
                !chain.limitMin.z && !chain.limitMax.z;
            bool zOnly = (chain.limitMin.z || chain.limitMax.z) &&
                !chain.limitMin.x && !chain.limitMax.x &&
                !chain.limitMin.y && !chain.limitMax.y;

            if (xOnly) { SolvePlane(iter, chainIdx, SolveAxis::X, bones, local, world); continue; }
            if (yOnly) { SolvePlane(iter, chainIdx, SolveAxis::Y, bones, local, world); continue; }
            if (zOnly) { SolvePlane(iter, chainIdx, SolveAxis::Z, bones, local, world); continue; }
        }

        /* ── (B) 일반 CCD 회전 ───────────────────────────────── */

        /* 1) 링크 국소계로 IK/Target 위치 변환 */
        glm::mat4 invW = glm::inverse(world[idx]);
        glm::vec3 vIK = glm::normalize(glm::vec3(invW * world[_ikIndex][3]));
        glm::vec3 vTgt = glm::normalize(glm::vec3(invW * world[_targetIndex][3]));

        float  dot = glm::clamp(glm::dot(vTgt, vIK), -1.0f, 1.0f);
        float  angle = std::acos(dot);
        if (glm::degrees(angle) < 1.0e-3f) continue;          // 너무 작으면 skip
        if (angle > _ikLimitAngle)  angle = _ikLimitAngle;  // 1-회전 제한

        glm::vec3 axis = glm::normalize(glm::cross(vTgt, vIK));
        if (glm::length2(axis) < 1e-8f) continue;             // 축 불안정

        glm::quat dq = glm::angleAxis(angle, axis);

        /* 2) 축 제한(angle_limit) 처리 */
        if (chain.enableAxisLimit)
        {
            glm::quat curQ = glm::quat_cast(local[idx]);
            glm::quat newQ = dq * curQ;                     // 임시 적용
            glm::vec3 euler = DecomposeEuler(glm::toMat4(newQ), chain.prevAngle);

            /* min/max 클램프 */
            for (int a = 0; a < 3; ++a)
                euler[a] = std::clamp(euler[a], chain.limitMin[a], chain.limitMax[a]);

            /* 루프당 최대 회전각 제한 */
            glm::vec3 delta = euler - chain.prevAngle;
            for (int a = 0; a < 3; ++a)
                delta[a] = std::clamp(delta[a], -_ikLimitAngle, _ikLimitAngle);

            euler = chain.prevAngle + delta;
            chain.prevAngle = euler;
            newQ = glm::quat(euler);
            dq = newQ * glm::inverse(curQ);  // 순수 델타로 재계산
        }

        /* 3) 로컬 행렬에 델타 적용 (T * R_new) */
        glm::mat4 Rw = glm::toMat4(dq);
        local[idx] = local[idx] * (invW * Rw * world[idx]);

        /* 4) 링크부터 자식까지 월드 행렬 갱신 (간단히 전체 재계산) */
        for (int b = idx; b < (int)world.size(); ++b)
        {
            int p = bones[b].parent_index;
            world[b] = (p >= 0) ? world[p] * local[b]
                : local[b];
        }
    }
}

// ────────────────────────────────────────────────────────────────────
// 각도 유틸              (GLM 버전, 라디안 단위)
// ────────────────────────────────────────────────────────────────────
inline float IKSolver::NormalizeAngle(float a)          // 0 ≤ a < 2π
{
    const float TWO_PI = glm::two_pi<float>();
    a = std::fmod(a, TWO_PI);
    if (a < 0.0f) a += TWO_PI;
    return a;
}

inline float IKSolver::DiffAngle(float a, float b)      // (−π, π] 범위로 래핑된 차
{
    float diff = NormalizeAngle(a) - NormalizeAngle(b);
    if (diff > glm::pi<float>())      diff -= glm::two_pi<float>();
    else if (diff < -glm::pi<float>()) diff += glm::two_pi<float>();
    return diff;
}

// ────────────────────────────────────────────────────────────────────
// 회전행렬 → XYZ 오일러 분해 (이전 프레임 각과 가장 가까운 해 선택)
// DirectXMath Decompose 함수의 GLM 치환 버전
//    m      : 회전 행렬 (column-major, glm::mat4)
//    before : 이전 프레임 Euler (연속성 유지용)
// ────────────────────────────────────────────────────────────────────
glm::vec3 IKSolver::DecomposeEuler(const glm::mat4& m, const glm::vec3& before)
{
    glm::vec3 r;                                      // 1차 추출값
    float sy = -m[0][2];                              // −m(0,2)
    const float E = 1.0e-6f;

    // ── 짐벌락 검사 (|sy| ≃ 1) ──────────────────────────────────
    if (std::abs(1.0f - std::abs(sy)) < E)
    {
        r.y = std::asin(sy);                          // ±π/2 부근

        // X·Z 중 더 안정적인 축을 고정
        float sx = std::sin(before.x);
        float sz = std::sin(before.z);

        if (std::abs(sx) < std::abs(sz))              // Z축 우선
        {
            float cx = std::cos(before.x);
            if (cx > 0) { r.x = 0.0f;       r.z = std::asin(-m[1][0]); }
            else { r.x = glm::pi<float>(); r.z = std::asin(m[1][0]); }
        }
        else                                           // X축 우선
        {
            float cz = std::cos(before.z);
            if (cz > 0) { r.z = 0.0f;       r.x = std::asin(-m[2][1]); }
            else { r.z = glm::pi<float>(); r.x = std::asin(m[2][1]); }
        }
    }
    // ── 일반 케이스 ──────────────────────────────────────────────
    else
    {
        r.x = std::atan2(m[1][2], m[2][2]);
        r.y = std::asin(-m[0][2]);
        r.z = std::atan2(m[0][1], m[0][0]);
    }

    // ── ±π 보정 후보 8개 생성 후, 이전 값과 가장 가까운 해 선택 ──
    const float PI = glm::pi<float>();
    const glm::vec3 tests[8] = {
        { r.x + PI,  PI - r.y,  r.z + PI }, { r.x + PI,  PI - r.y,  r.z - PI },
        { r.x + PI, -PI - r.y,  r.z + PI }, { r.x + PI, -PI - r.y,  r.z - PI },
        { r.x - PI,  PI - r.y,  r.z + PI }, { r.x - PI,  PI - r.y,  r.z - PI },
        { r.x - PI, -PI - r.y,  r.z + PI }, { r.x - PI, -PI - r.y,  r.z - PI }
    };

    auto errSum = [&](const glm::vec3& v) -> float {
        return std::abs(DiffAngle(v.x, before.x)) +
            std::abs(DiffAngle(v.y, before.y)) +
            std::abs(DiffAngle(v.z, before.z));
        };

    float bestErr = errSum(r);
    glm::vec3 best = r;

    for (const glm::vec3& t : tests)
    {
        float e = errSum(t);
        if (e < bestErr) { bestErr = e; best = t; }
    }
    return best;                                       // 라디안 XYZ
}

/*--------------------------------------------------------------------
   단일 축(Plane)-제한 링크 전용 CCD 회전
   - iter        : CCD 반복 인덱스
   - chainIdx    : _ikChains 내 링크 인덱스
   - solveAxis   : X / Y / Z 중 하나
--------------------------------------------------------------------*/
void IKSolver::SolvePlane(unsigned              iter,
    std::size_t           chainIdx,
    SolveAxis             solveAxis,
    const pmx::PmxBone* bones,
    std::vector<glm::mat4>& local,
    std::vector<glm::mat4>& world)
{
    IKChain& chain = _ikChains[chainIdx];
    int      idx = chain.boneIndex;
    if (idx < 0) return;

    /* ── 0. 축 · 제한각 세팅 ───────────────────────────────────── */
    glm::vec3 rotateAxis;
    float     limitMin, limitMax;

    switch (solveAxis) {
    case SolveAxis::X: rotateAxis = { 1,0,0 }; limitMin = chain.limitMin.x; limitMax = chain.limitMax.x; break;
    case SolveAxis::Y: rotateAxis = { 0,1,0 }; limitMin = chain.limitMin.y; limitMax = chain.limitMax.y; break;
    case SolveAxis::Z: rotateAxis = { 0,0,1 }; limitMin = chain.limitMin.z; limitMax = chain.limitMax.z; break;
    }

    /* ── 1. 링크 국소계에서 IK / Target 방향 벡터 구하기 ─────── */
    glm::mat4 invW = glm::inverse(world[idx]);
    glm::vec3 vIK = glm::normalize(glm::vec3(invW * world[_ikIndex][3]));
    glm::vec3 vTarget = glm::normalize(glm::vec3(invW * world[_targetIndex][3]));

    /* ── 2. 두 방향 사이 각도 계산 & 1-스텝 각도 제한 ─────────── */
    float dot = glm::clamp(glm::dot(vTarget, vIK), -1.0f, 1.0f);
    float angle = std::acos(dot);
    angle = std::clamp(angle, -_ikLimitAngle, _ikLimitAngle);

    /* ── 3. +angle / −angle 중 더 가까운 쪽 선택 ───────────────── */
    glm::quat rotPlus = glm::angleAxis(angle, rotateAxis);
    glm::quat rotMinus = glm::angleAxis(-angle, rotateAxis);

    float dotPlus = glm::dot(vIK, glm::normalize(glm::vec3(rotPlus * glm::vec4(vTarget, 0))));
    float dotMinus = glm::dot(vIK, glm::normalize(glm::vec3(rotMinus * glm::vec4(vTarget, 0))));

    float  newAngle = chain.planeModeAngle + (dotPlus > dotMinus ? angle : -angle);

    /* ── 4. 첫 반복일 때 각 한계 넘어가면 부호 반전 후보 테스트 ─ */
    if (iter == 0 && (newAngle < limitMin || newAngle > limitMax))
    {
        if (-newAngle > limitMin && -newAngle < limitMax)  newAngle = -newAngle;
        else {
            float half = 0.5f * (limitMin + limitMax);
            if (std::abs(half - newAngle) > std::abs(half + newAngle))
                newAngle = -newAngle;
        }
    }

    /* ── 5. 제한각 안으로 clamp 후 Δ각 계산 ────────────────────── */
    newAngle = std::clamp(newAngle, limitMin, limitMax);
    float  deltaAngle = newAngle - chain.planeModeAngle;
    chain.planeModeAngle = newAngle;

    if (std::abs(deltaAngle) < 1e-6f) return;               // 변화 없으면 skip

    /* ── 6. 로컬 행렬에 Δ회전 적용 ─────────────────────────────── */
    glm::quat dq = glm::angleAxis(deltaAngle, rotateAxis);
    glm::vec3 t = glm::vec3(local[idx][3]);
    glm::quat curQ = glm::quat_cast(local[idx]);
    glm::mat4 newLocal = glm::translate(glm::mat4(1), t) *
        glm::toMat4(dq * curQ);
    local[idx] = newLocal;

    /* ── 7. 링크부터 후손까지 월드 행렬 재계산 ─────────────────── */
    for (int b = idx; b < (int)world.size(); ++b) {
        int p = bones[b].parent_index;
        world[b] = (p >= 0) ? world[p] * local[b]
            : local[b];
    }
}
