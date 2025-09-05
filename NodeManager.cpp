#include "NodeManager.h"

#include <algorithm>
#include <execution>

#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtx/quaternion.hpp>

#include "IKSolver.h"

BoneNode::BoneNode(unsigned int index, const pmx::PmxBone& pmxBone)
{
    _boneIndex = index,
        _name = pmxBone.bone_name,
        _position = glm::vec3(pmxBone.position[0], pmxBone.position[1], pmxBone.position[2]),
        _parentBoneIndex = pmxBone.parent_index,
        _deformDepth = pmxBone.level,
        _boneFlag = pmxBone.bone_flag,
        _appendBoneIndex = pmxBone.grant_parent_index,
        _ikTargetBoneIndex = pmxBone.ik_target_bone_index,
        _ikIterationCount = pmxBone.ik_link_count,
        _ikLimit = pmxBone.ik_loop_angle_limit,
        _enableIK = false;
    _animatePosition = glm::vec3(0.0f);
    _animateRotation = glm::quat(1.0f, 0.0f, 0.0f, 0.0f);
    _morphPosition = glm::vec3(0.0f);
    _morphRotation = glm::quat(1.0f, 0.0f, 0.0f, 0.0f);
    _ikRotation = glm::quat(1.0f, 0.0f, 0.0f, 0.0f);
    _inverseInitTransform = glm::translate(glm::mat4(1.0f), -_position);
    _localTransform = glm::mat4(1.0f);
    _globalTransform = glm::mat4(1.0f);
    _appendTranslate = glm::vec3(0.0f);
    _appendRotation = glm::quat(1.0f, 0.0f, 0.0f, 0.0f);

    _parentBoneNode = nullptr;
    _isAppendRotate = false;
    _isAppendTranslate = false;
    _isAppendLocal = false;
    _appendWeight = 0.f;
    _appendBoneNode = nullptr;
    _ikSolver = nullptr;
}

void BoneNode::SetParentBoneNode(BoneNode* parentNode)
{
    _parentBoneNode = parentNode;
    _parentBoneNode->_childrenNodes.push_back(this);
}

void BoneNode::SortAllKeys() {
    std::sort(_motionKeys.begin(), _motionKeys.end(),
        [](const vmd::VmdBoneFrame& left, const vmd::VmdBoneFrame& right)
        {
            return left.frame <= right.frame;
        });
}

unsigned int BoneNode::GetMaxFrameNo() const {
    unsigned int maxFrame = 0;
    for (const auto& key : _motionKeys) {
        if (key.frame > static_cast<int>(maxFrame)) {
            maxFrame = key.frame;
        }
    }
    return maxFrame;
}

void BoneNode::UpdateLocalTransform()
{
    glm::mat4 scale = glm::mat4(1.0f);

    // 회전 조합
    glm::quat rotation = _animateRotation;
    if (_enableIK)
        rotation = _ikRotation * rotation;        // (프로젝트 컨벤션에 맞게 유지)
    if (_isAppendRotate)
        rotation = _appendRotation * rotation;

    // 위치: base + animate + morph (+ append)
    glm::vec3 t = _position + _animatePosition + _morphPosition;  // morph 추가
    if (_isAppendTranslate)
        t += _appendTranslate;

    // 행렬 구성 (현재 방식 유지: T * R * S)
    glm::mat4 translate = glm::translate(glm::mat4(1.0f), t);
    _localTransform = translate * glm::toMat4(glm::normalize(rotation)) * scale;
}

void BoneNode::UpdateGlobalTransform() {
    if (_parentBoneNode == nullptr)
    {
        _globalTransform = _localTransform;
    }
    else
    {
        _globalTransform = _parentBoneNode->_globalTransform * _localTransform;
    }

    for (BoneNode* child : _childrenNodes)
    {
        child->UpdateGlobalTransform();
    }
}

void BoneNode::UpdateAppendTransform()
{
    if (_appendBoneNode == nullptr)
    {
        return;
    }

    glm::quat appendRotation;
    if (_isAppendRotate == true)
    {
        if (_isAppendLocal == true)
        {
            appendRotation = _appendBoneNode->_animateRotation;
        }
        else
        {
            if (_appendBoneNode->_appendBoneNode == nullptr)
            {
                appendRotation = _appendBoneNode->_animateRotation;
            }
            else
            {
                appendRotation = _appendBoneNode->_appendRotation;
            }
        }

        if (_appendBoneNode->_enableIK == true)
        {
            appendRotation = _appendBoneNode->_ikRotation * appendRotation;
        }

        glm::quat appendRotationQuaternion = glm::quat_cast(glm::mat3(appendRotation));
        appendRotationQuaternion = glm::slerp(glm::quat(1, 0, 0, 0), appendRotationQuaternion, _appendWeight);
        _appendRotation = glm::toMat4(appendRotationQuaternion);
    }

    glm::vec3 appendTranslate(0.0f);
    if (_isAppendTranslate == true)
    {
        if (_isAppendLocal == true)
        {
            appendTranslate = _appendBoneNode->_animatePosition;
        }
        else
        {
            if (_appendBoneNode->_appendBoneNode == nullptr)
            {
                appendTranslate = _appendBoneNode->_animatePosition;
            }
            else
            {
                appendTranslate = _appendBoneNode->_appendTranslate;
            }
        }

        _appendTranslate = appendTranslate;
    }

    UpdateLocalTransform();
}

void BoneNode::UpdateChildTransform()
{
    for (BoneNode* child : _childrenNodes)
    {
        child->UpdateGlobalTransform();
    }
}

void BoneNode::AnimateMotion(float frameNo) {
    _animateRotation = glm::quat(1.0f, 0.0f, 0.0f, 0.0f);
    _animatePosition = glm::vec3(0.0f);

    if (_motionKeys.empty()) return;

    auto rit = std::find_if(_motionKeys.rbegin(), _motionKeys.rend(),
        [frameNo](const vmd::VmdBoneFrame& key)
        {
            return key.frame <= frameNo;
        });

    glm::vec3 pos1(rit->position[0], rit->position[1], rit->position[2]);
    glm::quat rot1(rit->orientation[3], rit->orientation[0], rit->orientation[1], rit->orientation[2]);

    auto it = rit.base(); // 다음 키프레임
    if (it != _motionKeys.end()) {
        const auto& next = *it;

        float total = static_cast<float>(next.frame - rit->frame);
        if (total <= 0.0f) return;

        float t = static_cast<float>(frameNo - rit->frame) / total;

        const uint8_t* interp = reinterpret_cast<const uint8_t*>(next.interpolation);

        glm::vec2 p1x(interp[0], interp[1]);
        glm::vec2 p2x(interp[8], interp[9]);
        float tx = GetYFromXOnBezier(t, p1x / 127.0f, p2x / 127.0f, 12);

        glm::vec2 p1y(interp[16], interp[17]);
        glm::vec2 p2y(interp[24], interp[25]);
        float ty = GetYFromXOnBezier(t, p1y / 127.0f, p2y / 127.0f, 12);

        glm::vec2 p1z(interp[32], interp[33]);
        glm::vec2 p2z(interp[40], interp[41]);
        float tz = GetYFromXOnBezier(t, p1z / 127.0f, p2z / 127.0f, 12);

        glm::vec2 p1r(interp[48], interp[49]);
        glm::vec2 p2r(interp[56], interp[57]);
        float tr = GetYFromXOnBezier(t, p1r / 127.0f, p2r / 127.0f, 12);

        glm::vec3 pos2(next.position[0], next.position[1], next.position[2]);
        glm::quat rot2(next.orientation[3], next.orientation[0], next.orientation[1], next.orientation[2]);

        _animatePosition = glm::vec3(
            glm::mix(pos1.x, pos2.x, glm::clamp(tx, 0.0f, 1.0f)),
            glm::mix(pos1.y, pos2.y, glm::clamp(ty, 0.0f, 1.0f)),
            glm::mix(pos1.z, pos2.z, glm::clamp(tz, 0.0f, 1.0f))
        );
        _animateRotation = glm::slerp(rot1, rot2, glm::clamp(tr, 0.0f, 1.0f));
    }
    else {
        _animatePosition = pos1;
        _animateRotation = rot1;
    }
}

void BoneNode::AnimateIK(float frameNo)
{
    if (_motionKeys.size() <= 0 || _ikSolver == nullptr)
    {
        return;
    }

    auto rit = std::find_if(_ikKeys.rbegin(), _ikKeys.rend(),
        [frameNo](const vmd::VmdIkFrame& key)
        {
            return key.frame <= frameNo;
        });

    if (rit == _ikKeys.rend())
    {
        return;
    }

    std::wstring ikName = _ikSolver->_ikNode->_name;

    auto it = std::find_if(rit->ik_enable.begin(), rit->ik_enable.end(),
        [&ikName](const vmd::VmdIkEnable& ik)
        {
            std::wstring name;
            oguna::EncodingConverter{}.Cp932ToUtf16(ik.ik_name.c_str(), static_cast<int>(ik.ik_name.size()), &name);
            return name == ikName;
        });

    if (it != rit->ik_enable.end())
    {
        _ikSolver->_enable = it->enable;
    }
}

float BoneNode::GetYFromXOnBezier(float x, const glm::vec2& a, const glm::vec2& b, int n, float epsilon)
{
    if (a.x == a.y && b.x == b.y)
        return x;

    float t = x;
    float k0 = 1 + 3 * a.x - 3 * b.x;
    float k1 = 3 * b.x - 6 * a.x;
    float k2 = 3 * a.x;

    for (int i = 0; i < n; ++i) {
        auto ft = k0 * t * t * t + k1 * t * t + k2 * t - x;

        if (ft <= epsilon && ft >= -epsilon)
        {
            break;
        }

        float dft = 3 * k0 * t * t + 2 * k1 * t + k2;
        if (dft <= epsilon && dft >= -epsilon)
        {
            break;
        }

        t -= ft / dft;

        t = glm::clamp(t, 0.0f, 1.0f);
    }

    float r = 1 - t;
    return t * t * t + 3 * t * t * r * b.y + 3 * t * r * r * a.y;
}

void NodeManager::Init(std::unique_ptr<pmx::PmxBone[]>& bones, size_t boneCount) {
    _boneNodeByIdx.resize(boneCount);
    _sortedNodes.resize(boneCount);

    for (int index = 0; index < boneCount; ++index)
    {
        const auto& currentBoneData = bones[index];
        _boneNodeByIdx[index] = new BoneNode(index, currentBoneData);
        _boneNodeByName[_boneNodeByIdx[index]->_name] = _boneNodeByIdx[index];
        _sortedNodes[index] = _boneNodeByIdx[index];
    }

    for (int index = 0; index < _boneNodeByIdx.size(); ++index)
    {
        BoneNode* currentBoneNode = _boneNodeByIdx[index];

        unsigned int parentBoneIndex = currentBoneNode->_parentBoneIndex;
        if (parentBoneIndex != 65535 && _boneNodeByIdx.size() > parentBoneIndex)
        {
            currentBoneNode->SetParentBoneNode(_boneNodeByIdx[parentBoneIndex]);
        }

        const pmx::PmxBone& currentPmxBone = bones[index];

        bool deformAfterPhysics = (currentPmxBone.bone_flag & 0x1000) != 0;
        currentBoneNode->_deformAfterPhysics = deformAfterPhysics;

        bool appendRotate = currentPmxBone.bone_flag & 0x0100;
        bool appendTranslate = currentPmxBone.bone_flag & 0x0200;
        currentBoneNode->_isAppendRotate = appendRotate;
        currentBoneNode->_isAppendTranslate = appendTranslate;
        if ((appendRotate || appendTranslate) && currentPmxBone.grant_parent_index < _boneNodeByIdx.size())
        {
            if (index > currentPmxBone.grant_parent_index)
            {
                bool appendLocal = (uint16_t)currentPmxBone.bone_flag & 0x0400;
                BoneNode* appendBoneNode = _boneNodeByIdx[currentPmxBone.grant_parent_index];
                currentBoneNode->_isAppendLocal = appendLocal;
                currentBoneNode->_appendBoneNode = appendBoneNode;
                currentBoneNode->_appendWeight = currentPmxBone.grant_weight;
            }
        }

        if ((currentPmxBone.bone_flag & 0x0020) && currentPmxBone.ik_target_bone_index < _boneNodeByIdx.size())
        {
            BoneNode* targetNode = _boneNodeByIdx[currentPmxBone.ik_target_bone_index];
            unsigned int iterationCount = currentPmxBone.ik_loop;
            float limitAngle = currentPmxBone.ik_loop_angle_limit;

            IKSolver* solver = new IKSolver(currentBoneNode, targetNode, iterationCount, limitAngle);
            _ikSolvers.push_back(solver);

            for (int i = 0; i < currentPmxBone.ik_link_count; ++i)
            {
                const pmx::PmxIkLink& ikLink = currentPmxBone.ik_links[i];
                int linkIndex = ikLink.link_target;
                if (linkIndex < 0 || linkIndex >= _boneNodeByIdx.size())
                {
                    continue;
                }

                BoneNode* linkNode = _boneNodeByIdx[linkIndex];
                if (ikLink.angle_lock)
                {
                    glm::vec3 limitMin(
                        ikLink.min_radian[0],
                        ikLink.min_radian[1],
                        ikLink.min_radian[2]
                    );
                    glm::vec3 limitMax(
                        ikLink.max_radian[0],
                        ikLink.max_radian[1],
                        ikLink.max_radian[2]
                    );
                    for (int d = 0; d < 3; ++d) {
                        if (limitMin[d] > limitMax[d]) {
                            std::swap(limitMin[d], limitMax[d]);
                        }
                    }
                    solver->_ikChains.emplace_back(linkNode, true, limitMin, limitMax);
                }
                else
                {
                    solver->_ikChains.emplace_back(linkNode, false,
                        glm::vec3(glm::radians(0.5f), 0.0f, 0.0f),
                        glm::vec3(glm::radians(180.0f), 0.0f, 0.0f));
                }
                linkNode->_enableIK = true;
            }
            currentBoneNode->_ikSolver = solver;
        }
    }

    for (int index = 0; index < _boneNodeByIdx.size(); ++index)
    {
        BoneNode* currentBoneNode = _boneNodeByIdx[index];
        int boneIndex = currentBoneNode->_boneIndex;
        int parentIndex = currentBoneNode->_parentBoneIndex;

        if (currentBoneNode->_parentBoneNode == nullptr) continue;

        glm::vec3 pos(
            bones[boneIndex].position[0],
            bones[boneIndex].position[1],
            bones[boneIndex].position[2]);

        glm::vec3 parentPos(
            bones[parentIndex].position[0],
            bones[parentIndex].position[1],
            bones[parentIndex].position[2]);

        glm::vec3 resultPos = pos - parentPos;
        currentBoneNode->_position = resultPos;
    }

    std::stable_sort(_sortedNodes.begin(), _sortedNodes.end(),
        [](const BoneNode* left, const BoneNode* right)
        {
            return left->_deformDepth < right->_deformDepth;
        });

    BuildLevels();
}

void NodeManager::BuildLevels() {
    _levels.clear();
    std::queue<std::pair<BoneNode*, int>> q;
    for (auto* n : _boneNodeByIdx) if (!n->_parentBoneNode) q.emplace(n, 0);
    while (!q.empty()) {
        auto [n, d] = q.front(); q.pop();
        if ((int)_levels.size() <= d) _levels.resize(d + 1);
        _levels[d].push_back(n);
        for (auto* c : n->_childrenNodes) q.emplace(c, d + 1);
    }
}

void NodeManager::SortKey() {
    for (int index = 0; index < _boneNodeByIdx.size(); index++)
    {
        BoneNode* currentBoneNode = _boneNodeByIdx[index];
        currentBoneNode->SortAllKeys();
        _duration = (std::max)(_duration, currentBoneNode->GetMaxFrameNo());
    }
}

BoneNode* NodeManager::GetBoneNodeByIndex(int index) const {
    if (index < 0 || index >= static_cast<int>(_boneNodeByIdx.size()))
        return nullptr;
    return _boneNodeByIdx[index];
}

BoneNode* NodeManager::GetBoneNodeByName(std::wstring& name) const {
    auto it = _boneNodeByName.find(name);
    if (it != _boneNodeByName.end())
        return it->second;
    return nullptr;
}

void NodeManager::BeforeUpdateAnimation()
{
    for (BoneNode* curNode : _boneNodeByIdx)
    {
        curNode->_morphPosition = glm::vec3(0.0f, 0.0f, 0.0f);
        curNode->_morphRotation = glm::quat(0.0f, 0.0f, 0.0f, 0.0f);
    }
}

void NodeManager::EvaluateAnimation(float frameNo)
{
    std::for_each(std::execution::par, _boneNodeByIdx.begin(), _boneNodeByIdx.end(),
        [&](BoneNode* n) {
            n->AnimateMotion(frameNo);
            n->AnimateIK(frameNo);
        });
}

void NodeManager::UpdateAnimation() {
    std::for_each(std::execution::par, _boneNodeByIdx.begin(), _boneNodeByIdx.end(),
        [&](BoneNode* n) {
            if (n->_deformAfterPhysics != true)
            {
                n->UpdateLocalTransform();
            }
        });

    if (_boneNodeByIdx.size() > 0)
    {
        if (_boneNodeByIdx[0]->_deformAfterPhysics != true)
        {
            _boneNodeByIdx[0]->UpdateGlobalTransform();
        }
    }

    for (auto& lvl : _levels) {
        std::for_each(std::execution::par, lvl.begin(), lvl.end(),
            [&](BoneNode* n) {
                if (n->_deformAfterPhysics != true)
                {
                    if (n->_appendBoneNode != nullptr)
                    {
                        n->UpdateAppendTransform();
                        n->UpdateGlobalTransform();
                    }

                    IKSolver* curSolver = n->_ikSolver;
                    if (curSolver != nullptr)
                    {
                        curSolver->Solve();
                        n->UpdateGlobalTransform();
                    }
                }
            });
    }
}

void NodeManager::UpdateAnimationAfterPhysics() {
    std::for_each(std::execution::par, _boneNodeByIdx.begin(), _boneNodeByIdx.end(),
        [&](BoneNode* n) {
            if (n->_deformAfterPhysics != false)
            {
                n->UpdateLocalTransform();
            }
        });

    if (_boneNodeByIdx.size() > 0)
    {
        if (_boneNodeByIdx[0]->_deformAfterPhysics != false)
        {
            _boneNodeByIdx[0]->UpdateGlobalTransform();
        }
    }

    for (auto& lvl : _levels) {
        std::for_each(std::execution::par, lvl.begin(), lvl.end(),
            [&](BoneNode* n) {
                if (n->_deformAfterPhysics != false)
                {
                    if (n->_appendBoneNode != nullptr)
                    {
                        n->UpdateAppendTransform();
                        n->UpdateGlobalTransform();
                    }

                    IKSolver* curSolver = n->_ikSolver;
                    if (curSolver != nullptr)
                    {
                        curSolver->Solve();
                        n->UpdateGlobalTransform();
                    }
                }
            });
    }
}

void NodeManager::Dispose() {
    for (BoneNode* node : _boneNodeByIdx)
    {
        delete node;
    }

    _boneNodeByIdx.clear();
    _boneNodeByName.clear();
    _sortedNodes.clear();
    _duration = 0;
}
