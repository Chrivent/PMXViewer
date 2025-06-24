#include "BoneNode.h"

#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtx/quaternion.hpp>

#include <algorithm>
#include "Vmd.h"
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

    glm::quat rotation = _animateRotation;
    if (_enableIK == true)
    {
        rotation = _ikRotation * rotation;
    }

    if (_isAppendRotate == true)
    {
        rotation = _appendRotation * rotation;
    }

    glm::vec3 t = _position + _animatePosition;
    if (_isAppendTranslate)
    {
        t += _appendTranslate;
    }

    glm::mat4 translate = glm::translate(glm::mat4(1.0f), t);

    _localTransform = translate * glm::toMat4(rotation) * scale;
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

        const char* interp = reinterpret_cast<const char*>(next.interpolation);

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
