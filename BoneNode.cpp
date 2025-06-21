#include "BoneNode.h"

#include <algorithm>

#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtx/quaternion.hpp>

BoneNode::BoneNode(unsigned int index, const pmx::PmxBone& pmxBone)
    : _boneIndex(index),
    _name(pmxBone.bone_name),
    _position(pmxBone.position[0], pmxBone.position[1], pmxBone.position[2]),
    _parentBoneIndex(pmxBone.parent_index),
    _deformDepth(pmxBone.level),
    _boneFlag(pmxBone.bone_flag),
    _appendBoneIndex(pmxBone.grant_parent_index),
    _ikTargetBoneIndex(pmxBone.ik_target_bone_index),
    _ikIterationCount(pmxBone.ik_loop),
    _ikLimit(pmxBone.ik_loop_angle_limit),
    _animatePosition(0.f),
    _animateRotation(glm::mat4(1.0f)),
    _inverseInitTransform(glm::translate(glm::mat4(1.0f), -_position)),
    _localTransform(glm::mat4(1.0f)),
    _globalTransform(glm::mat4(1.0f)),
    _appendTranslate(0.f),
    _appendRotation(glm::mat4(1.0f))
{
}

void BoneNode::AddMotionKey(unsigned int frameNo, glm::quat quaternion, glm::vec3 offset, glm::vec2 p1, glm::vec2 p2)
{
    _motionKeys.emplace_back(frameNo, quaternion, offset, p1, p2);
}

void BoneNode::SortAllKeys()
{
    std::sort(_motionKeys.begin(), _motionKeys.end(),
        [](const VMDKey& left, const VMDKey& right)
        {
            return left.frameNo <= right.frameNo;
        });
}

unsigned int BoneNode::GetMaxFrameNo() const
{
    if (_motionKeys.empty())
        return 0;
    return _motionKeys.back().frameNo;
}

void BoneNode::UpdateLocalTransform()
{
    glm::mat4 scale = glm::mat4(1.0f);  // 현재 스케일 없음

    glm::mat4 rotation = glm::toMat4(_animateRotation);

    glm::vec3 t = _animatePosition + _position;
    glm::mat4 translate = glm::translate(glm::mat4(1.0f), t);

    _localTransform = translate * rotation;  // 일반적으로 T * R * S 순서
}

void BoneNode::UpdateGlobalTransform()
{
    if (_parentBoneNode == nullptr)
    {
        _globalTransform = _localTransform;
    }
    else
    {
        _globalTransform = _parentBoneNode->GetGlobalTransform() * _localTransform;
    }

    for (BoneNode* child : _childrenNodes)
    {
        child->UpdateGlobalTransform();
    }
}

void BoneNode::AnimateMotion(unsigned int frameNo)
{
    _animateRotation = glm::mat4(1.0f);
    _animatePosition = glm::vec3(0.0f);

    if (_motionKeys.empty())
        return;

    auto rit = std::find_if(_motionKeys.rbegin(), _motionKeys.rend(),
        [frameNo](const VMDKey& key)
        {
            return key.frameNo <= frameNo;
        });

    if (rit == _motionKeys.rend())
        return;

    glm::vec3 animatePosition = rit->offset;

    auto it = rit.base();

    if (it != _motionKeys.end())
    {
        float t = static_cast<float>(frameNo - rit->frameNo) /
            static_cast<float>(it->frameNo - rit->frameNo);

        t = GetYFromXOnBezier(t, rit->p1, rit->p2, 12);

        glm::quat rot = glm::slerp(rit->quaternion, it->quaternion, t);
        _animateRotation = glm::toMat4(rot);
        _animatePosition = glm::mix(animatePosition, it->offset, t);
    }
    else
    {
        _animateRotation = glm::toMat4(rit->quaternion);
    }
}

float BoneNode::GetYFromXOnBezier(float x, const glm::vec2& a, const glm::vec2& b, uint8_t n)
{
    constexpr float epsilon = 1e-5f;

    if (a.x == a.y && b.x == b.y)
        return x;

    float t = x;

    const float k0 = 1.0f + 3.0f * a.x - 3.0f * b.x;
    const float k1 = 3.0f * b.x - 6.0f * a.x;
    const float k2 = 3.0f * a.x;

    for (int i = 0; i < n; ++i)
    {
        float ft = k0 * t * t * t + k1 * t * t + k2 * t - x;

        if (std::abs(ft) < epsilon)
            break;

        float dft = 3.0f * k0 * t * t + 2.0f * k1 * t + k2;
        if (std::abs(dft) < epsilon)
            break;

        t -= ft / dft;
    }

    float r = 1.0f - t;
    return t * t * t + 3.0f * t * t * r * b.y + 3.0f * t * r * r * a.y;
}
