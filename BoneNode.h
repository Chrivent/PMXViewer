#pragma once

#include <string>
#include <vector>
#include <glm/gtc/quaternion.hpp>
#include "Pmx.h"

struct VMDKey
{
    unsigned int frameNo;
    glm::quat quaternion;
    glm::vec3 offset;
    glm::vec2 p1;
    glm::vec2 p2;

    VMDKey(unsigned int frameNo, const glm::quat& quaternion, const glm::vec3& offset,
        const glm::vec2& p1, const glm::vec2& p2)
        : frameNo(frameNo), quaternion(quaternion), offset(offset), p1(p1), p2(p2) {
    }
};

class BoneNode {
public:
    BoneNode(unsigned int index, const pmx::PmxBone& pmxBone);

    unsigned int GetBoneIndex() const { return _boneIndex; }
    const std::wstring& GetName() const { return _name; }
    unsigned int GetParentBoneIndex() const { return _parentBoneIndex; }
    unsigned int GetDeformDepth() const { return _deformDepth; }
    unsigned int GetAppendBoneIndex() const { return _appendBoneIndex; }
    unsigned int GetIKTargetBoneIndex() const { return _ikTargetBoneIndex; }

    void SetParentBoneNode(BoneNode* parentNode) {
        _parentBoneNode = parentNode;
        _parentBoneNode->AddChildBoneNode(this);
    }
    const BoneNode* GetParentBoneNode() const { return _parentBoneNode; }

    void AddChildBoneNode(BoneNode* childNode) { _childrenNodes.push_back(childNode); }
    const std::vector<BoneNode*>& GetChildrenNodes() const { return _childrenNodes; }

    const glm::mat4& GetInitInverseTransform() const { return _inverseInitTransform; }
    const glm::mat4& GetLocalTransform() const { return _localTransform; }
    const glm::mat4& GetGlobalTransform() const { return _globalTransform; }

    void SetAnimateRotation(const glm::quat& rotation) { _animateRotation = rotation; }
    const glm::quat& GetAnimateRotation() const { return _animateRotation; }
    const glm::vec3& GetAnimatePosition() const { return _animatePosition; }

    void SetPosition(const glm::vec3& position) { _position = position; }
    const glm::vec3& GetPosition() const { return _position; }

    void SetIKRotation(const glm::quat& rotation) { _ikRotation = rotation; }
    const glm::quat& GetIKRotation() const { return _ikRotation; }

    void SetMorphPosition(const glm::vec3& position) { _morphPosition = position; }
    void SetMorphRotation(const glm::quat& rotation) { _morphRotation = rotation; }

    void AddMotionKey(unsigned int frameNo, glm::quat quaternion, glm::vec3 offset, glm::vec2 p1, glm::vec2 p2);
    void SortAllKeys();

    void SetEnableAppendRotate(bool enable) { _isAppendRotate = enable; }
    void SetEnableAppendTranslate(bool enable) { _isAppendTranslate = enable; }
    void SetEnableAppendLocal(bool enable) { _isAppendLocal = enable; }
    void SetAppendWeight(float weight) { _appendWeight = weight; }
    float GetAppendWeight() const { return _appendWeight; }
    void SetAppendBoneNode(BoneNode* node) { _appendBoneNode = node; }
    BoneNode* GetAppendBoneNode() const { return _appendBoneNode; }
    const glm::quat& GetAppendRotation() const { return _appendRotation; }
    const glm::vec3& GetAppendTranslate() const { return _appendTranslate; }

    unsigned int GetMaxFrameNo() const;

    void UpdateLocalTransform();
    void UpdateGlobalTransform();

    void AnimateMotion(unsigned int frameNo);

private:
    float GetYFromXOnBezier(float x, const glm::vec2& a, const glm::vec2& b, uint8_t n);

private:
    unsigned int _boneIndex;
    std::wstring _name;
    glm::vec3 _position;
    unsigned int _parentBoneIndex = -1;
    unsigned int _deformDepth;
    uint16_t _boneFlag;
    unsigned int _appendBoneIndex;
    unsigned int _ikTargetBoneIndex;
    unsigned int _ikIterationCount;
    float _ikLimit;
    bool _enableIK = false;

    glm::vec3 _animatePosition;
    glm::quat _animateRotation;

    glm::vec3 _morphPosition;
    glm::quat _morphRotation;

    glm::quat _ikRotation;

    glm::vec3 _appendTranslate;
    glm::quat _appendRotation;

    glm::mat4 _inverseInitTransform = glm::mat4(1.0f);
    glm::mat4 _localTransform = glm::mat4(1.0f);
    glm::mat4 _globalTransform = glm::mat4(1.0f);

    BoneNode* _parentBoneNode = nullptr;
    std::vector<BoneNode*> _childrenNodes;

    bool _isAppendRotate = false;
    bool _isAppendTranslate = false;
    bool _isAppendLocal = false;
    float _appendWeight = 0.f;
    BoneNode* _appendBoneNode = nullptr;

    std::vector<VMDKey> _motionKeys;
};