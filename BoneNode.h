#pragma once

#include <glm/gtc/quaternion.hpp>
#include "Pmx.h"
#include "Vmd.h"

class IKSolver;

class BoneNode
{
public:
    BoneNode(unsigned int index, const pmx::PmxBone& pmxBone);

    void SetParentBoneNode(BoneNode* parentNode);

    void SortAllKeys();

    unsigned int GetMaxFrameNo() const;

    void UpdateLocalTransform();

    void UpdateGlobalTransform();

    void AnimateMotion(float frameNo);

    float GetYFromXOnBezier(float x, const glm::vec2& a, const glm::vec2& b, int n, float epsilon = 1e-5f);

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

    glm::mat4 _inverseInitTransform;
    glm::mat4 _localTransform;
    glm::mat4 _globalTransform;

    BoneNode* _parentBoneNode = nullptr;
    std::vector<BoneNode*> _childrenNodes;

    bool _isAppendRotate = false;
    bool _isAppendTranslate = false;
    bool _isAppendLocal = false;
    float _appendWeight = 0.f;
    BoneNode* _appendBoneNode = nullptr;

    std::vector<vmd::VmdBoneFrame> _motionKeys;
    std::vector<vmd::VmdIkFrame> _ikKeys;

    IKSolver* _ikSolver = nullptr;

    void AnimateIK(float frameNo);

    void UpdateAppendTransform();
};