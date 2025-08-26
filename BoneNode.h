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
    void UpdateAppendTransform();
    void UpdateChildTransform();

    void AnimateMotion(float frameNo);
    void AnimateIK(float frameNo);

    unsigned int _boneIndex;
    std::wstring _name;
    glm::vec3 _position;
    unsigned int _parentBoneIndex;
    unsigned int _deformDepth;
    uint16_t _boneFlag;
    unsigned int _appendBoneIndex;
    unsigned int _ikTargetBoneIndex;
    unsigned int _ikIterationCount;
    float _ikLimit;
    bool _enableIK;

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

    BoneNode* _parentBoneNode;
    std::vector<BoneNode*> _childrenNodes;

    bool _isAppendRotate;
    bool _isAppendTranslate;
    bool _isAppendLocal;
    float _appendWeight;
    BoneNode* _appendBoneNode;

    std::vector<vmd::VmdBoneFrame> _motionKeys;
    std::vector<vmd::VmdIkFrame> _ikKeys;

    IKSolver* _ikSolver;

private:
    float GetYFromXOnBezier(float x, const glm::vec2& a, const glm::vec2& b, int n, float epsilon = 1e-5f);
};