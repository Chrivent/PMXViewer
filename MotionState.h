#pragma once

#include <btBulletDynamicsCommon.h>
#include <glm/gtc/type_ptr.hpp>

class BoneNode;

class MotionState : public btMotionState
{
public:
	virtual void Reset() = 0;
	virtual void ReflectGlobalTransform() = 0;
};

class DefaultMotionState : public MotionState
{
public:
    DefaultMotionState(const glm::mat4& transform);

    void getWorldTransform(btTransform& worldTrans) const override;
    void setWorldTransform(const btTransform& worldTrans) override;
    void Reset() override;
    void ReflectGlobalTransform() override;

private:
    btTransform _initTransform;
    btTransform _transform;
};

class DynamicMotionState : public MotionState
{
public:
    // offset: (�����ٵ� ����) -> (�� �۷ι�) �� ���� ������
    DynamicMotionState(BoneNode* boneNode,
        const glm::mat4& offset,
        bool overrideBone = true);

    void getWorldTransform(btTransform& worldTrans) const override;
    void setWorldTransform(const btTransform& worldTrans) override;
    void Reset() override;
    void ReflectGlobalTransform() override;

private:
    BoneNode* _boneNode = nullptr;
    glm::mat4   _offset{ 1.0f };
    glm::mat4   _invOffset{ 1.0f };
    btTransform _transform;   // Bullet world transform
    bool        _override = true;
};

class DynamicAndBoneMergeMotionState : public MotionState
{
public:
    // offset: (�����ٵ� ����) -> (�� �۷ι�) ��ȯ
    DynamicAndBoneMergeMotionState(BoneNode* boneNode,
        const glm::mat4& offset,
        bool overrideBone = true);

    void getWorldTransform(btTransform& worldTrans) const override;
    void setWorldTransform(const btTransform& worldTrans) override;
    void Reset() override;
    void ReflectGlobalTransform() override;

private:
    BoneNode* _boneNode = nullptr;
    glm::mat4   _offset{ 1.0f };
    glm::mat4   _invOffset{ 1.0f };
    btTransform _transform;
    bool        _override = true;
};

class KinematicMotionState : public MotionState
{
public:
    KinematicMotionState(BoneNode* node, const glm::mat4& offset);

    void getWorldTransform(btTransform& worldTrans) const override;
    void setWorldTransform(const btTransform& /*worldTrans*/) override;
    void Reset() override;
    void ReflectGlobalTransform() override;

private:
    BoneNode* _boneNode = nullptr;
    glm::mat4  _offset{ 1.0f };  // �����ٵ� ���á溻 �۷ι� ������
};
