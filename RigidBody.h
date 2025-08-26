#pragma once

#include <glm/gtc/quaternion.hpp>
#include <btBulletDynamicsCommon.h>
#include "Pmx.h"

class NodeManager;
class BoneNode;
class MotionState;

class RigidBody
{
public:
    bool Init(const pmx::PmxRigidBody& pmxRigidBody, NodeManager* nodeManager, BoneNode* boneNode);

    void SetActive(bool active);
    void Reset(btDiscreteDynamicsWorld* world);
    void ResetTransform();
    void ReflectGlobalTransform();
    void CalcLocalTransform();

    uint8_t         _rigidBodyType = 0;
    unsigned short  _group = 0;
    unsigned short  _groupMask = 0;

    std::unique_ptr<btCollisionShape>  _shape;
    std::unique_ptr<MotionState>       _activeMotionState;
    std::unique_ptr<MotionState>       _kinematicMotionState;
    std::unique_ptr<btRigidBody>       _rigidBody;

    BoneNode* _node = nullptr;
    glm::mat4       _offsetMatrix{ 1.0f };

    std::wstring    _name;
};
