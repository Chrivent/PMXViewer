#include "RigidBody.h"

#include "NodeManager.h"
#include "BoneNode.h"
#include "MotionState.h"

bool RigidBody::Init(const pmx::PmxRigidBody& rb, NodeManager* nodeManager, BoneNode* boneNode)
{
    _shape.reset();

    // 1) Collision Shape
    switch (rb.shape)
    {
    case 0: // Sphere
        _shape = std::make_unique<btSphereShape>(rb.size[0]); // radius
        break;
    case 1: // Box (주의: PMX가 절반/전체 치수 중 무엇을 주는지 모델에 따라 다름. 원본 구현처럼 그대로 사용)
        _shape = std::make_unique<btBoxShape>(btVector3(rb.size[0], rb.size[1], rb.size[2]));
        break;
    case 2: // Capsule
        _shape = std::make_unique<btCapsuleShape>(rb.size[0], rb.size[1]); // radius, height
        break;
    default:
        break;
    }
    if (!_shape) return false;

    // 2) Mass / Inertia
    btScalar   mass = 0.0f;
    btVector3  localInertia(0, 0, 0);

    const uint8_t calcType = rb.physics_calc_type; // 0:kinematic 1:dynamic 2:dynamic+merge
    if (calcType != 0) {
        mass = rb.mass;
    }
    if (mass != 0.0f) {
        _shape->calculateLocalInertia(mass, localInertia);
    }

    // 3) RigidBody Transform (rotation * translation : 원본과 동일한 순서 유지)
    glm::mat4 rotation(1.0f);
    rotation = glm::rotate(rotation, rb.orientation[0], glm::vec3(1, 0, 0)); // X (Pitch)
    rotation = glm::rotate(rotation, rb.orientation[1], glm::vec3(0, 1, 0)); // Y (Yaw)
    rotation = glm::rotate(rotation, rb.orientation[2], glm::vec3(0, 0, 1)); // Z (Roll)

    glm::mat4 translate = glm::translate(glm::mat4(1.0f),
        glm::vec3(rb.position[0], rb.position[1], rb.position[2]));
    glm::mat4 rigidBodyMat = rotation * translate; // 원 코드와 동일: rotation * translation

    // 4) 오프셋 계산 (offset = rigidBodyMat * inverse(boneGlobal))
    BoneNode* kinematicNode = nullptr;
    if (boneNode) {
        _offsetMatrix = rigidBodyMat * glm::inverse(boneNode->_globalTransform);
        kinematicNode = boneNode;
    }
    else {
        BoneNode* root = nodeManager->GetBoneNodeByIndex(0);
        _offsetMatrix = rigidBodyMat * glm::inverse(root->_globalTransform);
        kinematicNode = root;
    }

    // 5) MotionState 선택
    btMotionState* motionState = nullptr;
    if (calcType == 0) { // Kinematic
        _kinematicMotionState = std::make_unique<KinematicMotionState>(kinematicNode, _offsetMatrix);
        motionState = _kinematicMotionState.get();
    }
    else {
        if (boneNode) {
            if (calcType == 1) { // Dynamic
                _activeMotionState = std::make_unique<DynamicMotionState>(kinematicNode, _offsetMatrix);
                _kinematicMotionState = std::make_unique<KinematicMotionState>(kinematicNode, _offsetMatrix);
                motionState = _activeMotionState.get();
            }
            else {              // 2: Dynamic + BoneMerge
                _activeMotionState = std::make_unique<DynamicAndBoneMergeMotionState>(kinematicNode, _offsetMatrix);
                _kinematicMotionState = std::make_unique<KinematicMotionState>(kinematicNode, _offsetMatrix);
                motionState = _activeMotionState.get();
            }
        }
        else {
            // 본이 없는 경우 기본(디폴트) 모션 스테이트 사용
            _activeMotionState = std::make_unique<DefaultMotionState>(_offsetMatrix);
            _kinematicMotionState = std::make_unique<KinematicMotionState>(kinematicNode, _offsetMatrix);
            motionState = _activeMotionState.get();
        }
    }

    // 6) RigidBody 생성 및 파라미터 적용
    btRigidBody::btRigidBodyConstructionInfo info(mass, motionState, _shape.get(), localInertia);
    info.m_linearDamping = rb.move_attenuation;
    info.m_angularDamping = rb.rotation_attenuation;
    info.m_restitution = rb.repulsion;
    info.m_friction = rb.friction;
    info.m_additionalDamping = true;

    _rigidBody = std::make_unique<btRigidBody>(info);
    _rigidBody->setUserPointer(this);
    _rigidBody->setSleepingThresholds(0.01f, glm::radians(0.1f));
    _rigidBody->setActivationState(DISABLE_DEACTIVATION);

    if (calcType == 0) { // Kinematic
        _rigidBody->setCollisionFlags(_rigidBody->getCollisionFlags() | btCollisionObject::CF_KINEMATIC_OBJECT);
    }

    // 7) 그룹/마스크/메타
    _rigidBodyType = (calcType == 0) ? 0
        : (calcType == 1) ? 1
        : 2; // BoneMerge를 Aligned로 매핑

    _group = rb.group;
    _groupMask = rb.mask;     // ⚠ PMX의 "비충돌 마스크"를 쓰는 파일이라면 필요에 따라 비트 반전(~mask) 고려
    _node = boneNode;
    _name = std::wstring(rb.rigid_body_name.begin(), rb.rigid_body_name.end()); // utfstring→wstring 변환이 다르면 적절히 수정

    return true;
}

void RigidBody::SetActive(bool active)
{
    if (_rigidBodyType != 0)
    {
        if (active == true)
        {
            _rigidBody->setMotionState(_activeMotionState.get());
        }
        else
        {
            _rigidBody->setMotionState(_kinematicMotionState.get());
        }
    }
    else
    {
        _rigidBody->setMotionState(_kinematicMotionState.get());
    }
}

void RigidBody::Reset(btDiscreteDynamicsWorld* world)
{
    btOverlappingPairCache* cache = world->getPairCache();
    if (cache != nullptr)
    {
        btDispatcher* dispatcher = world->getDispatcher();
        cache->cleanProxyFromPairs(_rigidBody->getBroadphaseHandle(), dispatcher);
    }

    _rigidBody->setAngularVelocity(btVector3(0, 0, 0));
    _rigidBody->setLinearVelocity(btVector3(0, 0, 0));
    _rigidBody->clearForces();
}

void RigidBody::ResetTransform()
{
    if (_activeMotionState != nullptr)
    {
        _activeMotionState->Reset();
    }
}

void RigidBody::ReflectGlobalTransform()
{
    if (_activeMotionState != nullptr)
    {
        _activeMotionState->ReflectGlobalTransform();
    }

    if (_kinematicMotionState != nullptr)
    {
        _kinematicMotionState->ReflectGlobalTransform();
    }
}

void RigidBody::CalcLocalTransform()
{
    if (_node == nullptr)
        return;

    const BoneNode* parent = _node->_parentBoneNode;
    if (parent == nullptr)
    {
        // 루트면 로컬=글로벌
        _node->_localTransform = _node->_globalTransform;
    }
    else
    {
        // OpenGL/GLM (column-major)에서는 local = inverse(parentGlobal) * childGlobal
        const glm::mat4& parentGlobal = parent->_globalTransform;
        const glm::mat4& childGlobal = _node->_globalTransform;

        glm::mat4 local = glm::inverse(parentGlobal) * childGlobal;
        _node->_localTransform = local;
    }
}
