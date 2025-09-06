#include "PhysicsManager.h"

#include "NodeManager.h"

glm::mat4 InvZ(const glm::mat4& m)
{
    const glm::mat4 invZ = glm::scale(glm::mat4(1), glm::vec3(1, 1, -1));
    return invZ * m * invZ;
}

DefaultMotionState::DefaultMotionState(const glm::mat4& transform)
{
    glm::mat4 trans = InvZ(transform);
    _transform.setFromOpenGLMatrix(&trans[0][0]);
    _initTransform = _transform;
}

void DefaultMotionState::getWorldTransform(btTransform& worldTrans) const
{
    worldTrans = _transform;
}

void DefaultMotionState::setWorldTransform(const btTransform& worldTrans)
{
    _transform = worldTrans;
}

void DefaultMotionState::Reset()
{
    _transform = _initTransform;
}

void DefaultMotionState::ReflectGlobalTransform()
{

}

DynamicMotionState::DynamicMotionState(BoneNode* boneNode, const glm::mat4& offset, bool overrideBone)
    : _boneNode(boneNode), _offset(offset), _override(overrideBone)
{
    _invOffset = glm::inverse(_offset);
    Reset();
}

void DynamicMotionState::getWorldTransform(btTransform& worldTrans) const
{
    worldTrans = _transform;
}

void DynamicMotionState::setWorldTransform(const btTransform& worldTrans)
{
    _transform = worldTrans;
}

void DynamicMotionState::Reset()
{
    glm::mat4 global = InvZ(_boneNode->_globalTransform * _offset);
    _transform.setFromOpenGLMatrix(&global[0][0]);
}

void DynamicMotionState::ReflectGlobalTransform()
{
    alignas(16) glm::mat4 world;
    _transform.getOpenGLMatrix(&world[0][0]);
    glm::mat4 btGlobal = InvZ(world) * _invOffset;

    if (_override)
    {
        _boneNode->_globalTransform = btGlobal;
        _boneNode->UpdateChildTransform();
    }
}

DynamicAndBoneMergeMotionState::DynamicAndBoneMergeMotionState(BoneNode* boneNode, const glm::mat4& offset, bool overrideBone)
    : _boneNode(boneNode), _offset(offset), _override(overrideBone)
{
    _invOffset = glm::inverse(_offset);
    Reset();
}

void DynamicAndBoneMergeMotionState::getWorldTransform(btTransform& worldTrans) const
{
    worldTrans = _transform;
}

void DynamicAndBoneMergeMotionState::setWorldTransform(const btTransform& worldTrans)
{
    _transform = worldTrans;
}

void DynamicAndBoneMergeMotionState::Reset()
{
    glm::mat4 global = InvZ(_boneNode->_globalTransform * _offset);
    _transform.setFromOpenGLMatrix(&global[0][0]);
}

void DynamicAndBoneMergeMotionState::ReflectGlobalTransform()
{
    alignas(16) glm::mat4 world;
    _transform.getOpenGLMatrix(&world[0][0]);
    glm::mat4 btGlobal = InvZ(world) * _invOffset;
    glm::mat4 global = _boneNode->_globalTransform;
    btGlobal[3] = global[3];

    if (_override)
    {
        _boneNode->_globalTransform = btGlobal;
        _boneNode->UpdateChildTransform();
    }
}

KinematicMotionState::KinematicMotionState(BoneNode* node, const glm::mat4& offset)
    : _boneNode(node), _offset(offset) {
}

void KinematicMotionState::getWorldTransform(btTransform& worldTrans) const
{
    glm::mat4 m;
    if (_boneNode != nullptr)
    {
        m = _boneNode->_globalTransform * _offset;
    }
    else
    {
        m = _offset;
    }
    m = InvZ(m);
    worldTrans.setFromOpenGLMatrix(&m[0][0]);
}

void KinematicMotionState::setWorldTransform(const btTransform&)
{

}

void KinematicMotionState::Reset()
{

}

void KinematicMotionState::ReflectGlobalTransform()
{

}

bool RigidBody::Create(const pmx::PmxRigidBody& pmxRigidBody, NodeManager* nodeManager, BoneNode* boneNode)
{
    Destroy();

    switch (pmxRigidBody.shape)
    {
    case 0:
        _shape = std::make_unique<btSphereShape>(pmxRigidBody.size[0]);
        break;
    case 1:
        _shape = std::make_unique<btBoxShape>(btVector3(pmxRigidBody.size[0], pmxRigidBody.size[1], pmxRigidBody.size[2]));
        break;
    case 2:
        _shape = std::make_unique<btCapsuleShape>(pmxRigidBody.size[0], pmxRigidBody.size[1]);
        break;
    default:
        break;
    }
    if (_shape == nullptr)
    {
        return false;
    }

    btScalar   mass = 0.0f;
    btVector3  localInertia(0, 0, 0);
    const uint8_t op = pmxRigidBody.physics_calc_type;
    if (op != 0) {
        mass = pmxRigidBody.mass;
    }
    if (mass != 0.0f) {
        _shape->calculateLocalInertia(mass, localInertia);
    }

    auto rx = glm::rotate(glm::mat4(1), pmxRigidBody.orientation[0], glm::vec3(1, 0, 0));
    auto ry = glm::rotate(glm::mat4(1), pmxRigidBody.orientation[1], glm::vec3(0, 1, 0));
    auto rz = glm::rotate(glm::mat4(1), pmxRigidBody.orientation[2], glm::vec3(0, 0, 1));
    glm::mat4 rotMat = ry * rx * rz;
    glm::mat4 translateMat = glm::translate(glm::mat4(1.0f), glm::vec3(pmxRigidBody.position[0], pmxRigidBody.position[1], pmxRigidBody.position[2]));

    glm::mat4 rbMat = InvZ(translateMat * rotMat);

    BoneNode* kinematicNode = nullptr;
    bool overrideNode = true;
    if (boneNode) {
        _offsetMatrix = glm::inverse(boneNode->_globalTransform) * rbMat;
        kinematicNode = boneNode;
    }
    else {
        BoneNode* root = nodeManager->GetBoneNodeByIndex(0);
        _offsetMatrix = glm::inverse(root->_globalTransform) * rbMat;
        kinematicNode = root;
        overrideNode = false;
    }

    btMotionState* motionState = nullptr;
    if (op == 0) {
        _kinematicMotionState = std::make_unique<KinematicMotionState>(kinematicNode, _offsetMatrix);
        motionState = _kinematicMotionState.get();
    }
    else {
        if (boneNode) {
            if (op == 1) {
                _activeMotionState = std::make_unique<DynamicMotionState>(kinematicNode, _offsetMatrix);
                _kinematicMotionState = std::make_unique<KinematicMotionState>(kinematicNode, _offsetMatrix);
                motionState = _activeMotionState.get();
            }
            else if (op == 2) {
                _activeMotionState = std::make_unique<DynamicAndBoneMergeMotionState>(kinematicNode, _offsetMatrix);
                _kinematicMotionState = std::make_unique<KinematicMotionState>(kinematicNode, _offsetMatrix);
                motionState = _activeMotionState.get();
            }
        }
        else {
            _activeMotionState = std::make_unique<DefaultMotionState>(_offsetMatrix);
            _kinematicMotionState = std::make_unique<KinematicMotionState>(kinematicNode, _offsetMatrix);
            motionState = _activeMotionState.get();
        }
    }

    btRigidBody::btRigidBodyConstructionInfo rbInfo(mass, motionState, _shape.get(), localInertia);
    rbInfo.m_linearDamping = pmxRigidBody.move_attenuation;
    rbInfo.m_angularDamping = pmxRigidBody.rotation_attenuation;
    rbInfo.m_restitution = pmxRigidBody.repulsion;
    rbInfo.m_friction = pmxRigidBody.friction;
    rbInfo.m_additionalDamping = true;

    _rigidBody = std::make_unique<btRigidBody>(rbInfo);
    _rigidBody->setUserPointer(this);
    _rigidBody->setSleepingThresholds(0.01f, glm::radians(0.1f));
    _rigidBody->setActivationState(DISABLE_DEACTIVATION);
    if (op == 0)
    {
        _rigidBody->setCollisionFlags(_rigidBody->getCollisionFlags() | btCollisionObject::CF_KINEMATIC_OBJECT);
    }

    _rigidBodyType = op;
    _group = pmxRigidBody.group;
    _groupMask = pmxRigidBody.mask;
    _node = boneNode;
    _name = pmxRigidBody.rigid_body_name;

    return true;
}

void RigidBody::Destroy()
{
    _shape = nullptr;
}

void RigidBody::SetActivation(bool active)
{
    if (_rigidBodyType != 0)
    {
        if (active == true)
        {
            _rigidBody->setCollisionFlags(_rigidBody->getCollisionFlags() & ~btCollisionObject::CF_KINEMATIC_OBJECT);
            _rigidBody->setMotionState(_activeMotionState.get());
        }
        else
        {
            _rigidBody->setCollisionFlags(_rigidBody->getCollisionFlags() | btCollisionObject::CF_KINEMATIC_OBJECT);
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
    auto cache = world->getPairCache();
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
    if (_node != nullptr)
    {
        auto parent = _node->_parentBoneNode;
        if (parent != nullptr)
        {
            auto local = glm::inverse(parent->_globalTransform) * _node->_globalTransform;
            _node->_localTransform = local;
        }
        else
        {
            _node->_localTransform = _node->_globalTransform;
        }
    }
}

glm::mat4 RigidBody::GetTransform()
{
    btTransform transform = _rigidBody->getCenterOfMassTransform();
    alignas(16) glm::mat4 mat;
    transform.getOpenGLMatrix(&mat[0][0]);
    return InvZ(mat);
}

bool Joint::CreateJoint(const pmx::PmxJoint& pmxJoint, RigidBody* rigidBodyA, RigidBody* rigidBodyB)
{
    Destroy();

    btMatrix3x3 rotMat;
    rotMat.setEulerZYX(pmxJoint.param.orientaiton[0], pmxJoint.param.orientaiton[1], pmxJoint.param.orientaiton[2]);

    btTransform transform;
    transform.setIdentity();
    transform.setOrigin(btVector3(
        pmxJoint.param.position[0],
        pmxJoint.param.position[1],
        pmxJoint.param.position[2]
    ));
    transform.setBasis(rotMat);

    btTransform invA = rigidBodyA->_rigidBody->getWorldTransform().inverse();
    btTransform invB = rigidBodyB->_rigidBody->getWorldTransform().inverse();
    invA = invA * transform;
    invB = invB * transform;

    auto constraint = std::make_unique<btGeneric6DofSpringConstraint>(
        *rigidBodyA->_rigidBody,
        *rigidBodyB->_rigidBody,
        invA,
        invB,
        true);
    constraint->setLinearLowerLimit(btVector3(
        pmxJoint.param.move_limitation_min[0],
        pmxJoint.param.move_limitation_min[1],
        pmxJoint.param.move_limitation_min[2]
    ));
    constraint->setLinearUpperLimit(btVector3(
        pmxJoint.param.move_limitation_max[0],
        pmxJoint.param.move_limitation_max[1],
        pmxJoint.param.move_limitation_max[2]
    ));

    constraint->setAngularLowerLimit(btVector3(
        pmxJoint.param.rotation_limitation_min[0],
        pmxJoint.param.rotation_limitation_min[1],
        pmxJoint.param.rotation_limitation_min[2]
    ));
    constraint->setAngularUpperLimit(btVector3(
        pmxJoint.param.rotation_limitation_max[0],
        pmxJoint.param.rotation_limitation_max[1],
        pmxJoint.param.rotation_limitation_max[2]
    ));

    if (pmxJoint.param.spring_move_coefficient[0] != 0)
    {
        constraint->enableSpring(0, true);
        constraint->setStiffness(0, pmxJoint.param.spring_move_coefficient[0]);
    }
    if (pmxJoint.param.spring_move_coefficient[1] != 0)
    {
        constraint->enableSpring(1, true);
        constraint->setStiffness(1, pmxJoint.param.spring_move_coefficient[1]);
    }
    if (pmxJoint.param.spring_move_coefficient[2] != 0)
    {
        constraint->enableSpring(2, true);
        constraint->setStiffness(2, pmxJoint.param.spring_move_coefficient[2]);
    }
    if (pmxJoint.param.spring_rotation_coefficient[0] != 0)
    {
        constraint->enableSpring(3, true);
        constraint->setStiffness(3, pmxJoint.param.spring_rotation_coefficient[0]);
    }
    if (pmxJoint.param.spring_rotation_coefficient[1] != 0)
    {
        constraint->enableSpring(4, true);
        constraint->setStiffness(4, pmxJoint.param.spring_rotation_coefficient[1]);
    }
    if (pmxJoint.param.spring_rotation_coefficient[2] != 0)
    {
        constraint->enableSpring(5, true);
        constraint->setStiffness(5, pmxJoint.param.spring_rotation_coefficient[2]);
    }

    _constraint = std::move(constraint);

    return true;
}

void Joint::Destroy()
{
    _constraint = nullptr;
}

struct FilterCallback : public btOverlapFilterCallback
{
	bool needBroadphaseCollision(btBroadphaseProxy* proxy0, btBroadphaseProxy* proxy1) const override
	{
		auto findIt = std::find_if(
			m_nonFilterProxy.begin(),
			m_nonFilterProxy.end(),
			[proxy0, proxy1](const auto& x) {return x == proxy0 || x == proxy1; }
		);
		if (findIt != m_nonFilterProxy.end())
		{
			return true;
		}
		bool collides = (proxy0->m_collisionFilterGroup & proxy1->m_collisionFilterMask) != 0;
		collides = collides && (proxy1->m_collisionFilterGroup & proxy0->m_collisionFilterMask);
		return collides;
	}

	std::vector<btBroadphaseProxy*> m_nonFilterProxy;
};

Physics::~Physics()
{
	Destroy();
}

bool Physics::Create()
{
	_broadPhase = std::make_unique<btDbvtBroadphase>();
	_collisionConfig = std::make_unique<btDefaultCollisionConfiguration>();
	_dispatcher = std::make_unique<btCollisionDispatcher>(_collisionConfig.get());

	_solver = std::make_unique<btSequentialImpulseConstraintSolver>();

	_world = std::make_unique<btDiscreteDynamicsWorld>(
		_dispatcher.get(),
		_broadPhase.get(),
		_solver.get(),
		_collisionConfig.get());

	_world->setGravity(btVector3(0, -9.8f * 10.0f, 0));

	_groundShape = std::make_unique<btStaticPlaneShape>(btVector3(0.f, 1.f, 0.f), 0.f);

	btTransform groundTransform;
	groundTransform.setIdentity();

	_groundMS = std::make_unique<btDefaultMotionState>(groundTransform);

	btRigidBody::btRigidBodyConstructionInfo groundInfo(0.f, _groundMS.get(), _groundShape.get(), btVector3(0.f, 0.f, 0.f));
	_groundRB = std::make_unique<btRigidBody>(groundInfo);

	_world->addRigidBody(_groundRB.get());

	auto filterCB = std::make_unique<FilterCallback>();
	filterCB->m_nonFilterProxy.push_back(_groundRB->getBroadphaseProxy());
	_world->getPairCache()->setOverlapFilterCallback(filterCB.get());
	_filterCB = std::move(filterCB);

    btContactSolverInfo& info = _world->getSolverInfo();
    info.m_numIterations = 4;
    info.m_solverMode = SOLVER_SIMD;

	return true;
}

void Physics::Destroy()
{
	if (_world != nullptr && _groundRB != nullptr)
	{
		_world->removeRigidBody(_groundRB.get());
	}

	_broadPhase = nullptr;
	_collisionConfig = nullptr;
	_dispatcher = nullptr;
	_solver = nullptr;
	_world = nullptr;
	_groundShape = nullptr;
	_groundMS = nullptr;
	_groundRB = nullptr;
}

void Physics::AddRigidBody(RigidBody* mmdRB)
{
	_world->addRigidBody(
		mmdRB->_rigidBody.get(),
		1 << mmdRB->_group,
		mmdRB->_groupMask
	);
}

void Physics::RemoveRigidBody(RigidBody* rigidBody)
{
	_world->removeRigidBody(rigidBody->_rigidBody.get());
}

void Physics::AddJoint(Joint* mmdJoint)
{
	if (mmdJoint->_constraint.get() != nullptr)
	{
		_world->addConstraint(mmdJoint->_constraint.get());
	}
}

void Physics::RemoveJoint(Joint* mmdJoint)
{
	if (mmdJoint->_constraint.get() != nullptr)
	{
		_world->removeConstraint(mmdJoint->_constraint.get());
	}
}

PhysicsManager::PhysicsManager()
{
    _threadFlag = false;
    _stopFlag.store(false);
    _endFlag.store(true);
}

PhysicsManager::~PhysicsManager()
{
    if (_threadFlag) {
        _stopFlag.store(true);
        if (_physicsUpdateThread.joinable())
            _physicsUpdateThread.join();
        _threadFlag = false;
        _endFlag.store(true);
    }

    if (_Physics && _Physics->_world) {
        for (auto& j : _joints) {
            _Physics->RemoveJoint(j.get());
        }
        for (auto& rb : _rigidBodys) {
            _Physics->RemoveRigidBody(rb.get());
        }
    }

    _joints.clear();
    _rigidBodys.clear();

    _Physics.reset();
}

bool PhysicsManager::Create()
{
	_Physics = std::make_unique<Physics>();
    return _Physics->Create();
}

RigidBody* PhysicsManager::AddRigidBody()
{
	auto rigidBody = std::make_unique<RigidBody>();
	auto ret = rigidBody.get();
	_rigidBodys.emplace_back(std::move(rigidBody));

	return ret;
}

Joint* PhysicsManager::AddJoint()
{
	auto joint = std::make_unique<Joint>();
	auto ret = joint.get();
	_joints.emplace_back(std::move(joint));

	return ret;
}

void PhysicsManager::ActivePhysics(bool active)
{
    if (active)
    {
        if (_threadFlag) return;
        _stopFlag.store(false);
        _endFlag.store(false);
        _threadFlag = true;

        _physicsUpdateThread = std::thread(&PhysicsManager::UpdateByThread, this);
    }
    else
    {
        if (!_threadFlag) return;
        _stopFlag.store(true);

        if (_physicsUpdateThread.joinable())
            _physicsUpdateThread.join();

        _threadFlag = false;
        _endFlag.store(true);
    }
}

void PhysicsManager::UpdateByThread()
{
    using clock = std::chrono::steady_clock;
    auto prev = clock::now();

    const int    kMaxSubSteps = 5;
    const double kFixedTimeStep = 0.02;

    while (!_stopFlag.load())
    {
        auto now = clock::now();
        double dt = std::chrono::duration<double>(now - prev).count();
        prev = now;

        if (dt > 0.25) dt = 0.25;

        auto phys = _Physics.get();
        if (phys && phys->_world)
        {
            phys->_world->stepSimulation(dt, kMaxSubSteps, (btScalar)kFixedTimeStep);
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    _endFlag.store(true);
}
