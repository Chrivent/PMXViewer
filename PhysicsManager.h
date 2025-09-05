#pragma once

#include <btBulletDynamicsCommon.h>
#include <glm/gtc/type_ptr.hpp>

#include "Pmx.h"

class NodeManager;
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
	DynamicMotionState(BoneNode* boneNode, const glm::mat4& offset, bool overrideBone = true);

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

class DynamicAndBoneMergeMotionState : public MotionState
{
public:
	DynamicAndBoneMergeMotionState(BoneNode* boneNode, const glm::mat4& offset, bool overrideBone = true);

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
	void setWorldTransform(const btTransform& worldTrans) override;
	void Reset() override;
	void ReflectGlobalTransform() override;

private:
	BoneNode* _boneNode = nullptr;
	glm::mat4  _offset{ 1.0f };
};

class RigidBody
{
public:
	bool Create(const pmx::PmxRigidBody& pmxRigidBody, NodeManager* nodeManager, BoneNode* boneNode);
	void Destroy();

	void SetActivation(bool active);
	void Reset(btDiscreteDynamicsWorld* world);
	void ResetTransform();
	void ReflectGlobalTransform();
	void CalcLocalTransform();

	glm::mat4 GetTransform();

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

class Joint
{
public:
	bool CreateJoint(const pmx::PmxJoint& pmxJoint, RigidBody* rigidBody0, RigidBody* rigidBody1);
	void Destroy();

	std::unique_ptr<btTypedConstraint> _constraint;
};

class Physics
{
public:
	~Physics();

	bool Create();
	void Destroy();

	void Update(float time);

	void AddRigidBody(RigidBody* rigidBody);
	void RemoveRigidBody(RigidBody* rigidBody);
	void AddJoint(Joint* joint);
	void RemoveJoint(Joint* joint);

public:
	std::unique_ptr<btDiscreteDynamicsWorld> _world;
	std::unique_ptr<btBroadphaseInterface> _broadPhase;
	std::unique_ptr<btDefaultCollisionConfiguration> _collisionConfig;
	std::unique_ptr<btCollisionDispatcher> _dispatcher;
	std::unique_ptr<btSequentialImpulseConstraintSolver> _solver;
	std::unique_ptr<btCollisionShape> _groundShape;
	std::unique_ptr<btMotionState> _groundMS;
	std::unique_ptr<btRigidBody> _groundRB;
	std::unique_ptr<btOverlapFilterCallback> _filterCB;

	double _fps;
	int _maxSubStepCount;
};

class PhysicsManager
{
public:
	PhysicsManager();
	~PhysicsManager();

	bool Create();

	RigidBody* AddRigidBody();
	Joint* AddJoint();

	std::unique_ptr<Physics>	_Physics;

	std::vector<std::unique_ptr<RigidBody>>	_rigidBodys;
	std::vector<std::unique_ptr<Joint>>		_joints;
};
