#pragma once

#include <btBulletDynamicsCommon.h>
#include <memory>
#include <thread>

class RigidBody;
class Joint;

class PhysicsManager
{
public:
	PhysicsManager(const PhysicsManager& other) = delete;
	PhysicsManager& operator = (const PhysicsManager& other) = delete;

	static bool Create();
	static void Destroy();

	static void ActivePhysics(bool active);

	static void AddRigidBody(RigidBody* rigidBody);
	static void RemoveRigidBody(RigidBody* rigidBody);
	static void AddJoint(Joint* joint);
	static void RemoveJoint(Joint* joint);

private:
	static void UpdateByThread();

public:
	static std::unique_ptr<btDiscreteDynamicsWorld> _world;
	static std::unique_ptr<btBroadphaseInterface> _broadPhase;
	static std::unique_ptr<btDefaultCollisionConfiguration> _collisionConfig;
	static std::unique_ptr<btCollisionDispatcher> _dispatcher;
	static std::unique_ptr<btSequentialImpulseConstraintSolver> _solver;
	static std::unique_ptr<btCollisionShape> _groundShape;
	static std::unique_ptr<btMotionState> _groundMS;
	static std::unique_ptr<btRigidBody> _groundRB;
	static std::unique_ptr<btOverlapFilterCallback> _filterCB;

	static float _fixedTimeStep;
	static int _maxSubStepCount;

	static std::thread _physicsUpdateThread;
	static std::atomic<bool> _threadFlag;

	static std::atomic<bool> _stopFlag;
};
