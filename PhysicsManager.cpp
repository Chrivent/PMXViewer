#include "PhysicsManager.h"

#include <vector>
#include <chrono>

#include "RigidBody.h"
#include "Joint.h"

std::unique_ptr<btDiscreteDynamicsWorld> PhysicsManager::_world = nullptr;
std::unique_ptr<btBroadphaseInterface> PhysicsManager::_broadPhase = nullptr;
std::unique_ptr<btDefaultCollisionConfiguration> PhysicsManager::_collisionConfig = nullptr;
std::unique_ptr<btCollisionDispatcher> PhysicsManager::_dispatcher = nullptr;
std::unique_ptr<btSequentialImpulseConstraintSolver> PhysicsManager::_solver = nullptr;
std::unique_ptr<btCollisionShape> PhysicsManager::_groundShape = nullptr;
std::unique_ptr<btMotionState> PhysicsManager::_groundMS = nullptr;
std::unique_ptr<btRigidBody> PhysicsManager::_groundRB = nullptr;
std::unique_ptr<btOverlapFilterCallback> PhysicsManager::_filterCB = nullptr;

float PhysicsManager::_fixedTimeStep = 0.02f;
int PhysicsManager::_maxSubStepCount = 5;

std::thread PhysicsManager::_physicsUpdateThread = std::thread();
std::atomic<bool> PhysicsManager::_threadFlag = false;

std::atomic<bool> PhysicsManager::_stopFlag(false);

struct FilterCallback : public btOverlapFilterCallback
{
	bool needBroadphaseCollision(btBroadphaseProxy* proxy0, btBroadphaseProxy* proxy1) const override
	{
		auto findIterator = std::find_if(NonFilterProxy.begin(), NonFilterProxy.end(),
			[proxy0, proxy1](const auto& x)
			{
				return x == proxy0 || x == proxy1;
			});

		if (findIterator != NonFilterProxy.end())
		{
			return true;
		}

		bool collides = (proxy0->m_collisionFilterGroup & proxy1->m_collisionFilterMask) != 0;
		collides = collides && (proxy1->m_collisionFilterGroup & proxy0->m_collisionFilterMask);
		return collides;
	}

	std::vector<btBroadphaseProxy*> NonFilterProxy;
};

bool PhysicsManager::Create()
{
	_broadPhase = std::make_unique<btDbvtBroadphase>();
	_collisionConfig = std::make_unique<btDefaultCollisionConfiguration>();
	_dispatcher = std::make_unique<btCollisionDispatcher>(_collisionConfig.get());
	_solver = std::make_unique<btSequentialImpulseConstraintSolver>();

	_world = std::make_unique<btDiscreteDynamicsWorld>(_dispatcher.get(), _broadPhase.get(), _solver.get(), _collisionConfig.get());
	_world->setGravity(btVector3(0, -9.8f * 10.0f, 0));

	_groundShape = std::make_unique<btStaticPlaneShape>(btVector3(0.f, 1.f, 0.f), 0.f);

	btTransform groundTransform;
	groundTransform.setIdentity();

	_groundMS = std::make_unique<btDefaultMotionState>(groundTransform);

	btRigidBody::btRigidBodyConstructionInfo groundInfo(0.f, _groundMS.get(), _groundShape.get(), btVector3(0.f, 0.f, 0.f));
	_groundRB = std::make_unique<btRigidBody>(groundInfo);

	_world->addRigidBody(_groundRB.get());

	auto filterCB = std::make_unique<FilterCallback>();
	filterCB->NonFilterProxy.push_back(_groundRB->getBroadphaseProxy());
	_world->getPairCache()->setOverlapFilterCallback(filterCB.get());
	_filterCB = std::move(filterCB);

	btContactSolverInfo& info = _world->getSolverInfo();
	info.m_numIterations = 4;
	info.m_solverMode = SOLVER_SIMD;

	return true;
}

void PhysicsManager::Destroy()
{
	ActivePhysics(false);

	if (_world) {
		if (_groundRB) _world->removeRigidBody(_groundRB.get());
	}
	_filterCB.reset();
	_groundRB.reset();
	_groundMS.reset();
	_groundShape.reset();

	_world.reset();
	_solver.reset();
	_dispatcher.reset();
	_collisionConfig.reset();
	_broadPhase.reset();
}

void PhysicsManager::ActivePhysics(bool active)
{
	if (active)
	{
		if (_threadFlag.load()) return;
		_stopFlag.store(false, std::memory_order_release);
		_physicsUpdateThread = std::thread(&PhysicsManager::UpdateByThread); // ← this 없이
		_threadFlag.store(true, std::memory_order_release);
	}
	else
	{
		if (!_threadFlag.load()) return;
		_stopFlag.store(true, std::memory_order_release);
		if (_physicsUpdateThread.joinable())
			_physicsUpdateThread.join();   // detach 대신 join
		_threadFlag.store(false, std::memory_order_release);
	}
}

void PhysicsManager::AddRigidBody(RigidBody* rigidBody)
{
	_world->addRigidBody(rigidBody->_rigidBody.get(), 1 << rigidBody->_group, rigidBody->_groupMask);
}

void PhysicsManager::RemoveRigidBody(RigidBody* rigidBody)
{
	_world->removeRigidBody(rigidBody->_rigidBody.get());
}

void PhysicsManager::AddJoint(Joint* joint)
{
	if (joint->_constraint.get() == nullptr)
	{
		return;
	}

	_world->addConstraint(joint->_constraint.get());
}

void PhysicsManager::RemoveJoint(Joint* joint)
{
	if (joint->_constraint.get() == nullptr)
	{
		return;
	}

	_world->removeConstraint(joint->_constraint.get());
}

void PhysicsManager::UpdateByThread()
{
	using clock = std::chrono::steady_clock;
	auto prev = clock::now();
	constexpr float MAX_FRAME_SEC = 0.10f;

	while (!_stopFlag.load(std::memory_order_acquire))
	{
		auto now = clock::now();
		float dt = std::chrono::duration<float>(now - prev).count();
		prev = now;

		if (dt > MAX_FRAME_SEC) dt = MAX_FRAME_SEC;
		if (_world) _world->stepSimulation(dt, _maxSubStepCount, _fixedTimeStep);

		std::this_thread::sleep_for(std::chrono::milliseconds(1)); // 과열 방지
	}
}
