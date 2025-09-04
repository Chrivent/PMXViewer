#include "Joint.h"

#include "RigidBody.h"

static btTransform MakeWorldAnchor(const float pos[3], const float euler[3])
{
    btMatrix3x3 basis;
    basis.setEulerZYX(euler[2], euler[1], euler[0]);

    btTransform t;
    t.setIdentity();
    t.setOrigin(btVector3(pos[0], pos[1], pos[2]));
    t.setBasis(basis);
    return t;
}

bool Joint::CreateJoint(const pmx::PmxJoint& pmxJoint, RigidBody* rigidBody0, RigidBody* rigidBody1)
{
    _constraint.reset();

    if (!rigidBody0 || !rigidBody1) return false;
    btRigidBody* A = rigidBody0->_rigidBody.get();
    btRigidBody* B = rigidBody1->_rigidBody.get();
    if (!A || !B) return false;

    const auto& p = pmxJoint.param;

    // 1) ���� ���� ��Ŀ ������
    //    ����: PmxJointParam �ʵ���� orientaiton �� ���ǵǾ� ����(��Ÿ �״�� ���)
    btTransform worldAnchor = MakeWorldAnchor(p.position, p.orientaiton);

    // 2) ���� ������ (Bullet�� ���� ������ ���� �䱸)
    btTransform frameInA = A->getWorldTransform().inverse() * worldAnchor;
    btTransform frameInB = B->getWorldTransform().inverse() * worldAnchor;

    // 3) 6DoF Spring ���� ����
    auto constraint = std::make_unique<btGeneric6DofSpringConstraint>(*A, *B, frameInA, frameInB, true);

    // 4) �Ѱ� ���� (�̵�/ȸ��)
    constraint->setLinearLowerLimit(btVector3(p.move_limitation_min[0], p.move_limitation_min[1], p.move_limitation_min[2]));
    constraint->setLinearUpperLimit(btVector3(p.move_limitation_max[0], p.move_limitation_max[1], p.move_limitation_max[2]));
    constraint->setAngularLowerLimit(btVector3(p.rotation_limitation_min[0], p.rotation_limitation_min[1], p.rotation_limitation_min[2]));
    constraint->setAngularUpperLimit(btVector3(p.rotation_limitation_max[0], p.rotation_limitation_max[1], p.rotation_limitation_max[2]));

    // 5) ������ ��� ���� (0�� �ƴϸ� �ش� �� ������ Ȱ��ȭ)
    if (p.spring_move_coefficient[0] != 0.0f) { constraint->enableSpring(0, true); constraint->setStiffness(0, p.spring_move_coefficient[0]); }
    if (p.spring_move_coefficient[1] != 0.0f) { constraint->enableSpring(1, true); constraint->setStiffness(1, p.spring_move_coefficient[1]); }
    if (p.spring_move_coefficient[2] != 0.0f) { constraint->enableSpring(2, true); constraint->setStiffness(2, p.spring_move_coefficient[2]); }

    if (p.spring_rotation_coefficient[0] != 0.0f) { constraint->enableSpring(3, true); constraint->setStiffness(3, p.spring_rotation_coefficient[0]); }
    if (p.spring_rotation_coefficient[1] != 0.0f) { constraint->enableSpring(4, true); constraint->setStiffness(4, p.spring_rotation_coefficient[1]); }
    if (p.spring_rotation_coefficient[2] != 0.0f) { constraint->enableSpring(5, true); constraint->setStiffness(5, p.spring_rotation_coefficient[2]); }

    _constraint = std::move(constraint);
    return true;
}
