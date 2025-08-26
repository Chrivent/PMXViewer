#pragma once

#include <btBulletDynamicsCommon.h>
#include "Pmx.h"

class RigidBody;

class Joint
{
public:
    // PMX → Bullet 변환된 시그니처
    bool CreateJoint(const pmx::PmxJoint& pmxJoint, RigidBody* rigidBody0, RigidBody* rigidBody1);

    std::unique_ptr<btTypedConstraint> _constraint;
};
