#include "IKSolver.h"

#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtx/quaternion.hpp>
#include <glm/gtx/euler_angles.hpp>

#include "BoneNode.h"

IKSolver::IKSolver(BoneNode* node, BoneNode* targetNode, unsigned int iterationCount, float limitAngle)
    : _ikNode(node),
    _targetNode(targetNode),
    _ikIterationCount(iterationCount),
    _ikLimitAngle(limitAngle),
    _enable(true)
{
}

void IKSolver::Solve() {
    if (_enable == false)
    {
        return;
    }

    if (_ikNode == nullptr || _targetNode == nullptr)
    {
        return;
    }

    for (IKChain& chain : _ikChains)
    {
        chain.prevAngle = glm::vec3(0.0f);
        chain.boneNode->_ikRotation = glm::quat(1.0f, 0.0f, 0.0f, 0.0f);
        chain.planeModeAngle = 0.f;

        chain.boneNode->UpdateLocalTransform();
        chain.boneNode->UpdateGlobalTransform();
    }

    float maxDistance = (std::numeric_limits<float>::max)();
    for (unsigned int i = 0; i < _ikIterationCount; ++i)
    {
        SolveCore(i);

        glm::vec3 targetPos = glm::vec3(_targetNode->_globalTransform[3]);
        glm::vec3 ikPosition = glm::vec3(_ikNode->_globalTransform[3]);
        float dist = glm::length(targetPos - ikPosition);

        if (dist < maxDistance)
        {
            maxDistance = dist;
            for (IKChain& chain : _ikChains)
            {
                chain.saveIKRotation = chain.boneNode->_ikRotation;
            }
        }
        else
        {
            for (IKChain& chain : _ikChains)
            {
                chain.boneNode->_ikRotation = chain.saveIKRotation;
                chain.boneNode->UpdateLocalTransform();
                chain.boneNode->UpdateGlobalTransform();
            }
            break;
        }
    }
}

void IKSolver::SolveCore(unsigned int iteration) {
    glm::vec3 ikPosition = glm::vec3(_ikNode->_globalTransform[3]);
    for (unsigned int chainIndex = 0; chainIndex < _ikChains.size(); chainIndex++)
    {
        IKChain& chain = _ikChains[chainIndex];
        BoneNode* chainNode = chain.boneNode;
        if (chainNode == nullptr)
        {
            continue;
        }

        if (chain.enableAxisLimit == true)
        {
            const glm::vec3& min = chain.limitMin;
            const glm::vec3& max = chain.limitMax;

            if ((chain.limitMin.x != 0 || chain.limitMax.x != 0) &&
                (chain.limitMin.y == 0 || chain.limitMax.y == 0) &&
                (chain.limitMin.z == 0 || chain.limitMax.z == 0))
            {
                SolvePlane(iteration, chainIndex, SolveAxis::X);
                continue;
            }
            else if ((chain.limitMin.y != 0 || chain.limitMax.y != 0) &&
                (chain.limitMin.x == 0 || chain.limitMax.x == 0) &&
                (chain.limitMin.z == 0 || chain.limitMax.z == 0))
            {
                SolvePlane(iteration, chainIndex, SolveAxis::Y);
                continue;
            }
            else if ((chain.limitMin.z != 0 || chain.limitMax.z != 0) &&
                (chain.limitMin.x == 0 || chain.limitMax.x == 0) &&
                (chain.limitMin.y == 0 || chain.limitMax.y == 0))
            {
                SolvePlane(iteration, chainIndex, SolveAxis::Z);
                continue;
            }
        }

        glm::vec3 targetPosition = glm::vec3(_targetNode->_globalTransform[3]);

        glm::mat4 inverseChain = glm::inverse(chainNode->_globalTransform);

        glm::vec3 chainIKPosition = glm::vec3(inverseChain * glm::vec4(ikPosition, 1.0f));
        glm::vec3 chainTargetPosition = glm::vec3(inverseChain * glm::vec4(targetPosition, 1.0f));

        glm::vec3 chainIKVector = glm::normalize(chainIKPosition);
        glm::vec3 chainTargetVector = glm::normalize(chainTargetPosition);

        float dot = glm::dot(chainTargetVector, chainIKVector);
        dot = glm::clamp(dot, -1.f, 1.f);

        float angle = acos(dot);
        float angleDegree = glm::degrees(angle);
        if (angleDegree < 1.0e-3f)
        {
            continue;
        }

        angle = glm::clamp(angle, -_ikLimitAngle, _ikLimitAngle);
        glm::vec3 cross = glm::normalize(glm::cross(chainTargetVector, chainIKVector));
        glm::mat4 rotation = glm::rotate(glm::mat4(1.0f), angle, cross);

        glm::mat4 chainRotation = rotation * glm::toMat4(chainNode->_animateRotation) * glm::toMat4(chainNode->_ikRotation);
        if (chain.enableAxisLimit == true)
        {
            glm::vec3 rotXYZ = Decompose(chainRotation, chain.prevAngle);

            glm::vec3 clampXYZ = glm::clamp(rotXYZ, chain.limitMin, chain.limitMax);
            float invLimitAngle = -_ikLimitAngle;
            clampXYZ = glm::clamp(clampXYZ - chain.prevAngle, glm::vec3(invLimitAngle), glm::vec3(_ikLimitAngle));
            clampXYZ += chain.prevAngle;

            chainRotation = glm::eulerAngleXYZ(clampXYZ.x, clampXYZ.y, clampXYZ.z);
            chain.prevAngle = clampXYZ;
        }

        glm::mat4 inverseAnimate = glm::inverse(glm::toMat4(chain.boneNode->_animateRotation));

        glm::mat4 ikRotation = inverseAnimate * chainRotation;
        chainNode->_ikRotation = glm::quat_cast(ikRotation);

        chainNode->UpdateLocalTransform();
        chainNode->UpdateGlobalTransform();
    }
}

void IKSolver::SolvePlane(unsigned int iteration, unsigned int chainIndex, SolveAxis solveAxis) {
    glm::vec3 rotateAxis;
    float limitMinAngle;
    float limitMaxAngle;

    IKChain& chain = _ikChains[chainIndex];

    switch (solveAxis) {
    case SolveAxis::X:
        limitMinAngle = chain.limitMin.x;
        limitMaxAngle = chain.limitMax.x;
        rotateAxis = glm::vec3(1.f, 0.f, 0.f);
        break;
    case SolveAxis::Y:
        limitMinAngle = chain.limitMin.y;
        limitMaxAngle = chain.limitMax.y;
        rotateAxis = glm::vec3(0.f, 1.f, 0.f);
        break;
    case SolveAxis::Z:
        limitMinAngle = chain.limitMin.z;
        limitMaxAngle = chain.limitMax.z;
        rotateAxis = glm::vec3(0.f, 0.f, 1.f);
        break;
    }

    glm::vec3 ikPosition = glm::vec3(_ikNode->_globalTransform[3]);
    glm::vec3 targetPosition = glm::vec3(_targetNode->_globalTransform[3]);

    glm::mat4 inverseChain = glm::inverse(chain.boneNode->_globalTransform);

    glm::vec3 chainIKPosition = glm::vec3(inverseChain * glm::vec4(ikPosition, 1.0f));
    glm::vec3 chainTargetPosition = glm::vec3(inverseChain * glm::vec4(targetPosition, 1.0f));

    glm::vec3 chainIKVector = glm::normalize(chainIKPosition);
    glm::vec3 chainTargetVector = glm::normalize(chainTargetPosition);

    float dot = glm::dot(chainTargetVector, chainIKVector);
    dot = glm::clamp(dot, -1.f, 1.f);

    float angle = std::acos(dot);
    angle = glm::clamp(angle, -_ikLimitAngle, _ikLimitAngle);

    glm::quat rotation1 = glm::angleAxis(angle, rotateAxis);
    glm::vec3 targetVector1 = glm::rotate(rotation1, chainTargetVector);
    float dot1 = glm::dot(targetVector1, chainIKVector);

    glm::quat rotation2 = glm::angleAxis(-angle, rotateAxis);
    glm::vec3 targetVector2 = glm::rotate(rotation2, chainTargetVector);
    float dot2 = glm::dot(targetVector2, chainIKVector);

    float newAngle = chain.planeModeAngle;
    if (dot1 > dot2) {
        newAngle += angle;
    }
    else {
        newAngle -= angle;
    }

    if (iteration == 0) {
        if (newAngle < limitMinAngle || newAngle > limitMaxAngle) {
            if (-newAngle > limitMinAngle && -newAngle < limitMaxAngle) {
                newAngle *= -1;
            }
            else {
                float halfRadian = (limitMinAngle + limitMaxAngle) * 0.5f;
                if (std::abs(halfRadian - newAngle) > std::abs(halfRadian + newAngle)) {
                    newAngle *= -1;
                }
            }
        }
    }

    newAngle = glm::clamp(newAngle, limitMinAngle, limitMaxAngle);
    chain.planeModeAngle = newAngle;

    glm::mat4 inverseAnimate = glm::inverse(glm::toMat4(chain.boneNode->_animateRotation));

    glm::mat4 ikRotation = inverseAnimate * glm::rotate(glm::mat4(1.0f), newAngle, rotateAxis);

    chain.boneNode->_ikRotation = glm::quat_cast(ikRotation);

    chain.boneNode->UpdateLocalTransform();
    chain.boneNode->UpdateGlobalTransform();
}

glm::vec3 IKSolver::Decompose(const glm::mat4& m, const glm::vec3& before) {
    glm::vec3 r;
    float sy = -m[0][2];
    const float e = 1.0e-6f;

    if ((1.0f - std::abs(sy)) < e)
    {
        r.y = std::asin(sy);
        float sx = std::sin(before.x);
        float sz = std::sin(before.z);
        if (std::abs(sx) < std::abs(sz))
        {
            float cx = std::cos(before.x);
            if (cx > 0)
            {
                r.x = 0;
                r.z = std::asin(-m[1][0]);
            }
            else
            {
                r.x = glm::pi<float>();
                r.z = std::asin(m[1][0]);
            }
        }
        else
        {
            float cz = std::cos(before.z);
            if (cz > 0)
            {
                r.z = 0;
                r.x = std::asin(-m[2][1]);
            }
            else
            {
                r.z = glm::pi<float>();
                r.x = std::asin(m[2][1]);
            }
        }
    }
    else
    {
        r.x = std::atan2(m[1][2], m[2][2]);
        r.y = std::asin(-m[0][2]);
        r.z = std::atan2(m[0][1], m[0][0]);
    }

    const float pi = glm::pi<float>();
    std::vector<glm::vec3> tests = {
        { r.x + pi, pi - r.y, r.z + pi },
        { r.x + pi, pi - r.y, r.z - pi },
        { r.x + pi, -pi - r.y, r.z + pi },
        { r.x + pi, -pi - r.y, r.z - pi },
        { r.x - pi, pi - r.y, r.z + pi },
        { r.x - pi, pi - r.y, r.z - pi },
        { r.x - pi, -pi - r.y, r.z + pi },
        { r.x - pi, -pi - r.y, r.z - pi },
    };

    float errX = std::abs(DiffAngle(r.x, before.x));
    float errY = std::abs(DiffAngle(r.y, before.y));
    float errZ = std::abs(DiffAngle(r.z, before.z));
    float minErr = errX + errY + errZ;
    for (const auto& test : tests)
    {
        float err = std::abs(DiffAngle(test.x, before.x))
            + std::abs(DiffAngle(test.y, before.y))
            + std::abs(DiffAngle(test.z, before.z));
        if (err < minErr)
        {
            minErr = err;
            r = test;
        }
    }
    return r;
}

float IKSolver::NormalizeAngle(float angle)
{
    float ret = angle;
    while (ret >= glm::two_pi<float>()) {
        ret -= glm::two_pi<float>();
    }
    while (ret < 0.0f) {
        ret += glm::two_pi<float>();
    }
    return ret;
}

float IKSolver::DiffAngle(float a, float b)
{
    float diff = NormalizeAngle(a) - NormalizeAngle(b);
    if (diff > glm::pi<float>()) {
        return diff - glm::two_pi<float>();
    }
    else if (diff < -glm::pi<float>()) {
        return diff + glm::two_pi<float>();
    }
    return diff;
}
