#pragma once

#include <glm/gtc/quaternion.hpp>
#include <vector>

class BoneNode;

struct IKChain
{
    BoneNode* boneNode;
    bool enableAxisLimit;
    glm::vec3 limitMin;
    glm::vec3 limitMax;
    glm::vec3 prevAngle;
    glm::quat saveIKRotation;
    float planeModeAngle;

    IKChain(BoneNode* linkNode, bool axisLimit, const glm::vec3& limitMinimum, const glm::vec3& limitMaximum)
    {
        boneNode = linkNode;
        enableAxisLimit = axisLimit;
        limitMin = limitMinimum;
        limitMax = limitMaximum;
        saveIKRotation = glm::quat(1.0f, 0.0f, 0.0f, 0.0f);
    }
};

enum class SolveAxis
{
    X, Y, Z
};

class IKSolver
{
public:
    IKSolver(BoneNode* node, BoneNode* targetNode, unsigned int iterationCount, float limitAngle);

    void Solve();

    void AddIKChain(BoneNode* linkNode, bool axisLimit, const glm::vec3& limitMin, const glm::vec3& limitMax)
    {
        _ikChains.emplace_back(linkNode, axisLimit, limitMin, limitMax);
    }

private:
    void SolveCore(unsigned int iteration);
    void SolvePlane(unsigned int iteration, unsigned int chainIndex, SolveAxis solveAxis);
    glm::vec3 Decompose(const glm::mat4& m, const glm::vec3& before);
    float NormalizeAngle(float angle);
    float DiffAngle(float a, float b);

public:
    bool _enable;

    BoneNode* _ikNode;
    BoneNode* _targetNode;

    std::vector<IKChain> _ikChains;

    unsigned int _ikIterationCount;
    float _ikLimitAngle;
};