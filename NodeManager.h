#pragma once

#include <unordered_map>
#include "Pmx.h"

class BoneNode;
class IKSolver;

class NodeManager
{
public:
    void Init(std::unique_ptr<pmx::PmxBone[]>& bones, size_t boneCount);
    void BuildLevels();
    void SortKey();

    BoneNode* GetBoneNodeByIndex(int index) const;
    BoneNode* GetBoneNodeByName(std::wstring& name) const;

    void BeforeUpdateAnimation();
    void UpdateAnimation(float frameNo);

    void Dispose();

    std::unordered_map<std::wstring, BoneNode*> _boneNodeByName;
    std::vector<BoneNode*> _boneNodeByIdx;
    std::vector<BoneNode*> _sortedNodes;

    unsigned int _duration = 0;

    std::vector<IKSolver*> _ikSolvers;

    std::vector<std::vector<BoneNode*>> _levels;
};