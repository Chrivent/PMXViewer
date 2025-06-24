#include "NodeManager.h"

#include <algorithm>
#include "BoneNode.h"
#include "IKSolver.h"

void NodeManager::Init(std::unique_ptr<pmx::PmxBone[]>& bones, size_t boneCount) {
    _boneNodeByIdx.resize(boneCount);
    _sortedNodes.resize(boneCount);

    for (int index = 0; index < boneCount; ++index)
    {
        const auto& currentBoneData = bones[index];
        _boneNodeByIdx[index] = new BoneNode(index, currentBoneData);
        _boneNodeByName[_boneNodeByIdx[index]->_name] = _boneNodeByIdx[index];
        _sortedNodes[index] = _boneNodeByIdx[index];
    }

    for (int index = 0; index < _boneNodeByIdx.size(); ++index)
    {
        BoneNode* currentBoneNode = _boneNodeByIdx[index];

        unsigned int parentBoneIndex = currentBoneNode->_parentBoneIndex;
        if (parentBoneIndex != 65535 && _boneNodeByIdx.size() > parentBoneIndex)
        {
            currentBoneNode->SetParentBoneNode(_boneNodeByIdx[parentBoneIndex]);
        }

        const pmx::PmxBone& currentPmxBone = bones[index];

        if ((currentPmxBone.bone_flag & 0x0020) && currentPmxBone.ik_target_bone_index < _boneNodeByIdx.size())
        {
            BoneNode* targetNode = _boneNodeByIdx[currentPmxBone.ik_target_bone_index];
            unsigned int iterationCount = currentPmxBone.ik_loop;
            float limitAngle = currentPmxBone.ik_loop_angle_limit;

            IKSolver* solver = new IKSolver(currentBoneNode, targetNode, iterationCount, limitAngle);
            _ikSolvers.push_back(solver);

            for (int i = 0; i < currentPmxBone.ik_link_count; ++i)
            {
                const pmx::PmxIkLink& ikLink = currentPmxBone.ik_links[i];
                int linkIndex = ikLink.link_target;
                if (linkIndex < 0 || linkIndex >= _boneNodeByIdx.size())
                {
                    continue;
                }

                BoneNode* linkNode = _boneNodeByIdx[linkIndex];
                if (ikLink.angle_lock == true)
                {
                    glm::vec3 limitMin(
                        ikLink.min_radian[0],
                        ikLink.min_radian[1],
                        ikLink.min_radian[2]
                    );
                    glm::vec3 limitMax(
                        ikLink.max_radian[0],
                        ikLink.max_radian[1],
                        ikLink.max_radian[2]
                    );
                    for (int d = 0; d < 3; ++d) {
                        if (limitMin[d] > limitMax[d]) {
                            std::swap(limitMin[d], limitMax[d]);
                        }
                    }
                    solver->AddIKChain(linkNode, true, limitMin, limitMax);
                }
                else
                {
                    solver->AddIKChain(linkNode, false,
                        glm::vec3(glm::radians(0.5f), 0.0f, 0.0f),
                        glm::vec3(glm::radians(180.0f), 0.0f, 0.0f));
                }
                linkNode->_enableIK = true;
            }
            currentBoneNode->_ikSolver = solver;
        }
    }

    for (int index = 0; index < _boneNodeByIdx.size(); ++index)
    {
        BoneNode* currentBoneNode = _boneNodeByIdx[index];
        int boneIndex = currentBoneNode->_boneIndex;
        int parentIndex = currentBoneNode->_parentBoneIndex;

        if (currentBoneNode->_parentBoneNode == nullptr) continue;

        glm::vec3 pos(
            bones[boneIndex].position[0],
            bones[boneIndex].position[1],
            bones[boneIndex].position[2]);

        glm::vec3 parentPos(
            bones[parentIndex].position[0],
            bones[parentIndex].position[1],
            bones[parentIndex].position[2]);

        glm::vec3 resultPos = pos - parentPos;
        currentBoneNode->_position = resultPos;
    }

    std::stable_sort(_sortedNodes.begin(), _sortedNodes.end(),
        [](const BoneNode* left, const BoneNode* right)
        {
            return left->_deformDepth < right->_deformDepth;
        });
}

void NodeManager::SortKey() {
    for (int index = 0; index < _boneNodeByIdx.size(); index++)
    {
        BoneNode* currentBoneNode = _boneNodeByIdx[index];
        currentBoneNode->SortAllKeys();
        _duration = (std::max)(_duration, currentBoneNode->GetMaxFrameNo());
    }
}

BoneNode* NodeManager::GetBoneNodeByIndex(int index) const {
    if (index < 0 || index >= static_cast<int>(_boneNodeByIdx.size()))
        return nullptr;
    return _boneNodeByIdx[index];
}

BoneNode* NodeManager::GetBoneNodeByName(std::wstring& name) const {
    auto it = _boneNodeByName.find(name);
    if (it != _boneNodeByName.end())
        return it->second;
    return nullptr;
}

void NodeManager::UpdateAnimation(float frameNo) {
    for (BoneNode* curNode : _boneNodeByIdx)
    {
        curNode->AnimateMotion(frameNo);
        curNode->AnimateIK(frameNo);
        curNode->UpdateLocalTransform();
    }

    if (_boneNodeByIdx.size() > 0)
    {
        _boneNodeByIdx[0]->UpdateGlobalTransform();
    }

    for (BoneNode* curNode : _sortedNodes)
    {
        IKSolver* curSolver = curNode->_ikSolver;
        if (curSolver != nullptr)
        {
            curSolver->Solve();
            curNode->UpdateGlobalTransform();
        }
    }
}

void NodeManager::Dispose() {
    for (BoneNode* node : _boneNodeByIdx)
    {
        delete node;
    }

    _boneNodeByIdx.clear();
    _boneNodeByName.clear();
    _sortedNodes.clear();
    _duration = 0;
}
