#include "NodeManager.h"

#include <glm/gtc/type_ptr.hpp>
#include <algorithm>

void NodeManager::Init(const std::vector<pmx::PmxBone>& bones)
{
    _boneNodeByIdx.resize(bones.size());
    _sortedNodes.resize(bones.size());

    for (int index = 0; index < bones.size(); index++)
    {
        const pmx::PmxBone& currentBoneData = bones[index];
        _boneNodeByIdx[index] = new BoneNode(index, currentBoneData);
        _boneNodeByName[_boneNodeByIdx[index]->GetName()] = _boneNodeByIdx[index];
        _sortedNodes[index] = _boneNodeByIdx[index];
    }

    // 부모 본 설정
    for (int index = 0; index < _boneNodeByIdx.size(); index++)
    {
        BoneNode* currentBoneNode = _boneNodeByIdx[index];
        unsigned int parentBoneIndex = currentBoneNode->GetParentBoneIndex();

        if (parentBoneIndex != 65535 && parentBoneIndex < _boneNodeByIdx.size())
        {
            currentBoneNode->SetParentBoneNode(_boneNodeByIdx[parentBoneIndex]);
        }
    }

    // 포지션 오프셋 계산
    for (int index = 0; index < _boneNodeByIdx.size(); index++)
    {
        BoneNode* currentBoneNode = _boneNodeByIdx[index];

        if (currentBoneNode->GetParentBoneNode() == nullptr)
            continue;

        glm::vec3 pos = glm::make_vec3(bones[currentBoneNode->GetBoneIndex()].position);
        glm::vec3 parentPos = glm::make_vec3(bones[currentBoneNode->GetParentBoneIndex()].position);
        glm::vec3 resultPos = pos - parentPos;

        currentBoneNode->SetPosition(resultPos);
    }

    // Deform Depth 기준으로 정렬
    std::stable_sort(_sortedNodes.begin(), _sortedNodes.end(),
        [](const BoneNode* left, const BoneNode* right) {
            return left->GetDeformDepth() < right->GetDeformDepth();
        });
}

void NodeManager::SortKey()
{
    for (int index = 0; index < _boneNodeByIdx.size(); index++)
    {
        BoneNode* currentBoneNode = _boneNodeByIdx[index];
        currentBoneNode->SortAllKeys();
        _duration = std::max(_duration, currentBoneNode->GetMaxFrameNo());
    }
}

void NodeManager::UpdateAnimation(unsigned int frameNo)
{
    for (BoneNode* curNode : _boneNodeByIdx)
    {
        curNode->AnimateMotion(frameNo);
        curNode->UpdateLocalTransform();
    }

    if (_boneNodeByIdx.size() > 0)
    {
        _boneNodeByIdx[0]->UpdateGlobalTransform();
    }
}
