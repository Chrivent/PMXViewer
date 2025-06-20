#pragma once

#include <unordered_map>
#include "BoneNode.h"

class NodeManager
{
public:
	void Init(const std::vector<pmx::PmxBone>& bones);
	void SortKey();

	BoneNode* GetBoneNodeByIndex(int index) const;
	BoneNode* GetBoneNodeByName(std::wstring& name) const;

	const std::vector<BoneNode*>& GetAllNodes() const { return _boneNodeByIdx; }

	void UpdateAnimation(unsigned int frameNo);

	void Dispose();

private:
	std::unordered_map<std::wstring, BoneNode*> _boneNodeByName;
	std::vector<BoneNode*> _boneNodeByIdx;
	std::vector<BoneNode*> _sortedNodes;

	unsigned int _duration = 0;
};