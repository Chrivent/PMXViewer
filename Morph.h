#pragma once

#include <string>
#include <vector>
#include "Pmx.h"

class Morph
{
public:
	void SetPositionMorph(std::vector<pmx::PmxMorphVertexOffset>& pmxPositionMorphs);
	void SetUVMorph(std::vector<pmx::PmxMorphUVOffset>& pmxUVMorphs);
	void SetMaterialMorph(std::vector<pmx::PmxMorphMaterialOffset>& pmxMaterialMorphs);
	void SetBoneMorph(std::vector<pmx::PmxMorphBoneOffset>& pmxBoneMorphs);
	void SetGroupMorph(std::vector<pmx::PmxMorphGroupOffset>& pmxGroupMorphs);

	void SaveBaseAnimation() { _saveAnimWeight = _weight; }
	void LoadBaseAnimation() { _weight = _saveAnimWeight; }
	void ClearBaseAnimation() { _saveAnimWeight = 0; }
	float GetBaseAnimationWeight() const { return _saveAnimWeight; }

	std::wstring _name;
	float _weight = 0.f;
	float _saveAnimWeight = 0.f;
	pmx::MorphType _morphType;

private:
	std::vector<pmx::PmxMorphVertexOffset> _positionMorphData;
	std::vector<pmx::PmxMorphUVOffset> _uvMorphData;
	std::vector<pmx::PmxMorphMaterialOffset> _materialMorphData;
	std::vector<pmx::PmxMorphBoneOffset> _boneMorphData;
	std::vector<pmx::PmxMorphGroupOffset> _groupMorphData;
};