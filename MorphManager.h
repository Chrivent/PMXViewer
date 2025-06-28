#pragma once

#include <string>
#include <vector>
#include <unordered_map>
#include <glm/gtc/quaternion.hpp>
#include "Pmx.h"
#include "Vmd.h"

class Morph
{
public:
	Morph();

	void SetPositionMorph(std::vector<pmx::PmxMorphVertexOffset> pmxPositionMorphs);
	void SetUVMorph(std::vector<pmx::PmxMorphUVOffset> pmxUVMorphs);
	void SetMaterialMorph(std::vector<pmx::PmxMorphMaterialOffset> pmxMaterialMorphs);
	void SetBoneMorph(std::vector<pmx::PmxMorphBoneOffset> pmxBoneMorphs);
	void SetGroupMorph(std::vector<pmx::PmxMorphGroupOffset> pmxGroupMorphs);

	const std::vector<pmx::PmxMorphVertexOffset>& GetPositionMorphData() const { return _positionMorphData; }
	const std::vector<pmx::PmxMorphUVOffset>& GetUVMorphData() const { return _uvMorphData; }
	const std::vector<pmx::PmxMorphMaterialOffset>& GetMaterialMorphData() const { return _materialMorphData; }
	const std::vector<pmx::PmxMorphBoneOffset>& GetBoneMorphData() const { return _boneMorphData; }
	const std::vector<pmx::PmxMorphGroupOffset>& GetGroupMorphData() const { return _groupMorphData; }

	void SaveBaseAnimation();
	void LoadBaseAnimation();
	void ClearBaseAnimation();
	float GetBaseAnimationWeight() const;

	std::wstring _name;
	float _weight;
	pmx::MorphType _morphType;

private:
	float _saveAnimWeight = 0.f;

	std::vector<pmx::PmxMorphVertexOffset> _positionMorphData;
	std::vector<pmx::PmxMorphUVOffset> _uvMorphData;
	std::vector<pmx::PmxMorphMaterialOffset> _materialMorphData;
	std::vector<pmx::PmxMorphBoneOffset> _boneMorphData;
	std::vector<pmx::PmxMorphGroupOffset> _groupMorphData;
};

class MorphManager
{
public:
	void Init(const std::vector<pmx::PmxMorph>& pmxMorphs, const std::vector<vmd::VmdFaceFrame>& vmdMorphs, unsigned int vertexCount, unsigned int materialCount, unsigned int boneCount);

	void Animate(float frame);

	/*const pmx::PmxMorphVertexOffset& GetMorphVertexPosition(unsigned int index) const;
	const pmx::PmxMorphUVOffset& GetMorphUV(unsigned int index) const;
	const pmx::PmxMorphMaterialOffset& GetMorphMaterial(unsigned int index) const;
	const pmx::PmxMorphBoneOffset& GetMorphBone(unsigned int index) const;*/

private:
	void AnimateMorph(Morph& morph, float weight = 1.f);
	void AnimatePositionMorph(Morph& morph, float weight);
	void AnimateUVMorph(Morph& morph, float weight);
	/*void AnimateMaterialMorph(Morph& morph, float weight);
	void AnimateBoneMorph(Morph& morph, float weight);
	void AnimateGroupMorph(Morph& morph, float weight);*/

	void ResetMorphData();

	std::vector<Morph> _morphs;
	std::unordered_map<std::wstring, Morph*> _morphByName;

	std::vector<vmd::VmdFaceFrame> _morphKeys;
	std::unordered_map<std::wstring, std::vector<vmd::VmdFaceFrame*>> _morphKeyByName;

	std::vector<pmx::PmxMorphVertexOffset> _morphVertexPosition;
	std::vector<pmx::PmxMorphUVOffset> _morphUV;
	std::vector<pmx::PmxMorphMaterialOffset> _morphMaterial;
	std::vector<pmx::PmxMorphBoneOffset> _morphBone;
};