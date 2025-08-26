#pragma once

#include <string>
#include <vector>
#include <unordered_map>
#include <glm/gtc/quaternion.hpp>
#include "Pmx.h"
#include "Vmd.h"

struct PosOffsetCached
{
	int idx;
	glm::vec3 delta;
};

class Morph
{
public:
	Morph();

	void SetPositionMorph(std::vector<pmx::PmxMorphVertexOffset> pmxPositionMorphs, size_t vertexBufferSize);
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
	std::vector<PosOffsetCached> _posCached;

private:
	float _saveAnimWeight = 0.f;

	std::vector<pmx::PmxMorphVertexOffset> _positionMorphData;
	std::vector<pmx::PmxMorphUVOffset> _uvMorphData;
	std::vector<pmx::PmxMorphMaterialOffset> _materialMorphData;
	std::vector<pmx::PmxMorphBoneOffset> _boneMorphData;
	std::vector<pmx::PmxMorphGroupOffset> _groupMorphData;
};

struct MaterialMorphData
{
	float weight = 0.0f;
	uint8_t opType = 0;
	glm::vec4 diffuse{ 0.0f };
	glm::vec3 specular{ 0.0f };
	float     specularPower = 0.0f;
	glm::vec3 ambient{ 0.0f };
	glm::vec4 edgeColor{ 0.0f };
	float     edgeSize = 0.0f;
	glm::vec4 textureFactor{ 0.0f };
	glm::vec4 sphereTextureFactor{ 0.0f };
	glm::vec4 toonTextureFactor{ 0.0f };
};

struct BoneMorphData
{
	float     weight = 0.0f;
	glm::vec3 position{ 0.0f };
	glm::quat quaternion{ 1,0,0,0 };
};

class MorphManager
{
public:
	void Init(const pmx::PmxMorph* pmxMorphs,
		int pmxMorphCount,
		const std::vector<vmd::VmdFaceFrame>& vmdMorphs,
		unsigned int vertexCount,
		unsigned int materialCount,
		unsigned int boneCount);

	void Animate(float frame);

	const glm::vec3& GetMorphVertexPosition(unsigned int index) const;
	const glm::vec4& GetMorphUV(unsigned int index) const;
	const MaterialMorphData& GetMorphMaterial(unsigned int index) const;
	const BoneMorphData& GetMorphBone(unsigned int index) const;

private:
	void AnimateMorph(Morph& morph, float weight = 1.f);
	void AnimatePositionMorph(Morph& morph, float weight);
	void AnimateUVMorph(Morph& morph, float weight);
	void AnimateMaterialMorph(Morph& morph, float weight);
	void AnimateBoneMorph(Morph& morph, float weight);
	void AnimateGroupMorph(Morph& morph, float weight);

	void ResetMorphData();

	std::vector<Morph> _morphs;
	std::unordered_map<std::wstring, Morph*> _morphByName;

	std::vector<vmd::VmdFaceFrame> _morphKeys;
	std::unordered_map<std::wstring, std::vector<vmd::VmdFaceFrame*>> _morphKeyByName;

	std::vector<glm::vec3> _morphVertexPosition;
	std::vector<glm::vec4> _morphUV;
	std::vector<MaterialMorphData> _morphMaterial;
	std::vector<BoneMorphData> _morphBone;
};