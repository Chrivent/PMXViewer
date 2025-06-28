#include "MorphManager.h"

#include <algorithm>
#include "Pmx.h"

Morph::Morph()
{
	_weight = 0.f;
	_saveAnimWeight = 0.f;
	_morphType = pmx::MorphType::Group;
}

void Morph::SetPositionMorph(std::vector<pmx::PmxMorphVertexOffset> pmxPositionMorphs)
{
	_positionMorphData = std::move(pmxPositionMorphs);
}

void Morph::SetUVMorph(std::vector<pmx::PmxMorphUVOffset> pmxUVMorphs)
{
	_uvMorphData = std::move(pmxUVMorphs);
}

void Morph::SetMaterialMorph(std::vector<pmx::PmxMorphMaterialOffset> pmxMaterialMorphs)
{
	_materialMorphData = std::move(pmxMaterialMorphs);
}

void Morph::SetBoneMorph(std::vector<pmx::PmxMorphBoneOffset> pmxBoneMorphs)
{
	_boneMorphData = std::move(pmxBoneMorphs);
}

void Morph::SetGroupMorph(std::vector<pmx::PmxMorphGroupOffset> pmxGroupMorphs)
{
	_groupMorphData = std::move(pmxGroupMorphs);
}

void Morph::SaveBaseAnimation()
{
	_saveAnimWeight = _weight;
}

void Morph::LoadBaseAnimation()
{
	_weight = _saveAnimWeight;
}

void Morph::ClearBaseAnimation()
{
	_saveAnimWeight = 0;
}

float Morph::GetBaseAnimationWeight() const
{
	return _saveAnimWeight;
}

void MorphManager::Init(const std::vector<pmx::PmxMorph>& pmxMorphs, const std::vector<vmd::VmdFaceFrame>& vmdMorphs, unsigned int vertexCount, unsigned int materialCount, unsigned int boneCount)
{
	_morphs.resize(pmxMorphs.size());

	for (unsigned int index = 0; index < pmxMorphs.size(); index++)
	{
		Morph& currentMorph = _morphs[index];
		const pmx::PmxMorph& currentPMXMorph = pmxMorphs[index];
		currentMorph._name = currentPMXMorph.morph_name;
		currentMorph._weight = 0.0f;
		currentMorph._morphType = currentPMXMorph.morph_type;

		switch (currentMorph._morphType)
		{
		case pmx::MorphType::Vertex:
		{
			std::vector<pmx::PmxMorphVertexOffset> offsets(
				currentPMXMorph.vertex_offsets.get(),
				currentPMXMorph.vertex_offsets.get() + currentPMXMorph.offset_count
			);
			currentMorph.SetPositionMorph(offsets);
		}
		break;
		case pmx::MorphType::UV:
		{
			std::vector<pmx::PmxMorphUVOffset> uvOffsets(
				currentPMXMorph.uv_offsets.get(),
				currentPMXMorph.uv_offsets.get() + currentPMXMorph.offset_count
			);
			currentMorph.SetUVMorph(std::move(uvOffsets));
		}
		break;

		case pmx::MorphType::Material:
		{
			std::vector<pmx::PmxMorphMaterialOffset> materialOffsets(
				currentPMXMorph.material_offsets.get(),
				currentPMXMorph.material_offsets.get() + currentPMXMorph.offset_count
			);
			currentMorph.SetMaterialMorph(std::move(materialOffsets));
		}
		break;

		case pmx::MorphType::Bone:
		{
			std::vector<pmx::PmxMorphBoneOffset> boneOffsets(
				currentPMXMorph.bone_offsets.get(),
				currentPMXMorph.bone_offsets.get() + currentPMXMorph.offset_count
			);
			currentMorph.SetBoneMorph(std::move(boneOffsets));
		}
		break;

		case pmx::MorphType::Group:
		{
			std::vector<pmx::PmxMorphGroupOffset> groupOffsets(
				currentPMXMorph.group_offsets.get(),
				currentPMXMorph.group_offsets.get() + currentPMXMorph.offset_count
			);
			currentMorph.SetGroupMorph(std::move(groupOffsets));
		}
		break;
		}

		_morphByName[currentMorph._name] = &currentMorph;
	}

	_morphKeys.resize(vmdMorphs.size());
	for (int i = 0; i < vmdMorphs.size(); i++)
	{
		_morphKeys[i] = vmdMorphs[i];

		std::wstring name;
		oguna::EncodingConverter{}.Cp932ToUtf16(_morphKeys[i].face_name.c_str(), static_cast<int>(_morphKeys[i].face_name.size()), &name);
		if (_morphByName.find(name) == _morphByName.end())
		{
			continue;
		}

		_morphKeyByName[name].push_back(&_morphKeys[i]);
	}

	for (auto& morphKey : _morphKeyByName)
	{
		std::vector<vmd::VmdFaceFrame*>& morphKeys = morphKey.second;

		if (morphKeys.size() <= 1)
		{
			continue;
		}

		std::sort(morphKeys.begin(), morphKeys.end(),
			[](const vmd::VmdFaceFrame* left, const vmd::VmdFaceFrame* right)
			{
				if (left->frame == right->frame)
				{
					return false;
				}

				return left->frame < right->frame;
			});
	}

	_morphVertexPosition.resize(vertexCount);
	_morphUV.resize(vertexCount);
	_morphMaterial.resize(materialCount);
	_morphBone.resize(boneCount);
}

void MorphManager::Animate(float frame)
{
	ResetMorphData();

	for (auto& morphKey : _morphKeyByName)
	{
		auto morphIt = _morphByName.find(morphKey.first);
		if (morphIt == _morphByName.end())
		{
			continue;
		}

		auto rit = std::find_if(morphKey.second.rbegin(), morphKey.second.rend(),
			[frame](const vmd::VmdFaceFrame* morph)
			{
				return morph->frame <= frame;
			});

		auto iterator = rit.base();

		if (iterator == morphKey.second.end())
		{
			morphIt->second->_weight = 0.0f;
		}
		else
		{
			float t = static_cast<float>(frame - (*rit)->frame) / static_cast<float>((*iterator)->frame - (*rit)->frame);
			morphIt->second->_weight = std::lerp((*rit)->weight, (*iterator)->weight, t);
		}
	}

	for (Morph& morph : _morphs)
	{
		AnimateMorph(morph);
	}
}

void MorphManager::ResetMorphData()
{
	for (auto& pos : _morphVertexPosition)
	{
		pos.position_offset[0] = 0.f;
		pos.position_offset[1] = 0.f;
		pos.position_offset[2] = 0.f;
	}

	for (auto& uv : _morphUV)
	{
		uv.uv_offset[0] = 0.f;
		uv.uv_offset[1] = 0.f;
		uv.uv_offset[2] = 0.f;
		uv.uv_offset[3] = 0.f;
	}

	for (auto& material : _morphMaterial)
	{
		material.offset_operation = 0;
		for (int i = 0; i < 4; ++i) {
			material.diffuse[i] = 0.f;
			material.edge_color[i] = 0.f;
			material.texture_argb[i] = 0.f;
			material.sphere_texture_argb[i] = 0.f;
			material.toon_texture_argb[i] = 0.f;
		}
		for (int i = 0; i < 3; ++i) {
			material.specular[i] = 0.f;
			material.ambient[i] = 0.f;
		}
		material.specularity = 0.f;
		material.edge_size = 0.f;
	}

	for (auto& bone : _morphBone)
	{
		for (int i = 0; i < 3; ++i)
			bone.translation[i] = 0.f;
		for (int i = 0; i < 4; ++i)
			bone.rotation[i] = 0.f;
	}
}

void MorphManager::AnimateMorph(Morph& morph, float weight)
{
	switch (morph._morphType)
	{
	case pmx::MorphType::Vertex:
	{
		AnimatePositionMorph(morph, weight);
	}
	break;
	case pmx::MorphType::UV:
	{
		AnimateUVMorph(morph, weight);
	}
	break;
	case pmx::MorphType::Material:
	{
		//AnimateMaterialMorph(morph, weight);
	}
	break;
	case pmx::MorphType::Bone:
	{
		//AnimateBoneMorph(morph, weight);
	}
	break;
	case pmx::MorphType::Group:
	{
		//AnimateGroupMorph(morph, weight);
	}
	break;
	}
}

void MorphManager::AnimatePositionMorph(Morph& morph, float weight)
{
	const auto& vertexPositionMorph = morph.GetPositionMorphData();

	for (const auto& data : vertexPositionMorph)
	{
		if (data.vertex_index >= _morphVertexPosition.size())
			continue;

		glm::vec3 originPosition(
			_morphVertexPosition[data.vertex_index].position_offset[0],
			_morphVertexPosition[data.vertex_index].position_offset[1],
			_morphVertexPosition[data.vertex_index].position_offset[2]
			);
		glm::vec3 morphOffset(
			data.position_offset[0],
			data.position_offset[1],
			data.position_offset[2]
			);
		glm::vec3 result = originPosition + morphOffset * morph._weight * weight;

		_morphVertexPosition[data.vertex_index].position_offset[0] = result.x;
		_morphVertexPosition[data.vertex_index].position_offset[1] = result.y;
		_morphVertexPosition[data.vertex_index].position_offset[2] = result.z;
	}
}

void MorphManager::AnimateUVMorph(Morph& morph, float weight)
{
	const auto& uvMorph = morph.GetUVMorphData();

	for (const auto& data : uvMorph)
	{
		if (data.vertex_index >= _morphUV.size())
			continue;

		glm::vec4 morphUV(
			data.uv_offset[0],
			data.uv_offset[1],
			data.uv_offset[2],
			data.uv_offset[3]
		);
		glm::vec4 originUV(
			_morphUV[data.vertex_index].uv_offset[0],
			_morphUV[data.vertex_index].uv_offset[1],
			_morphUV[data.vertex_index].uv_offset[2],
			_morphUV[data.vertex_index].uv_offset[3]
		);

		originUV += morphUV * morph._weight * weight;
	}
}

/*void MorphManager::AnimateMaterialMorph(Morph& morph, float weight)
{
	const auto& materialMorph = morph.GetMaterialMorphData();

	for (const auto& data : materialMorph)
	{
		if (data.material_index >= _morphMaterial.size())
			continue;

		auto& cur = _morphMaterial[data.material_index];
		cur.weight = morph._weight * weight;
		cur.opType = data.offset_operation;
		cur.diffuse = glm::vec4(data.diffuse[0], data.diffuse[1], data.diffuse[2], data.diffuse[3]);
		cur.specular = glm::vec3(data.specular[0], data.specular[1], data.specular[2]);
		cur.specularPower = data.specularity;
		cur.ambient = glm::vec3(data.ambient[0], data.ambient[1], data.ambient[2]);
		cur.edgeColor = glm::vec4(data.edge_color[0], data.edge_color[1], data.edge_color[2], data.edge_color[3]);
		cur.edgeSize = data.edge_size;
		cur.textureFactor = glm::vec4(data.texture_argb[0], data.texture_argb[1], data.texture_argb[2], data.texture_argb[3]);
		cur.sphereTextureFactor = glm::vec4(data.sphere_texture_argb[0], data.sphere_texture_argb[1], data.sphere_texture_argb[2], data.sphere_texture_argb[3]);
		cur.toonTextureFactor = glm::vec4(data.toon_texture_argb[0], data.toon_texture_argb[1], data.toon_texture_argb[2], data.toon_texture_argb[3]);
	}
}*/
