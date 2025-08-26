#include "MorphManager.h"

#include <glm/gtc/type_ptr.hpp>
#include <algorithm>
#include <chrono>
#include <execution>
#include "Pmx.h"

Morph::Morph()
{
	_weight = 0.f;
	_saveAnimWeight = 0.f;
	_morphType = pmx::MorphType::Group;
}

void Morph::SetPositionMorph(std::vector<pmx::PmxMorphVertexOffset> pmxPositionMorphs, size_t vertexBufferSize)
{
	_positionMorphData = std::move(pmxPositionMorphs);
	_posCached.clear();
	_posCached.reserve(_positionMorphData.size());

	for (auto& x : _positionMorphData) {
		// 음수 인덱스는 unsigned로 말려올 수 있으니, 여기서 바로 거르려면 이렇게:
		if (x.vertex_index < 0) continue;
		_posCached.push_back({ x.vertex_index,
							   glm::make_vec3(x.position_offset) });
	}

	// 1) 범위 밖 제거
	_posCached.erase(
		std::remove_if(_posCached.begin(), _posCached.end(),
			[vertexBufferSize](const PosOffsetCached& d) {
				return d.idx >= vertexBufferSize;
			}),
		_posCached.end());

	// 2) 정렬
	std::sort(_posCached.begin(), _posCached.end(),
		[](const PosOffsetCached& a, const PosOffsetCached& b) {
			return a.idx < b.idx;
		});

	// 3) 중복 idx 머지
	std::vector<PosOffsetCached> merged;
	merged.reserve(_posCached.size());
	for (size_t i = 0; i < _posCached.size(); ) {
		const int idx = _posCached[i].idx;
		glm::vec3 acc(0.0f);
		do { acc += _posCached[i].delta; ++i; } while (i < _posCached.size() && _posCached[i].idx == idx);
		merged.push_back({ idx, acc });
	}
	_posCached.swap(merged);
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

void MorphManager::Init(const pmx::PmxMorph* pmxMorphs,
	int pmxMorphCount,
	const std::vector<vmd::VmdFaceFrame>& vmdMorphs,
	unsigned int vertexCount,
	unsigned int materialCount,
	unsigned int boneCount)
{
	// 컨테이너 클리어 & 용량 준비
	_morphs.clear();
	_morphByName.clear();
	_morphs.resize(static_cast<size_t>(pmxMorphCount));

	for (int i = 0; i < pmxMorphCount; ++i)
	{
		Morph& cur = _morphs[static_cast<size_t>(i)];
		const pmx::PmxMorph& src = pmxMorphs[i];

		cur._name = src.morph_name;   // pmx::PmxMorph의 이름이 wide라면 그대로 복사
		cur._weight = 0.0f;
		cur._morphType = src.morph_type;

		switch (cur._morphType)
		{
		case pmx::MorphType::Vertex:
		{
			std::vector<pmx::PmxMorphVertexOffset> v;
			if (src.offset_count > 0 && src.vertex_offsets) {
				v.assign(src.vertex_offsets.get(),
					src.vertex_offsets.get() + src.offset_count);
			}
			cur.SetPositionMorph(std::move(v), vertexCount);
			break;
		}
		case pmx::MorphType::UV:
		{
			std::vector<pmx::PmxMorphUVOffset> v;
			if (src.offset_count > 0 && src.uv_offsets) {
				v.assign(src.uv_offsets.get(),
					src.uv_offsets.get() + src.offset_count);
			}
			cur.SetUVMorph(std::move(v));
			break;
		}
		case pmx::MorphType::Material: // (enum 철자 확인: Material/Matrial 프로젝트 정의에 맞추기)
		{
			std::vector<pmx::PmxMorphMaterialOffset> v;
			if (src.offset_count > 0 && src.material_offsets) {
				v.assign(src.material_offsets.get(),
					src.material_offsets.get() + src.offset_count);
			}
			cur.SetMaterialMorph(std::move(v));
			break;
		}
		case pmx::MorphType::Bone:
		{
			std::vector<pmx::PmxMorphBoneOffset> v;
			if (src.offset_count > 0 && src.bone_offsets) {
				v.assign(src.bone_offsets.get(),
					src.bone_offsets.get() + src.offset_count);
			}
			cur.SetBoneMorph(std::move(v));
			break;
		}
		case pmx::MorphType::Group:
		{
			std::vector<pmx::PmxMorphGroupOffset> v;
			if (src.offset_count > 0 && src.group_offsets) {
				v.assign(src.group_offsets.get(),
					src.group_offsets.get() + src.offset_count);
			}
			cur.SetGroupMorph(std::move(v));
			break;
		}
		default:
			break;
		}

		_morphByName[cur._name] = &cur;
	}

	// VMD 키 복사 및 이름 매핑
	_morphKeys = vmdMorphs;
	_morphKeyByName.clear();

	for (auto& k : _morphKeys)
	{
		std::wstring wname;
		oguna::EncodingConverter{}.Cp932ToUtf16(
			k.face_name.c_str(),
			static_cast<int>(k.face_name.size()),
			&wname);

		auto it = _morphByName.find(wname);
		if (it == _morphByName.end()) continue;
		_morphKeyByName[wname].push_back(&k);
	}

	// 키프레임 정렬(프레임 오름차순)
	for (auto& kv : _morphKeyByName)
	{
		auto& arr = kv.second;
		if (arr.size() <= 1) continue;
		std::sort(arr.begin(), arr.end(),
			[](const vmd::VmdFaceFrame* a, const vmd::VmdFaceFrame* b)
			{ return a->frame < b->frame; });
	}

	// 누적 버퍼 초기화(0으로)
	_morphVertexPosition.assign(vertexCount, glm::vec3(0.0f));
	_morphUV.assign(vertexCount, glm::vec4(0.0f));
	_morphMaterial.assign(materialCount, MaterialMorphData{});
	_morphBone.assign(boneCount, BoneMorphData{});
}

void MorphManager::Animate(float frame)
{
	using clock = std::chrono::steady_clock;
	using dsec = std::chrono::duration<double>;

	auto t0 = clock::now();

	// ===== 시간 누적용 static 변수 =====
	static double accReset = 0.0;  // ResetMorphData 시간 누적
	static double accTotal = 0.0;
	static double accPos = 0.0;
	static double accUV = 0.0;
	static double accMat = 0.0;
	static double accBone = 0.0;
	static double accGroup = 0.0;
	static int    frames = 0;
	static auto   lastPrint = clock::now();

	// --- ResetMorphData 계측 ---
	auto r0 = clock::now();
	ResetMorphData();
	auto r1 = clock::now();
	accReset += std::chrono::duration_cast<dsec>(r1 - r0).count();

	// --- 키 보간 ---
	for (auto& morphKey : _morphKeyByName) {
		auto morphIt = _morphByName.find(morphKey.first);
		if (morphIt == _morphByName.end()) continue;

		auto rit = std::find_if(morphKey.second.rbegin(), morphKey.second.rend(),
			[frame](const vmd::VmdFaceFrame* morph) {
				return morph->frame <= frame;
			});

		auto iterator = rit.base();

		if (iterator == morphKey.second.end()) {
			morphIt->second->_weight = 0.0f;
		}
		else {
			float t = static_cast<float>(frame - (*rit)->frame) /
				static_cast<float>((*iterator)->frame - (*rit)->frame);
			morphIt->second->_weight =
				std::lerp((*rit)->weight, (*iterator)->weight, t);
		}
	}

	// --- 모프 적용 ---
	for (Morph& morph : _morphs) {
		auto start = clock::now();
		AnimateMorph(morph);
		auto end = clock::now();

		double dt = std::chrono::duration_cast<dsec>(end - start).count();
		switch (morph._morphType) {
		case pmx::MorphType::Vertex:   accPos += dt; break;
		case pmx::MorphType::UV:       accUV += dt; break;
		case pmx::MorphType::Material: accMat += dt; break;
		case pmx::MorphType::Bone:     accBone += dt; break;
		case pmx::MorphType::Group:    accGroup += dt; break;
		default: break;
		}
	}

	auto t1 = clock::now();
	accTotal += std::chrono::duration_cast<dsec>(t1 - t0).count();
	frames += 1;

	// ===== 1초마다 출력 =====
	auto now = clock::now();
	if (std::chrono::duration_cast<dsec>(now - lastPrint).count() >= 1.0) {
		const double secs = std::chrono::duration_cast<dsec>(now - lastPrint).count();
		const double fps = frames / secs;
		auto avg_ms = [&](double acc) { return (acc / frames) * 1000.0; };

		std::wcerr << std::fixed << std::setprecision(2)
			<< L"[MorphManager] FPS: " << fps
			<< L" | avg ms  total:" << avg_ms(accTotal)
			<< L"  reset:" << avg_ms(accReset)   // ★ 추가된 부분
			<< L"  pos:" << avg_ms(accPos)
			<< L"  uv:" << avg_ms(accUV)
			<< L"  mat:" << avg_ms(accMat)
			<< L"  bone:" << avg_ms(accBone)
			<< L"  group:" << avg_ms(accGroup)
			<< L" (" << frames << L" frames / " << secs << L"s)\n";

		accReset = accTotal = accPos = accUV = accMat = accBone = accGroup = 0.0;
		frames = 0;
		lastPrint = now;
	}
}

const glm::vec3& MorphManager::GetMorphVertexPosition(unsigned int index) const
{
	return _morphVertexPosition[index];
}

const glm::vec4& MorphManager::GetMorphUV(unsigned int index) const
{
	return _morphUV[index];
}

const MaterialMorphData& MorphManager::GetMorphMaterial(unsigned int index) const
{
	return _morphMaterial[index];
}

const BoneMorphData& MorphManager::GetMorphBone(unsigned int index) const
{
	return _morphBone[index];
}

void MorphManager::ResetMorphData()
{
	// pos
	std::for_each(std::execution::par,
		_morphVertexPosition.begin(), _morphVertexPosition.end(),
		[](glm::vec3& v) { v = glm::vec3(0); });

	// uv
	std::for_each(std::execution::par,
		_morphUV.begin(), _morphUV.end(),
		[](glm::vec4& v) { v = glm::vec4(0); });

	// material
	std::for_each(std::execution::par,
		_morphMaterial.begin(), _morphMaterial.end(),
		[](MaterialMorphData& m) { m = MaterialMorphData{}; });

	// bone
	std::for_each(std::execution::par,
		_morphBone.begin(), _morphBone.end(),
		[](BoneMorphData& b) { b = BoneMorphData{}; });
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
		AnimateMaterialMorph(morph, weight);
	}
	break;
	case pmx::MorphType::Bone:
	{
		AnimateBoneMorph(morph, weight);
	}
	break;
	case pmx::MorphType::Group:
	{
		AnimateGroupMorph(morph, weight);
	}
	break;
	}
}

void MorphManager::AnimatePositionMorph(Morph& morph, float weight)
{
	const auto& A = morph._posCached;
	if (A.empty()) return;

	const float w = morph._weight * weight;
	// 1) weight 스킵 (분기/연산 모두 제거)
	if (std::abs(w) <= 1e-8f) return;

	glm::vec3* __restrict out = _morphVertexPosition.data();

	// 3) 연속 인덱스(run) 처리 + 4) 크기 기준 하이브리드
	constexpr size_t kParallelThreshold = 4096;

	if (A.size() < kParallelThreshold) {
		// --- 작은 경우: 직렬 + run 처리 (캐시 친화적, 분기 최소화) ---
		const PosOffsetCached* p = A.data();
		const glm::vec3 ww(w);           // 스칼라 곱 최소화
		size_t i = 0, n = A.size();
		while (i < n) {
			uint32_t idx0 = p[i].idx;
			size_t j = i + 1;
			// 연속 인덱스 구간 길이 찾기
			while (j < n && p[j].idx == idx0 + (j - i)) ++j;

			// 연속 구간 일괄 처리 (분기/인덱스 계산 감소)
			glm::vec3* __restrict dst = out + idx0;
			for (size_t k = 0; k < (j - i); ++k) {
				dst[k] += p[i + k].delta * ww;
			}
			i = j;
		}
	}
	else {
		// --- 큰 경우: 병렬 ---
		// 중복 idx는 Init에서 이미 머지했으므로 레이스 없음
		std::for_each(std::execution::par, A.begin(), A.end(),
			[&](const PosOffsetCached& d) {
				out[d.idx] += d.delta * w;
			});
	}
}

void MorphManager::AnimateUVMorph(Morph& morph, float weight)
{
	const auto& uvMorph = morph.GetUVMorphData();

	for (const auto& data : uvMorph)
	{
		if (data.vertex_index >= _morphUV.size())
			continue;

		glm::vec4 morphUV = glm::make_vec4(data.uv_offset);
		glm::vec4 originUV = _morphUV[data.vertex_index];

		_morphUV[data.vertex_index] = originUV + morphUV * (morph._weight * weight);
	}
}

void MorphManager::AnimateMaterialMorph(Morph& morph, float weight)
{
	const auto& materialMorph = morph.GetMaterialMorphData();

	for (const auto& data : materialMorph)
	{
		if (static_cast<size_t>(data.material_index) >= _morphMaterial.size())
			continue;

		MaterialMorphData& cur = _morphMaterial[data.material_index];
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
}

void MorphManager::AnimateBoneMorph(Morph& morph, float weight)
{
	const auto& boneMorphs = morph.GetBoneMorphData();

	for (const auto& data : boneMorphs)
	{
		_morphBone[data.bone_index].weight = morph._weight * weight;
		_morphBone[data.bone_index].position = glm::vec3(data.translation[0], data.translation[1], data.translation[2]);
		// PMX stores rotation as (x, y, z, w). glm::quat ctor is (w, x, y, z).
		_morphBone[data.bone_index].quaternion = glm::quat(
			data.rotation[3],  // w
			data.rotation[0],  // x
			data.rotation[1],  // y
			data.rotation[2]   // z
		);
	}
}

void MorphManager::AnimateGroupMorph(Morph& morph, float weight)
{
	const auto& groupMorphs = morph.GetGroupMorphData();

	for (const auto& data : groupMorphs)
	{
		if (static_cast<size_t>(data.morph_index) >= _morphs.size())
			continue;

		AnimateMorph(_morphs[data.morph_index], morph._weight * weight);
	}
}
