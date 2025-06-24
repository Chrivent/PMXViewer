#pragma once

#include <string>
#include <vector>
#include "Pmx.h"

class Morph
{
public:
	void SaveBaseAnimation();
	void LoadBaseAnimation();
	void ClearBaseAnimation();
	float GetBaseAnimationWeight() const;

	float _weight = 0.f;
	float _saveAnimWeight = 0.f;
	pmx::PmxMorph _morph;
};

class MorphManager
{

};