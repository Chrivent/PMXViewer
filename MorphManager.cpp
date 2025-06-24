#include "MorphManager.h"

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
