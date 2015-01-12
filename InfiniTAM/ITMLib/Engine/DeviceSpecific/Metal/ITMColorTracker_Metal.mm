// Copyright 2014 Isis Innovation Limited and the authors of InfiniTAM

#ifndef COMPILE_WITHOUT_METAL

#include "ITMColorTracker_Metal.h"

using namespace ITMLib::Engine;

ITMColorTracker_Metal::ITMColorTracker_Metal(Vector2i imgSize, int noHierarchyLevels, int noRotationOnlyLevels, ITMLowLevelEngine *lowLevelEngine) 
	: ITMColorTracker(imgSize, noHierarchyLevels, noRotationOnlyLevels, lowLevelEngine, false) {  }

ITMColorTracker_Metal::~ITMColorTracker_Metal(void) { }

void ITMColorTracker_Metal::F_oneLevel(float *f, ITMPose *pose)
{
}

void ITMColorTracker_Metal::G_oneLevel(float *gradient, float *hessian, ITMPose *pose) const
{
	
}

#endif