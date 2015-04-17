// Copyright 2014 Isis Innovation Limited and the authors of InfiniTAM

#include "PoseDatabase.h"

using namespace LCDLib;

PoseDatabase::PoseDatabase(void)
{}

PoseDatabase::~PoseDatabase(void)
{}

void PoseDatabase::storePose(int id, const ITMLib::ITMPose & pose)
{
	if (id < 0) return;
	if ((unsigned)id >= mPoses.size()) mPoses.resize(id+1);

	mPoses[id] = pose;
}

const ITMLib::ITMPose & PoseDatabase::retrievePose(int id) const
{
	return mPoses[id];
}

ITMLib::ITMPose PoseDatabase::retrieveWAPose(int k, int ids[], float distances[]) const
{
	ORUtils::Matrix4<float> m;
	m.setZeros();

	float sumWeights = 0.0f;
	for (int i = 0; i < k; ++i) {
		float weight = 1.0f - distances[i];
		m += retrievePose(ids[i]).GetM() * weight;
		sumWeights += weight;
	}

	m = m * (1.0f/sumWeights);
	return ITMLib::ITMPose(m);
}

