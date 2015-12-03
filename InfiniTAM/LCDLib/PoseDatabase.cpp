// Copyright 2014 Isis Innovation Limited and the authors of InfiniTAM

#include "PoseDatabase.h"

using namespace LCDLib;

PoseDatabase::PoseDatabase(void)
{}

PoseDatabase::~PoseDatabase(void)
{}

void PoseDatabase::storePose(int id, const ITMLib::ITMPose & pose, int sceneId)
{
	if (id < 0) return;
	if ((unsigned)id >= mPoses.size()) mPoses.resize(id+1);

	mPoses[id] = PoseInScene(pose, sceneId);
}

PoseDatabase::PoseInScene PoseDatabase::retrieveWAPose(int k, int ids[], float distances[]) const
{
	ORUtils::Matrix4<float> m;
	m.setZeros();

	int sceneID = -1;
	float sumWeights = 0.0f;
	for (int i = 0; i < k; ++i) {
		const PoseInScene & pose = retrievePose(ids[i]);
		if (sceneID == -1) sceneID = pose.sceneIdx;
		else if (sceneID != pose.sceneIdx) continue;

		float weight = 1.0f - distances[i];
		m += pose.pose.GetM() * weight;
		sumWeights += weight;
	}

	m = m * (1.0f/sumWeights);
	return PoseDatabase::PoseInScene(ITMLib::ITMPose(m), sceneID);
}

