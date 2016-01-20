// Copyright 2014 Isis Innovation Limited and the authors of InfiniTAM

#pragma once

#include <vector>

#include "../ORUtils/SE3Pose.h"

namespace LCDLib {

class PoseDatabase {
	public:
	struct PoseInScene {
		PoseInScene(void) {}
		PoseInScene(const ORUtils::SE3Pose & _pose, int _sceneIdx)
		  : pose(_pose), sceneIdx(_sceneIdx) {}
		ORUtils::SE3Pose pose;
		int sceneIdx;
	};

	PoseDatabase(void);
	~PoseDatabase(void);

	void storePose(int id, const ORUtils::SE3Pose & pose, int sceneId);
	int numPoses(void) const { return (int)mPoses.size(); }

	const PoseInScene & retrievePose(int id) const { return mPoses[id]; }
	PoseInScene retrieveWAPose(int k, int ids[], float weights[]) const;

	private:
	std::vector<PoseInScene> mPoses;
};

}
