// Copyright 2014 Isis Innovation Limited and the authors of InfiniTAM

#pragma once

#include <vector>

#include "../ITMLib/Camera/ITMPose.h"

namespace LCDLib {

class PoseDatabase {
	public:
	struct PoseInScene {
		PoseInScene(void) {}
		PoseInScene(const ITMLib::ITMPose & _pose, int _sceneIdx)
		  : pose(_pose), sceneIdx(_sceneIdx) {}
		ITMLib::ITMPose pose;
		int sceneIdx;
	};

	PoseDatabase(void);
	~PoseDatabase(void);

	void storePose(int id, const ITMLib::ITMPose & pose, int sceneId);
	int numPoses(void) const { return (int)mPoses.size(); }

	const PoseInScene & retrievePose(int id) const { return mPoses[id]; }
	PoseInScene retrieveWAPose(int k, int ids[], float weights[]) const;

	private:
	std::vector<PoseInScene> mPoses;
};

}
