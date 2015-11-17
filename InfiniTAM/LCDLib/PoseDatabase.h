// Copyright 2014 Isis Innovation Limited and the authors of InfiniTAM

#pragma once

#include <vector>

#include "../ITMLib/Objects/ITMPose.h"

namespace LCDLib {

class PoseDatabase {
	public:
	PoseDatabase(void);
	~PoseDatabase(void);

	void storePose(int id, const ITMLib::ITMPose & pose);
	int numPoses(void) const { return (int)mPoses.size(); }

	const ITMLib::ITMPose & retrievePose(int id) const;
	ITMLib::ITMPose retrieveWAPose(int k, int ids[], float weights[]) const;

	private:
	std::vector<ITMLib::ITMPose> mPoses;
};

}

