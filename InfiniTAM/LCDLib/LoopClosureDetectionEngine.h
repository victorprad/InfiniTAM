#pragma once

#include "FernConservatory.h"
#include "LCDDatabase.h"

namespace LCDLib {

class LoopClosureDetector {
	public:
	LoopClosureDetector(ORUtils::Vector2<int> imgSize, ORUtils::Vector2<float> range, int numFerns);
	~LoopClosureDetector(void);

//	static bool ProcessFrame(const ORUtils::Image<short> *img_d, const FernConservatory *encoding, LCDDatabase *database, int nearestNeighbours[], int k);

	// takes image at full resolution
	// return new id, if the image is a new keyframe, negative value otherwise
	int ProcessFrame(const ORUtils::Image<float> *img_d, int nearestNeighbours[], int k) const;

	FernConservatory *mEncoding;
	LCDDatabase *mDatabase;
};

}

