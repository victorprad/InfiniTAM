// Copyright 2014 Isis Innovation Limited and the authors of InfiniTAM

#pragma once

#include "FernConservatory.h"
#include "RelocDatabase.h"

namespace RelocLib {

class Relocaliser {
	public:
	Relocaliser(ORUtils::Vector2<int> imgSize, ORUtils::Vector2<float> range, float harvestingThreshold, int numFerns, int numDecisionsPerFern);
	~Relocaliser(void);

	/** Process an image for loop closure detection. This will find nearby
	    keyframes in the database and possibly add the image as a new
	    keyframe.
	    @param img_d Depth image to be processed. This should be given at
		full resolution and without blur, as any such operations will
		be performed internally.
	    @param k The function will return at most @p k nearest neighbours
	    @param nearestNeighbours Storage space where the nearest neighbours
		will be returned. Has to be at least of size @p k.
	    @param distances Distances to nearest neighbours. Usually
		normalised to the range [0...1]. Can be set to NULL, if the
		distances are not required.
	    @param harvestKeyframes If set to true, this frame will be
		considered for starting a new keyframe
	    @return New keyframe id, if the image is selected as a new
		keyframe, otherwise a negative value
	*/
	int ProcessFrame(const ORUtils::Image<float> *img_d, int k, int nearestNeighbours[], float *distances = NULL, bool harvestKeyframes = true) const;

	private:
	float mKeyframeHarvestingThreshold;
	FernConservatory *mEncoding;
	RelocDatabase *mDatabase;
	ORUtils::Image<float> *processedImage1, *processedImage2;
};

}

