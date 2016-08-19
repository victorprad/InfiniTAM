// Copyright 2014-2016 Isis Innovation Limited and the authors of InfiniTAM

#include "FernConservatory.h"

using namespace RelocLib;

static float random_uniform01(void)
{
	return (float)rand() / (float)RAND_MAX;
}

FernConservatory::FernConservatory(int numFerns, ORUtils::Vector2<int> imgSize, ORUtils::Vector2<float> bounds, int decisionsPerFern)
{
	mNumFerns = numFerns;
	mNumDecisions = decisionsPerFern;
	mEncoders = new FernTester[mNumFerns*decisionsPerFern];
	for (int f = 0; f < mNumFerns*decisionsPerFern; ++f) {
		mEncoders[f].location.x = (int)floor(random_uniform01() * imgSize.x);
		mEncoders[f].location.y = (int)floor(random_uniform01() * imgSize.y);
		mEncoders[f].threshold = random_uniform01() * (bounds.y - bounds.x) + bounds.x;
	}
}

FernConservatory::~FernConservatory(void)
{
	delete[] mEncoders;
}

void FernConservatory::computeCode(const ORUtils::Image<float> *img, char *codeFragments) const
{
	const float *imgData = img->GetData(MEMORYDEVICE_CPU);
	for (int f = 0; f < mNumFerns; ++f) {
		codeFragments[f] = 0;
		for (int d = 0; d < mNumDecisions; ++d) {
			const FernTester *tester = &(mEncoders[f*mNumDecisions+d]);
			int locId = tester->location.x + tester->location.y * img->noDims.x;
			float val = imgData[locId];

			/*if (val <= 0.0f) codeFragments[f] = -1;
			else*/ codeFragments[f] |= ((val < tester->threshold)?0:1)<<d;
		}
	}
}

