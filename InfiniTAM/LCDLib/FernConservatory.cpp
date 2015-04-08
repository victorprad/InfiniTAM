#include "FernConservatory.h"

using namespace LCDLib;

static float random_uniform01(void)
{
	return (float)random() / (float)RAND_MAX;
}

FernConservatory::FernConservatory(int numFerns, ORUtils::Vector2<int> imgSize, ORUtils::Vector2<float> bounds)
{
	mNumFerns = numFerns;
	mEncoders = new FernTester[mNumFerns];
	for (int f = 0; f < mNumFerns; ++f) {
		mEncoders[f].location.x = floor(random_uniform01() * imgSize.x);
		mEncoders[f].location.y = floor(random_uniform01() * imgSize.y);
		mEncoders[f].threshold = random_uniform01() * (bounds.y-bounds.x) + bounds.x;
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
		const FernTester *tester = &(mEncoders[f]);
		int locId = tester->location.x + tester->location.y * img->noDims.x;
		float val = imgData[locId];

		codeFragments[f] = (val < tester->threshold)?0:1;
	}
}

