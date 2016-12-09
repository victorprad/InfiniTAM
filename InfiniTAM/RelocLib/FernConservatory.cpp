// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#include "FernConservatory.h"

#include <fstream>

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
			const FernTester *tester = &(mEncoders[f*mNumDecisions + d]);
			int locId = tester->location.x + tester->location.y * img->noDims.x;
			float val = imgData[locId];

			/*if (val <= 0.0f) codeFragments[f] = -1;
			else*/ codeFragments[f] |= ((val < tester->threshold) ? 0 : 1) << d;
		}
	}
}

void FernConservatory::SaveToFile(const std::string &fernsFileName)
{
	std::ofstream ofs(fernsFileName.c_str());

	if (!ofs) throw std::runtime_error("Could not open " + fernsFileName + " for reading");;

	for (int f = 0; f < mNumFerns * mNumDecisions; ++f) 
		ofs << mEncoders[f].location.x << ' ' << mEncoders[f].location.y << ' ' << mEncoders[f].threshold << '\n';
}

void FernConservatory::LoadFromFile(const std::string &fernsFileName)
{
	std::ifstream ifs(fernsFileName.c_str());
	if (!ifs) throw std::runtime_error("unable to load " + fernsFileName);

	for (int i = 0; i < mNumFerns; i++) 
	{
		for (int j = 0; j < mNumDecisions; j++) 
		{
			FernTester &fernTester = mEncoders[i * mNumDecisions + j];
			ifs >> fernTester.location.x >> fernTester.location.y >> fernTester.threshold;
		}
	}
}
