// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../ORUtils/PlatformIndependence.h"
#include "../ORUtils/MathUtils.h"
#include "../ORUtils/Vector.h"
#include "../ORUtils/Image.h"

namespace FernRelocLib
{
	struct FernTester
	{
		ORUtils::Vector2<int> location;
		float threshold;
	};

	class FernConservatory
	{
	public:
		FernConservatory(int numFerns, ORUtils::Vector2<int> imgSize, ORUtils::Vector2<float> bounds, int decisionsPerFern = 1);
		~FernConservatory(void);

		// takes a (small) image, applies the binary tests in the ferns, creates
		// the code fragments as an array
		void computeCode(const ORUtils::Image<float> *img, char *codeFragments) const;
		void computeCode(const ORUtils::Image< ORUtils::Vector4<unsigned char> > *img, char *codeFragments) const;

		void SaveToFile(const std::string &fernsFileName);
		void LoadFromFile(const std::string &fernsFileName);

		int getNumFerns(void) const { return mNumFerns; }
		int getNumCodes(void) const { return 1 << mNumDecisions; }
		int getNumDecisions(void) const { return mNumDecisions; }

	private:
		int mNumFerns;
		int mNumDecisions;
		FernTester *mEncoders;
	};
}
