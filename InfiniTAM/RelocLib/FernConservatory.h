// Copyright 2014 Isis Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../ORUtils/PlatformIndependence.h"
#include "../ORUtils/MathUtils.h"
#include "../ORUtils/Vector.h"
#include "../ORUtils/Image.h"

namespace RelocLib {

struct FernTester {
	public:
	ORUtils::Vector2<int> location;
	float threshold;
};

class FernConservatory {
	public:
	FernConservatory(int numFerns, ORUtils::Vector2<int> imgSize, ORUtils::Vector2<float> bounds, int decisionsPerFern = 1);
	~FernConservatory(void);

	// takes a (small) image, applies the binary tests in the ferns, creates
	// the code fragments as an array
	void computeCode(const ORUtils::Image<float> *img, char *codeFragments) const;

	int getNumFerns(void) const
	{ return mNumFerns; }

	int getNumCodes(void) const
	{ return (1 << mNumDecisions); }

	private:
	int mNumFerns;
	int mNumDecisions;
	FernTester *mEncoders;
};

}

