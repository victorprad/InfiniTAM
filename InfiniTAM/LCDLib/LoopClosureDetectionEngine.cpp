#include "LoopClosureDetectionEngine.h"

using namespace LCDLib;

LoopClosureDetector::LoopClosureDetector(ORUtils::Vector2<int> imgSize, ORUtils::Vector2<float> range, int numFerns)
{
	mEncoding = new FernConservatory(numFerns, imgSize, range);
	mDatabase = new LCDDatabase(numFerns, 2);
}

LoopClosureDetector::~LoopClosureDetector(void)
{
	delete mEncoding;
	delete mDatabase;
}

int LoopClosureDetector::ProcessFrame(const ORUtils::Image<float> *img_d, int nearestNeighbours[], int k) const
{
	// TODO: downsample image

	int ret = -1;
	int codeLength = mEncoding->getNumFerns();
	char *code = new char[codeLength];

	mEncoding->computeCode(img_d, code);
for (int i = 0; i < codeLength; ++i) fprintf(stderr, "%i ", code[i]);
fprintf(stderr, "\n");

	float *distances = new float[k];
	int similarFound = mDatabase->findMostSimilar(code, nearestNeighbours, distances, k);
fprintf(stderr, "fouind similar\n");

	if (similarFound == 0) ret = mDatabase->addEntry(code);
	else if (distances[0] > 0.2f) ret = mDatabase->addEntry(code);
fprintf(stderr, "added\n");

	for (int i = 0; i < similarFound; ++i) {
		fprintf(stderr, "found similar entry #%i: %i (%f)\n", i+1, nearestNeighbours[i], distances[i]);
	}

	delete[] code;
	delete[] distances;

	return ret;
}

