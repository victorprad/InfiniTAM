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

int LoopClosureDetector::ProcessFrame(const ORUtils::Image<float> *img_d, int k, int nearestNeighbours[], float *distances, bool harvestKeyframes) const
{
	// TODO: downsample image

	int ret = -1;
	int codeLength = mEncoding->getNumFerns();
	char *code = new char[codeLength];

	mEncoding->computeCode(img_d, code);
for (int i = 0; i < codeLength; ++i) fprintf(stderr, "%i ", code[i]);
fprintf(stderr, "\n");

	bool releaseDistances = (distances == NULL);
	if (distances == NULL) distances = new float[k];

	int similarFound = mDatabase->findMostSimilar(code, nearestNeighbours, distances, k);

	if (harvestKeyframes) {
		if (similarFound == 0) ret = mDatabase->addEntry(code);
		else if (distances[0] > 0.2f) ret = mDatabase->addEntry(code);
	}

	for (int i = 0; i < similarFound; ++i) {
		fprintf(stderr, "found similar entry #%i: %i (%f)\n", i+1, nearestNeighbours[i], distances[i]);
	}

	delete[] code;
	if (releaseDistances) delete[] distances;

	return ret;
}

