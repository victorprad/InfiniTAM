#include "LCDDatabase.h"

#include <stdio.h>
using namespace LCDLib;

LCDDatabase::LCDDatabase(int codeLength, int codeFragmentDim)
{
	mTotalEntries = 0;
	mCodeLength = codeLength;
	mCodeFragmentDim = codeFragmentDim;

	mIds = new std::vector<int>[codeLength*codeFragmentDim];
}

LCDDatabase::~LCDDatabase(void)
{
	delete[] mIds;
}

int LCDDatabase::findMostSimilar(const char *codeFragments, int nearestNeighbours[], float distances[], int k)
{
	if (mTotalEntries == 0) return 0;

	int *similarities = new int[mTotalEntries];
	for (int i = 0; i < mTotalEntries; ++i) similarities[i] = 0;

fprintf(stderr, "a\n");
	for (int f = 0; f < mCodeLength; f++) {
		const std::vector<int> *sameCode = &(mIds[f * mCodeFragmentDim + codeFragments[f]]);

		for (unsigned int i = 0; i < sameCode->size(); ++i) similarities[(*sameCode)[i]]++;
	}
fprintf(stderr, "b\n");
for (int i = 0; i < mTotalEntries; ++i) fprintf(stderr, "%i\n", similarities[i]);

	int foundNN = 0;
	for (int i = 0; i < mTotalEntries; ++i) {
		float distance = ((float)mCodeLength - (float)similarities[i])/(float)mCodeLength;
		int j;
		for (j = foundNN; j > 0; --j) {
			if (distances[j-1] < distance) break;
			if (j == k) continue;
			distances[j] = distances[j-1];
			nearestNeighbours[j] = nearestNeighbours[j-1];
		}
		if (j != k) {
			distances[j] = distance;
			nearestNeighbours[j] = i;
			if (foundNN < k) ++foundNN;
		}
	}
fprintf(stderr, "c\n");

	delete[] similarities;

	return foundNN;
}

// returns ID of newly added entry
int LCDDatabase::addEntry(const char *codeFragments)
{
fprintf(stderr, "ADD NEW ENTRY\n");
	int newId = mTotalEntries++;
	for (int f = 0; f < mCodeLength; f++) {
		std::vector<int> *sameCode = &(mIds[f * mCodeFragmentDim + codeFragments[f]]);

		sameCode->push_back(newId);
	}

	return newId;
}

