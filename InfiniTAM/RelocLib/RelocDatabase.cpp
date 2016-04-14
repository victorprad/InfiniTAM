// Copyright 2014 Isis Innovation Limited and the authors of InfiniTAM

#include "RelocDatabase.h"

using namespace RelocLib;

RelocDatabase::RelocDatabase(int codeLength, int codeFragmentDim)
{
	mTotalEntries = 0;
	mCodeLength = codeLength;
	mCodeFragmentDim = codeFragmentDim;

	mIds = new std::vector<int>[codeLength*codeFragmentDim];
}

RelocDatabase::~RelocDatabase(void)
{
	delete[] mIds;
}

int RelocDatabase::findMostSimilar(const char *codeFragments, int nearestNeighbours[], float distances[], int k)
{
	int foundNN = 0;
	if (mTotalEntries > 0) {
		int *similarities = new int[mTotalEntries];
		for (int i = 0; i < mTotalEntries; ++i) similarities[i] = 0;

		for (int f = 0; f < mCodeLength; f++) {
			if (codeFragments[f] < 0) continue;
			const std::vector<int> *sameCode = &(mIds[f * mCodeFragmentDim + codeFragments[f]]);

			for (unsigned int i = 0; i < sameCode->size(); ++i) similarities[(*sameCode)[i]]++;
		}
//for (int i = 0; i < mTotalEntries; ++i) fprintf(stderr, "%i\n", similarities[i]);

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

		delete[] similarities;
	}

	for (int i = foundNN; i < k; ++i) {
		distances[i] = 1.0f;
		nearestNeighbours[i] = -1;
	}

	return foundNN;
}

// returns ID of newly added entry
int RelocDatabase::addEntry(const char *codeFragments)
{
	int newId = mTotalEntries++;
	for (int f = 0; f < mCodeLength; f++) {
		if (codeFragments[f] < 0) continue;
		std::vector<int> *sameCode = &(mIds[f * mCodeFragmentDim + codeFragments[f]]);

		sameCode->push_back(newId);
	}

	return newId;
}

