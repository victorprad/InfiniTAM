// Copyright 2014 Isis Innovation Limited and the authors of InfiniTAM

#pragma once

#include <vector>

namespace LCDLib {

class LCDDatabase {
	public:
	LCDDatabase(int codeLength, int codeFragmentDim);
	~LCDDatabase(void);

	// returns number of validly found similar entries (think of an empty
	// database here!)
	int findMostSimilar(const char *codeFragments, int nearestNeighbours[], float distances[], int k);

	// returns ID of newly added entry
	int addEntry(const char *codeFragments);

	private:
	int mTotalEntries;

	int mCodeLength, mCodeFragmentDim;
	std::vector<int> *mIds;
};

}

