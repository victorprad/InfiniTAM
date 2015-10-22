// Copyright 2014 Isis Innovation Limited and the authors of InfiniTAM

#pragma once

#include <vector>

namespace LCDLib {

class LCDDatabase {
	public:
	LCDDatabase(int codeLength, int codeFragmentDim);
	~LCDDatabase(void);

	/** @return Number of valid similar entries that were found. Mostly
	    relevant in case of an empty database.
	*/
	int findMostSimilar(const char *codeFragments, int nearestNeighbours[], float distances[], int k);

	/** @return ID of newly added entry */
	int addEntry(const char *codeFragments);

	private:
	int mTotalEntries;

	int mCodeLength, mCodeFragmentDim;
	std::vector<int> *mIds;
};

}

