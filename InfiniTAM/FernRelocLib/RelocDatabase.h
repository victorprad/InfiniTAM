// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#include <vector>
#include <string>

namespace FernRelocLib
{
	class RelocDatabase
	{
	public:
		RelocDatabase(int codeLength, int codeFragmentDim);
		~RelocDatabase(void);

		/** @return Number of valid similar entries that were found. Mostly
			relevant in case of an empty database.
		*/
		int findMostSimilar(const char *codeFragments, int nearestNeighbours[], float distances[], int k);

		/** @return ID of newly added entry */
		int addEntry(const char *codeFragments);

		void SaveToFile(const std::string &framesFileName) const;
		void LoadFromFile(const std::string &filename);

	private:
		int mTotalEntries;

		int mCodeLength, mCodeFragmentDim;
		std::vector<int> *mIds;
	};
}
