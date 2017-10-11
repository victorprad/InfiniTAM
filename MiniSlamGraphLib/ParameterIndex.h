// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#include <map>

namespace MiniSlamGraph 
{
	class ParameterIndex {
	private:
		typedef std::map<int, int> Index;

		Index mIdx;
		int numPara;

	public:
		ParameterIndex(void)
		{
			numPara = 0;
		}

		void addIndex(int id, int num)
		{
			mIdx[id] = numPara;
			numPara += num;
		}

		int findIndex(int id) const
		{
			Index::const_iterator it = mIdx.find(id);
			if (it == mIdx.end()) return -1;
			return it->second;
		}

		int numTotalParameters(void) const
		{
			return numPara;
		}
	};
}