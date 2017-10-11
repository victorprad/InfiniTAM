// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

namespace MiniSlamGraph
{
	class VariableLengthVector
	{
	public:
		typedef std::vector<double> InternalStorage;

		void addData(int pos, int size, double *data)
		{
			int minSizeNeeded = pos + size;
			if (mData.size() < (unsigned)minSizeNeeded) mData.resize(minSizeNeeded, 0.0f);
			for (int i = 0; i < size; ++i) mData[i + pos] += data[i];
		}

		void setOverallSize(int size)
		{
			mData.resize(size, 0.0f);
		}

		int getOverallSize(void) const
		{
			return (int)mData.size();
		}

		const double* getData(void) const
		{
			return &(mData[0]);
		}

	private:
		InternalStorage mData;
	};
}

