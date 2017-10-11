// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#include "MathUtils.h"
#include <math.h>
#include <string.h>

/*This class is based on the homkermap.c functions from VLFeat www.vlfeat.org */

namespace ORUtils
{
	class SVMClassifier
	{
	private:
		float *w, b; int noDimensions;

	public:
		SVMClassifier(int noDimensions)
		{
			this->noDimensions = noDimensions;
			w = new float[noDimensions];
		}

		void SetVectors(float *w, float b)
		{
			for (int i = 0; i < noDimensions; i++) 
				this->w[i] = w[i];
			this->b = b;
		}

		float Classify(float *input)
		{
			float total = 0.0f;

			for (int dimId = 0; dimId < noDimensions; dimId++)
				total += w[dimId] * input[dimId];

			total += b;

			return total;
		}

		~SVMClassifier()
		{
			delete w;
		}
	};
}