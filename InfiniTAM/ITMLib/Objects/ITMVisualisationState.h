// Copyright 2014 Isis Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../Utils/ITMLibDefines.h"

namespace ITMLib
{
	namespace Objects
	{
		class ITMVisualisationState {
			public:
			ITMFloat2Image *minmaxImage;
			ITMUChar4Image *outputImage;

			ITMVisualisationState(const Vector2i & imgSize, bool allocateGPU)
			{
				minmaxImage = new ITMFloat2Image(imgSize, allocateGPU);
				outputImage = new ITMUChar4Image(imgSize, allocateGPU);
			}

			virtual ~ITMVisualisationState(void)
			{
				delete minmaxImage;
				delete outputImage;
			}
		};
	}
}
