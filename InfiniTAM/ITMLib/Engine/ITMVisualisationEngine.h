// Copyright 2014 Isis Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../Utils/ITMLibDefines.h"

#include "../Objects/ITMImage.h"

using namespace ITMLib::Objects;

namespace ITMLib
{
	namespace Engine
	{
		/** Interface to engines helping with the visualisation of
		    the results from the rest of the library.
		*/
		class ITMVisualisationEngine
		{
		private:
			// Private and undefined to prevent instantiation
			ITMVisualisationEngine();
		public:
			static void DepthToUchar4(ITMUChar4Image *dst, ITMFloatImage *src);
		};
	}
}
