// Copyright 2014 Isis Innovation Limited and the authors of InfiniTAM

#pragma once

#ifndef __METALC__

//#include "../../ITMLowLevelEngine.h"

#include "../CPU/ITMLowLevelEngine_CPU.h"

namespace ITMLib
{
	namespace Engine
	{
		class ITMLowLevelEngine_Metal : public ITMLowLevelEngine_CPU
		{
		public:
			void ConvertDisparityToDepth(ITMFloatImage *depth_out, const ITMShortImage *disp_in, const ITMIntrinsics *depthIntrinsics,
				const ITMDisparityCalib *disparityCalib);

			ITMLowLevelEngine_Metal(void);
			~ITMLowLevelEngine_Metal(void);
		};
	}
}

#endif

struct ConvertDisparityToDepth_Params
{
    Vector2f disparityCalibParams;
    Vector2i imgSize;
    float fx_depth;
};