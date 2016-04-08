// Copyright 2014 Isis Innovation Limited and the authors of InfiniTAM

#pragma once

#ifndef __METALC__

#include "../CPU/ITMViewBuilder_CPU.h"

namespace ITMLib
{
	namespace Engine
	{
		class ITMViewBuilder_Metal : public ITMViewBuilder_CPU
		{
		public:
			void ConvertDisparityToDepth(ITMFloatImage *depth_out, const ITMShortImage *disp_in, const ITMIntrinsics *depthIntrinsics,
				const ITMDisparityCalib *disparityCalib);

			ITMViewBuilder_Metal(const ITMRGBDCalib *calib);
			~ITMViewBuilder_Metal(void);
		};
	}
}

#endif

#if (defined __OBJC__) || (defined __METALC__)

struct ConvertDisparityToDepth_Params
{
    Vector2f disparityCalibParams;
    Vector2i imgSize;
    float fx_depth;
};

#endif