// Copyright 2014 Isis Innovation Limited and the authors of InfiniTAM

#pragma once

#ifndef __METALC__

#include "../../ITMLowLevelEngine.h"

namespace ITMLib
{
	namespace Engine
	{
		class ITMLowLevelEngine_Metal : public ITMLowLevelEngine
		{
		public:
			void CopyImage(ITMUChar4Image *image_out, const ITMUChar4Image *image_in);
			void CopyImage(ITMFloatImage *image_out, const ITMFloatImage *image_in);
			void CopyImage(ITMFloat4Image *image_out, const ITMFloat4Image *image_in);

			void FilterSubsample(ITMUChar4Image *image_out, const ITMUChar4Image *image_in);
			void FilterSubsampleWithHoles(ITMFloatImage *image_out, const ITMFloatImage *image_in);
			void FilterSubsampleWithHoles(ITMFloat4Image *image_out, const ITMFloat4Image *image_in);

			void GradientX(ITMShort4Image *grad_out, const ITMUChar4Image *image_in);
			void GradientY(ITMShort4Image *grad_out, const ITMUChar4Image *image_in);

			void ConvertDisparityToDepth(ITMFloatImage *depth_out, const ITMShortImage *disp_in, const ITMIntrinsics *depthIntrinsics, 
				const ITMDisparityCalib *disparityCalib);
			void ConvertDepthMMToFloat(ITMFloatImage *depth_out, const ITMShortImage *depth_in);

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