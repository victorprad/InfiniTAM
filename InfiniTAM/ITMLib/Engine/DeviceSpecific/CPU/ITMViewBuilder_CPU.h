// Copyright 2014 Isis Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../../ITMViewBuilder.h"

namespace ITMLib
{
	namespace Engine
	{
		class ITMViewBuilder_CPU : public ITMViewBuilder
		{
		protected:
			void AllocateView(ITMView *view, Vector2i imgSize_rgb, Vector2i imgSize_d);

		public:
			void ConvertDisparityToDepth(ITMFloatImage *depth_out, const ITMShortImage *disp_in, const ITMIntrinsics *depthIntrinsics, 
				const ITMDisparityCalib *disparityCalib);
			void ConvertDepthMMToFloat(ITMFloatImage *depth_out, const ITMShortImage *depth_in);

			void UpdateView(ITMView *view, ITMUChar4Image *rgbImage, ITMShortImage *rawDepthImage);
			void UpdateView(ITMView *view, ITMUChar4Image *rgbImage, ITMFloatImage *depthImage);

			void UpdateView(ITMView *view, ITMUChar4Image *rgbImage, ITMShortImage *depthImage, ITMIMUMeasurement *imuMeasurement);

			ITMViewBuilder_CPU(const ITMRGBDCalib *calib);
			~ITMViewBuilder_CPU(void);
		};
	}
}
