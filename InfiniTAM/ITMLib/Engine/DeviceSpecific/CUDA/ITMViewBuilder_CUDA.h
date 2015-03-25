// Copyright 2014 Isis Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../../ITMViewBuilder.h"

namespace ITMLib
{
	class ITMViewBuilder_CUDA : public ITMViewBuilder
	{
	public:
		void ConvertDisparityToDepth(ITMFloatImage *depth_out, const ITMShortImage *depth_in, const ITMIntrinsics *depthIntrinsics, 
			const ITMDisparityCalib *disparityCalib);
		void ConvertDepthMMToFloat(ITMFloatImage *depth_out, const ITMShortImage *depth_in);

		void DepthFiltering(ITMFloatImage *image_out, const ITMFloatImage *image_in);

		void UpdateView(ITMView **view, ITMUChar4Image *rgbImage, ITMShortImage *rawDepthImage, bool useBilateralFilter);
		void UpdateView(ITMView **view, ITMUChar4Image *rgbImage, ITMFloatImage *depthImage);

		void UpdateView(ITMView **view, ITMUChar4Image *rgbImage, ITMShortImage *depthImage, bool useBilateralFilter, ITMIMUMeasurement *imuMeasurement);

		ITMViewBuilder_CUDA(const ITMRGBDCalib *calib);
		~ITMViewBuilder_CUDA(void);
	};
}
