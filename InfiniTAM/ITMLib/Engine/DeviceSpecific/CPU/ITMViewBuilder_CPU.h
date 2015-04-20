// Copyright 2014 Isis Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../../ITMViewBuilder.h"

namespace ITMLib
{
	class ITMViewBuilder_CPU : public ITMViewBuilder
	{
	public:
		void ConvertDisparityToDepth(ITMFloatImage *depth_out, const ITMShortImage *disp_in, const ITMIntrinsics *depthIntrinsics, 
			const ITMDisparityCalib *disparityCalib);
		void ConvertDepthMMToFloat(ITMFloatImage *depth_out, const ITMShortImage *depth_in);

		void DepthFiltering(ITMFloatImage *image_out, const ITMFloatImage *image_in);
		void ComputeNormalAndWeights(ITMFloat4Image *normal_out, ITMFloatImage *sigmaZ_out, const ITMFloatImage *depth_in, Vector4f intrinsic);

		void UpdateView(ITMView **view, ITMUChar4Image *rgbImage, ITMShortImage *rawDepthImage, bool useBilateralFilter, bool modelSensorNoise = false);
		void UpdateView(ITMView **view, ITMUChar4Image *rgbImage, ITMFloatImage *depthImage);

		void UpdateView(ITMView **view, ITMUChar4Image *rgbImage, ITMShortImage *depthImage, bool useBilateralFilter, ITMIMUMeasurement *imuMeasurement, bool modelSensorNoise = false);

		ITMViewBuilder_CPU(const ITMRGBDCalib *calib);
		~ITMViewBuilder_CPU(void);
	};
}

