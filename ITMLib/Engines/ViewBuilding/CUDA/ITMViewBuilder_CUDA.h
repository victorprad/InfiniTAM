// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../Interface/ITMViewBuilder.h"

namespace ITMLib
{
	class ITMViewBuilder_CUDA : public ITMViewBuilder
	{
	public:
		void ConvertDisparityToDepth(ORFloatImage *depth_out, const ORShortImage *depth_in, const ITMIntrinsics *depthIntrinsics, 
			Vector2f disparityCalibParams);
		void ConvertDepthAffineToFloat(ORFloatImage *depth_out, const ORShortImage *depth_in, Vector2f depthCalibParams);

		void DepthFiltering(ORFloatImage *image_out, const ORFloatImage *image_in);
		void ComputeNormalAndWeights(ORFloat4Image *normal_out, ORFloatImage *sigmaZ_out, const ORFloatImage *depth_in, Vector4f intrinsic);

		void UpdateView(ITMView **view, ORUChar4Image *rgbImage, ORShortImage *rawDepthImage, bool useBilateralFilter, bool modelSensorNoise = false, bool storePreviousImage = true);
		void UpdateView(ITMView **view, ORUChar4Image *rgbImage, ORShortImage *depthImage, bool useBilateralFilter, ITMIMUMeasurement *imuMeasurement, bool modelSensorNoise = false, bool storePreviousImage = true);

		ITMViewBuilder_CUDA(const ITMRGBDCalib& calib);
		~ITMViewBuilder_CUDA(void);
	};
}
