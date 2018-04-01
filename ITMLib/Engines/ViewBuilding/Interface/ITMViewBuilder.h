// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../../../Objects/Camera/ITMRGBDCalib.h"
#include "../../../Objects/Views/ITMViewIMU.h"

namespace ITMLib
{
	/** \brief
	*/
	class ITMViewBuilder
	{
	protected:
		const ITMRGBDCalib calib;
		ORShortImage *shortImage;
		ORFloatImage *floatImage;

	public:
		virtual void ConvertDisparityToDepth(ORFloatImage *depth_out, const ORShortImage *disp_in, const ITMIntrinsics *depthIntrinsics,
			Vector2f disparityCalibParams) = 0;
		virtual void ConvertDepthAffineToFloat(ORFloatImage *depth_out, const ORShortImage *depth_in, Vector2f depthCalibParams) = 0;

		virtual void DepthFiltering(ORFloatImage *image_out, const ORFloatImage *image_in) = 0;
		virtual void ComputeNormalAndWeights(ORFloat4Image *normal_out, ORFloatImage *sigmaZ_out, const ORFloatImage *depth_in, Vector4f intrinsic) = 0;

		virtual void UpdateView(ITMView **view, ORUChar4Image *rgbImage, ORShortImage *rawDepthImage, bool useBilateralFilter, bool modelSensorNoise = false, bool storePreviousImage = true) = 0;
		virtual void UpdateView(ITMView **view, ORUChar4Image *rgbImage, ORShortImage *depthImage, bool useBilateralFilter, ITMIMUMeasurement *imuMeasurement, bool modelSensorNoise = false, bool storePreviousImage = true) = 0;

		ITMViewBuilder(const ITMRGBDCalib& calib_)
		: calib(calib_)
		{
			this->shortImage = NULL;
			this->floatImage = NULL;
		}

		virtual ~ITMViewBuilder()
		{
			if (this->shortImage != NULL) delete this->shortImage;
			if (this->floatImage != NULL) delete this->floatImage;
		}
	};
}
