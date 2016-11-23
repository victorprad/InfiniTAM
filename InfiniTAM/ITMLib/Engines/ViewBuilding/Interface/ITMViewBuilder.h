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
		ITMShortImage *shortImage;
		ITMFloatImage *floatImage;

	public:
		virtual void ConvertDisparityToDepth(ITMFloatImage *depth_out, const ITMShortImage *disp_in, const ITMIntrinsics *depthIntrinsics,
			Vector2f disparityCalibParams) = 0;
		virtual void ConvertDepthAffineToFloat(ITMFloatImage *depth_out, const ITMShortImage *depth_in, Vector2f depthCalibParams) = 0;

		virtual void DepthFiltering(ITMFloatImage *image_out, const ITMFloatImage *image_in) = 0;
		virtual void ComputeNormalAndWeights(ITMFloat4Image *normal_out, ITMFloatImage *sigmaZ_out, const ITMFloatImage *depth_in, Vector4f intrinsic) = 0;

		virtual void UpdateView(ITMView **view, ITMUChar4Image *rgbImage, ITMShortImage *rawDepthImage, bool useBilateralFilter, bool modelSensorNoise = false, bool storePreviousImage = true) = 0;
		virtual void UpdateView(ITMView **view, ITMUChar4Image *rgbImage, ITMShortImage *depthImage, bool useBilateralFilter, ITMIMUMeasurement *imuMeasurement, bool modelSensorNoise = false, bool storePreviousImage = true) = 0;

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
