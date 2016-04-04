// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM

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
		const ITMRGBDCalib *calib;
		Vector2i paddingSize;
		ITMShortImage *shortImage;
		ITMFloatImage *floatImage;

		ITMUChar4Image *rgbImage;
		ITMShortImage *rawDepthImage;
		ITMFloatImage *depthImage;

	public:
		virtual void ConvertDisparityToDepth(ITMFloatImage *depth_out, const ITMShortImage *disp_in, const ITMIntrinsics *depthIntrinsics,
			Vector2f disparityCalibParams) = 0;
		virtual void ConvertDepthAffineToFloat(ITMFloatImage *depth_out, const ITMShortImage *depth_in, Vector2f depthCalibParams) = 0;

		virtual void DepthFiltering(ITMFloatImage *image_out, const ITMFloatImage *image_in) = 0;
		virtual void ComputeNormalAndWeights(ITMFloat4Image *normal_out, ITMFloatImage *sigmaZ_out, const ITMFloatImage *depth_in, Vector4f intrinsic) = 0;

		virtual void UpdateView(ITMView **view, ITMUChar4Image *rgbImage, ITMShortImage *rawDepthImage, bool useBilateralFilter, bool modelSensorNoise = false) = 0;
		//virtual void UpdateView(ITMView **view, ITMUChar4Image *rgbImage, ITMFloatImage *depthImage) = 0;

		virtual void UpdateView(ITMView **view, ITMUChar4Image *rgbImage, ITMShortImage *depthImage, bool useBilateralFilter, ITMIMUMeasurement *imuMeasurement, bool modelSensorNoise = false) = 0;

		ITMViewBuilder(const ITMRGBDCalib *calib, const Vector2i &paddingSize)
		{
			this->calib = calib;
			this->shortImage = NULL;
			this->floatImage = NULL;
			this->rgbImage = NULL;
			this->rawDepthImage = NULL;
			this->depthImage = NULL;
			this->paddingSize = paddingSize;
		}

		void PadImage(ITMShortImage *paddedImage, ITMShortImage *sourceImage, Vector2i paddingSize)
		{
			Vector2i paddedSize = sourceImage->noDims + 2 * paddingSize;

			short *destination = paddedImage->GetData(MEMORYDEVICE_CPU);

			memset(destination, 0, paddedSize.x * paddedSize.y * sizeof(short));

			short *img = sourceImage->GetData(MEMORYDEVICE_CPU);

			for (int y = 0; y < sourceImage->noDims.y; ++y) for (int x = 0; x < sourceImage->noDims.x; ++x) {
				destination[(x + paddingSize.x) + (y + paddingSize.y) * paddedSize.x] = img[x + y * sourceImage->noDims.x];
			}
		}

		void PadImage(ITMUChar4Image *paddedImage, ITMUChar4Image *sourceImage, Vector2i paddingSize)
		{

		}

		virtual ~ITMViewBuilder()
		{
			if (this->shortImage != NULL) delete this->shortImage;
			if (this->floatImage != NULL) delete this->floatImage;

			if (this->rgbImage != NULL) delete this->rgbImage;
			if (this->rawDepthImage != NULL) delete this->rawDepthImage;
			if (this->depthImage != NULL) delete this->depthImage;
		}
	};
}
