// Copyright 2014 Isis Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../Utils/ITMLibDefines.h"
#include "../Utils/ITMLibSettings.h"

#include "../Objects/ITMView.h"
#include "../Objects/ITMViewIMU.h"
#include "../Objects/ITMRGBDCalib.h"

using namespace ITMLib::Objects;

namespace ITMLib
{
	namespace Engine
	{
		/** \brief
		*/
		class ITMViewBuilder
		{
		protected:
			const ITMRGBDCalib *calib;
			ITMShortImage *shortImage;
			ITMFloatImage *floatImage;

		public:
			enum InputImageType
			{
				//! Raw disparity images as received from the
				//! Kinect
				InfiniTAM_DISPARITY_IMAGE,
				//! Short valued depth image in millimetres
				InfiniTAM_SHORT_DEPTH_IMAGE,
				//! Floating point valued depth images in meters
				InfiniTAM_FLOAT_DEPTH_IMAGE
			}inputImageType;

			virtual void ConvertDisparityToDepth(ITMFloatImage *depth_out, const ITMShortImage *disp_in, const ITMIntrinsics *depthIntrinsics,
				const ITMDisparityCalib *disparityCalib) = 0;
			virtual void ConvertDepthMMToFloat(ITMFloatImage *depth_out, const ITMShortImage *depth_in) = 0;

			virtual void DepthFiltering(ITMFloatImage *image_out, const ITMFloatImage *image_in) = 0;

			virtual void UpdateView(ITMView **view, ITMUChar4Image *rgbImage, ITMShortImage *rawDepthImage, bool useBilateralFilter) = 0;
			virtual void UpdateView(ITMView **view, ITMUChar4Image *rgbImage, ITMFloatImage *depthImage) = 0;

			virtual void UpdateView(ITMView **view, ITMUChar4Image *rgbImage, ITMShortImage *depthImage, bool useBilateralFilter,
				ITMIMUMeasurement *imuMeasurement) = 0;

			ITMViewBuilder(const ITMRGBDCalib *calib)
			{
				this->calib = calib;
				this->shortImage = NULL;
				this->floatImage = NULL;

				if (calib->disparityCalib.params == Vector2f(0.0f, 0.0f)) inputImageType = InfiniTAM_SHORT_DEPTH_IMAGE;
				else inputImageType = InfiniTAM_DISPARITY_IMAGE;
			}

			virtual ~ITMViewBuilder()
			{
				if (this->shortImage != NULL) delete this->shortImage;
				if (this->floatImage != NULL) delete this->floatImage;
			}
		};
	}
}

