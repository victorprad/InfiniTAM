// Copyright 2014 Isis Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../Objects/ITMRGBDCalib.h"
#include "../Objects/ITMImage.h"
#include "../Utils/ITMCalibIO.h"

namespace ITMLib
{
	namespace Objects
	{
		/** \brief
		    Represents a single "view", i.e. RGB and depth images along
		    with all intrinsic and relative calibration information
		*/
		class ITMView
		{
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
			};

			/// Intrinsic calibration information for the view.
			ITMRGBDCalib *calib;

			/// Identifies which sort of depth images are given.
			InputImageType inputImageType;

			/// RGB colour image.
			ITMUChar4Image *rgb; 
			/// Float valued depth image, if available according to @ref inputImageType.
			ITMFloatImage *depth;
			/// Raw disparity image, if available according to @ref inputImageType.
			ITMShortImage *rawDepth; 

			ITMView(const ITMRGBDCalib & calib, Vector2i imgSize_rgb, Vector2i imgSize_d, bool useGPU)
			{
				this->calib = new ITMRGBDCalib(calib);

				this->rgb = new ITMUChar4Image(imgSize_rgb, useGPU);
				this->depth = new ITMFloatImage(imgSize_d, useGPU);

				this->rawDepth = new ITMShortImage(imgSize_d, useGPU);
				this->inputImageType = InfiniTAM_DISPARITY_IMAGE;
			}

			~ITMView(void)
			{
				delete calib;

				delete rgb;
				delete depth;

				delete rawDepth;
			}

			// Suppress the default copy constructor and assignment operator
			ITMView(const ITMView&);
			ITMView& operator=(const ITMView&);
		};
	}
}

