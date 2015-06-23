// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../Objects/ITMRGBDCalib.h"
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
			/// Intrinsic calibration information for the view.
			ITMRGBDCalib *calib;

			/// RGB colour image.
			ITMUChar4Image *rgb; 

			/// Float valued depth image, if available according to @ref inputImageType.
			ITMFloatImage *depth;

			/// surface normal of depth image
			// allocated when needed
			ITMFloat4Image *depthNormal;

			/// uncertainty (std) in each pixel of depth value based on sensor noise model
			/// allocated when needed
			ITMFloatImage *depthUncertainty;

			ITMView(const ITMRGBDCalib *calibration, Vector2i imgSize_rgb, Vector2i imgSize_d, bool useGPU)
			{
				this->calib = new ITMRGBDCalib(*calibration);
				this->rgb = new ITMUChar4Image(imgSize_rgb, true, useGPU);
				this->depth = new ITMFloatImage(imgSize_d, true, useGPU);
				this->depthNormal = NULL;
				this->depthUncertainty = NULL;
			}

			virtual ~ITMView(void)
			{
				delete calib;

				delete rgb;
				delete depth;

				if (depthNormal != NULL) delete depthNormal;
				if (depthUncertainty != NULL) delete depthUncertainty;
			}

			// Suppress the default copy constructor and assignment operator
			ITMView(const ITMView&);
			ITMView& operator=(const ITMView&);
		};
	}
}

