// Copyright 2014 Isis Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../Objects/ITMRGBDCalib.h"
#include "../Utils/ITMCalibIO.h"

namespace ITMLib
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

		ITMView(const ITMRGBDCalib *calibration, Vector2i imgSize_rgb, Vector2i imgSize_d, bool useGPU)
		{
			this->calib = new ITMRGBDCalib(*calibration);
			this->rgb = new ITMUChar4Image(imgSize_rgb, true, useGPU);
			this->depth = new ITMFloatImage(imgSize_d, true, useGPU);
		}

		virtual ~ITMView(void)
		{
			delete calib;

			delete rgb;
			delete depth;
		}

		// Suppress the default copy constructor and assignment operator
		ITMView(const ITMView&);
		ITMView& operator=(const ITMView&);
	};
}
