// Copyright 2014 Isis Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../Utils/ITMLibDefines.h"
#include "../Objects/ITMDisparityCalib.h"
#include "../Objects/ITMIntrinsics.h"
#include "../Objects/ITMExtrinsics.h"

#include "../Objects/ITMImage.h"

using namespace ITMLib::Objects;

namespace ITMLib
{
	namespace Engine
	{
		/// Interface to low level image processing engines.
		class ITMLowLevelEngine
		{
		public:
			virtual void CopyImage(ITMUChar4Image *image_out, const ITMUChar4Image *image_in) = 0;
			virtual void CopyImage(ITMFloatImage *image_out, const ITMFloatImage *image_in) = 0;
			virtual void CopyImage(ITMFloat4Image *image_out, const ITMFloat4Image *image_in) = 0;

			virtual void FilterSubsample(ITMUChar4Image *image_out, const ITMUChar4Image *image_in) = 0;
			virtual void FilterSubsampleWithHoles(ITMFloatImage *image_out, const ITMFloatImage *image_in) = 0;
			virtual void FilterSubsampleWithHoles(ITMFloat4Image *image_out, const ITMFloat4Image *image_in) = 0;

			virtual void GradientX(ITMShort4Image *grad_out, const ITMUChar4Image *image_in) = 0;
			virtual void GradientY(ITMShort4Image *grad_out, const ITMUChar4Image *image_in) = 0;

			virtual void ConvertDisparityToDepth(ITMFloatImage *depth_out, const ITMShortImage *disp_in, const ITMIntrinsics *depthIntrinsics,
				const ITMDisparityCalib *disparityCalib) = 0;
			virtual void ConvertDepthMMToFloat(ITMFloatImage *depth_out, const ITMShortImage *depth_in) = 0;

			ITMLowLevelEngine(void) { }
			virtual ~ITMLowLevelEngine(void) { }
		};
	}
}
