// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../Utils/ITMLibDefines.h"

using namespace ITMLib::Objects;

namespace ITMLib
{
	namespace Engine
	{
		/// Interface to low level image processing engines.
		class ITMLowLevelEngine
		{
		public:
			virtual void CopyImage(ITMUChar4Image *image_out, const ITMUChar4Image *image_in) const = 0;
			virtual void CopyImage(ITMFloatImage *image_out, const ITMFloatImage *image_in) const = 0;
			virtual void CopyImage(ITMFloat4Image *image_out, const ITMFloat4Image *image_in) const = 0;

			virtual void FilterSubsample(ITMUChar4Image *image_out, const ITMUChar4Image *image_in) const = 0;
			virtual void FilterSubsampleWithHoles(ITMFloatImage *image_out, const ITMFloatImage *image_in) const = 0;
			virtual void FilterSubsampleWithHoles(ITMFloat4Image *image_out, const ITMFloat4Image *image_in) const = 0;

			virtual void GradientX(ITMShort4Image *grad_out, const ITMUChar4Image *image_in) const = 0;
			virtual void GradientY(ITMShort4Image *grad_out, const ITMUChar4Image *image_in) const = 0;

			ITMLowLevelEngine(void) { }
			virtual ~ITMLowLevelEngine(void) { }
		};
	}
}
