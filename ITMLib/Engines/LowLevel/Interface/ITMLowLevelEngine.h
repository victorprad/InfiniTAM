// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../../../../ORUtils/ImageTypes.h"

namespace ITMLib
{
	/// Interface to low level image processing engines.
	class ITMLowLevelEngine
	{
	public:
		virtual void CopyImage(ORUChar4Image *image_out, const ORUChar4Image *image_in) const = 0;
		virtual void CopyImage(ORFloatImage *image_out, const ORFloatImage *image_in) const = 0;
		virtual void CopyImage(ORFloat4Image *image_out, const ORFloat4Image *image_in) const = 0;

		virtual void ConvertColourToIntensity(ORFloatImage *image_out, const ORUChar4Image *image_in) const = 0;

		virtual void FilterIntensity(ORFloatImage *image_out, const ORFloatImage *image_in) const = 0;

		virtual void FilterSubsample(ORUChar4Image *image_out, const ORUChar4Image *image_in) const = 0;
		virtual void FilterSubsample(ORFloatImage *image_out, const ORFloatImage *image_in) const = 0;
		virtual void FilterSubsampleWithHoles(ORFloatImage *image_out, const ORFloatImage *image_in) const = 0;
		virtual void FilterSubsampleWithHoles(ORFloat4Image *image_out, const ORFloat4Image *image_in) const = 0;

		virtual void GradientX(ORShort4Image *grad_out, const ORUChar4Image *image_in) const = 0;
		virtual void GradientY(ORShort4Image *grad_out, const ORUChar4Image *image_in) const = 0;
		virtual void GradientXY(ORFloat2Image *grad_out, const ORFloatImage *image_in) const = 0;

		virtual int CountValidDepths(const ORFloatImage *image_in) const = 0;

		ITMLowLevelEngine(void) { }
		virtual ~ITMLowLevelEngine(void) { }
	};
}
