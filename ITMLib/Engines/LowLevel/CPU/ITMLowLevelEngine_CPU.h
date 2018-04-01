// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../Interface/ITMLowLevelEngine.h"

namespace ITMLib
{
	class ITMLowLevelEngine_CPU : public ITMLowLevelEngine
	{
	public:
		void CopyImage(ORUChar4Image *image_out, const ORUChar4Image *image_in) const;
		void CopyImage(ORFloatImage *image_out, const ORFloatImage *image_in) const;
		void CopyImage(ORFloat4Image *image_out, const ORFloat4Image *image_in) const;

		void ConvertColourToIntensity(ORFloatImage *image_out, const ORUChar4Image *image_in) const;

		void FilterIntensity(ORFloatImage *image_out, const ORFloatImage *image_in) const;

		void FilterSubsample(ORUChar4Image *image_out, const ORUChar4Image *image_in) const;
		void FilterSubsample(ORFloatImage *image_out, const ORFloatImage *image_in) const;
		void FilterSubsampleWithHoles(ORFloatImage *image_out, const ORFloatImage *image_in) const;
		void FilterSubsampleWithHoles(ORFloat4Image *image_out, const ORFloat4Image *image_in) const;

		void GradientX(ORShort4Image *grad_out, const ORUChar4Image *image_in) const;
		void GradientY(ORShort4Image *grad_out, const ORUChar4Image *image_in) const;
		void GradientXY(ORFloat2Image *grad_out, const ORFloatImage *image_in) const;

		int CountValidDepths(const ORFloatImage *image_in) const;

		ITMLowLevelEngine_CPU(void);
		~ITMLowLevelEngine_CPU(void);
	};
}
