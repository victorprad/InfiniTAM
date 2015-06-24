// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../../ITMLowLevelEngine.h"

namespace ITMLib
{
	namespace Engine
	{
		class ITMLowLevelEngine_CUDA : public ITMLowLevelEngine
		{
		public:
			void CopyImage(ITMUChar4Image *image_out, const ITMUChar4Image *image_in) const;
			void CopyImage(ITMFloatImage *image_out, const ITMFloatImage *image_in) const;
			void CopyImage(ITMFloat4Image *image_out, const ITMFloat4Image *image_in) const;

			void FilterSubsample(ITMUChar4Image *image_out, const ITMUChar4Image *image_in) const;
			void FilterSubsampleWithHoles(ITMFloatImage *image_out, const ITMFloatImage *image_in) const;
			void FilterSubsampleWithHoles(ITMFloat4Image *image_out, const ITMFloat4Image *image_in) const;

			void GradientX(ITMShort4Image *grad_out, const ITMUChar4Image *image_in) const;
			void GradientY(ITMShort4Image *grad_out, const ITMUChar4Image *image_in) const;

			ITMLowLevelEngine_CUDA(void);
			~ITMLowLevelEngine_CUDA(void);
		};
	}
}
