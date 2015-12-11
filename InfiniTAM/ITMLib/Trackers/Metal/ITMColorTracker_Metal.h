// Copyright 2014 Isis Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../../ITMColorTracker.h"

namespace ITMLib
{
    namespace Engine
    {
        class ITMColorTracker_Metal : public ITMColorTracker
        {
        public:
            void F_oneLevel(float *f, ITMPose *pose);
            void G_oneLevel(float *gradient, float *hessian, ITMPose *pose) const;
            
            ITMColorTracker_Metal(Vector2i imgSize, int noHierarchyLevels, int noRotationOnlyLevels,
                                ITMLowLevelEngine *lowLevelEngine);
            ~ITMColorTracker_Metal(void);
        };
    }
}
