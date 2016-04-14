// Copyright 2014 Isis Innovation Limited and the authors of InfiniTAM

#pragma once

#ifndef __METALC__

#include "../CPU/ITMVisualisationEngine_CPU.h"

namespace ITMLib
{
    template<class TVoxel, class TIndex>
    class ITMVisualisationEngine_Metal : public ITMVisualisationEngine_CPU < TVoxel, TIndex >
    { };
    
    template<class TVoxel>
    class ITMVisualisationEngine_Metal<TVoxel, ITMVoxelBlockHash> : public ITMVisualisationEngine_CPU < TVoxel, ITMVoxelBlockHash >
    {
    public:
        void CreateICPMaps(const ITMScene<TVoxel,ITMVoxelBlockHash> *scene, const ITMView *view, ITMTrackingState *trackingState, ITMRenderState *renderState) const;
        
        ITMVisualisationEngine_Metal();
    };
}

#endif

#if (defined __OBJC__) || (defined __METALC__)

struct CreateICPMaps_Params
{
    Matrix4f invM;
    Matrix4f M;
    Vector4f projParams;
    Vector4f invProjParams;
    Vector4f lightSource;
    Vector4i imgSize;
    Vector2f voxelSizes;
};

#endif