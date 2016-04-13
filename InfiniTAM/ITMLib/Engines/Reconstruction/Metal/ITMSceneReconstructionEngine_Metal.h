// Copyright 2014 Isis Innovation Limited and the authors of InfiniTAM
#pragma once

#ifndef __METALC__

#include "../CPU/ITMSceneReconstructionEngine_CPU.h"

namespace ITMLib
{
    template<class TVoxel, class TIndex>
    class ITMSceneReconstructionEngine_Metal : public ITMSceneReconstructionEngine_CPU<TVoxel,TIndex>
    {};
    
    template<class TVoxel>
    class ITMSceneReconstructionEngine_Metal<TVoxel,ITMVoxelBlockHash> : public ITMSceneReconstructionEngine_CPU<TVoxel,ITMVoxelBlockHash>
    {
    private:
        void BuildAllocAndVisibleType(ITMScene<TVoxel, ITMVoxelBlockHash> *scene, const ITMView *view,
                                      const ITMTrackingState *trackingState, const ITMRenderState *renderState);
    public:
        void IntegrateIntoScene(ITMScene<TVoxel, ITMVoxelBlockHash> *scene, const ITMView *view, const ITMTrackingState *trackingState,
                                const ITMRenderState *renderState);
        
        void AllocateSceneFromDepth(ITMScene<TVoxel, ITMVoxelBlockHash> *scene, const ITMView *view,
                                    const ITMTrackingState *trackingState, const ITMRenderState *renderState,
                                    bool onlyUpdateVisibleList = false, bool resetVisibleList = false);
        
        ITMSceneReconstructionEngine_Metal(void);
    };
    
    template<class TVoxel>
    class ITMSceneReconstructionEngine_Metal<TVoxel,ITMPlainVoxelArray> : public ITMSceneReconstructionEngine_CPU<TVoxel,ITMPlainVoxelArray>
    { };
}

#endif

#if (defined __OBJC__) || (defined __METALC__)

struct IntegrateIntoScene_VH_Params
{
    Vector2i rgbImgSize, depthImgSize;
    Matrix4f M_d, M_rgb;
    Vector4f projParams_d, projParams_rgb;
    Vector4f others;
};

struct BuildAllocVisibleType_VH_Params
{
    Matrix4f invM_d;
    Vector4f invProjParams_d;
    Vector4f others;
    Vector2i depthImgSize;
};

#endif