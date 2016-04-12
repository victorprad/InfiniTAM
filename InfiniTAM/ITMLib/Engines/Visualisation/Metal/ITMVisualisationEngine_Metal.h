// Copyright 2014 Isis Innovation Limited and the authors of InfiniTAM

#pragma once

#ifndef __METALC__

#include "../CPU/ITMVisualisationEngine_CPU.h"

#if (defined __OBJC__)
struct VisualisationEngine_MetalBits
{
    id<MTLFunction> f_genericRaycastVH_device;
    id<MTLComputePipelineState> p_genericRaycastVH_device;
    
    id<MTLFunction> f_genericRaycastVHMissingPoints_device;
    id<MTLComputePipelineState> p_genericRaycastVHMissingPoints_device;
    
    id<MTLFunction> f_forwardProject_device;
    id<MTLComputePipelineState> p_forwardProject_device;
    
    id<MTLFunction> f_renderICP_device;
    id<MTLComputePipelineState> p_renderICP_device;
    
    id<MTLFunction> f_renderForward_device;
    id<MTLComputePipelineState> p_renderForward_device;
    
    id<MTLBuffer> paramsBuffer_visualisation;
};
#endif

namespace ITMLib
{
    template<class TVoxel, class TIndex>
    class ITMVisualisationEngine_Metal : public ITMVisualisationEngine_CPU < TVoxel, TIndex >
    { };
    
    template<class TVoxel>
    class ITMVisualisationEngine_Metal<TVoxel, ITMVoxelBlockHash> : public ITMVisualisationEngine_CPU < TVoxel, ITMVoxelBlockHash >
    {
    private:
#if (defined __OBJC__)
        VisualisationEngine_MetalBits metalBits;
#endif
        
    public:
        void CreateICPMaps(const ITMScene<TVoxel,ITMVoxelBlockHash> *scene, const ITMView *view, ITMTrackingState *trackingState, ITMRenderState *renderState) const;
        void ForwardRender(const ITMScene<TVoxel,ITMVoxelBlockHash> *scene, const ITMView *view, ITMTrackingState *trackingState, ITMRenderState *renderState) const;
        
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