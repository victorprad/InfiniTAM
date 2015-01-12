// Copyright 2014 Isis Innovation Limited and the authors of InfiniTAM
#pragma once

#ifndef __METALC__

#include "../../ITMSceneReconstructionEngine.h"

namespace ITMLib
{
    namespace Engine
    {
        template<class TVoxel, class TIndex>
        class ITMSceneReconstructionEngine_Metal : public ITMSceneReconstructionEngine<TVoxel,TIndex>
        {};
        
        template<class TVoxel>
        class ITMSceneReconstructionEngine_Metal<TVoxel,ITMVoxelBlockHash> : public ITMSceneReconstructionEngine<TVoxel,ITMVoxelBlockHash>
        {
        private:
            unsigned char *entriesAllocType;
            Vector3s *blockCoords;
            
        public:
            void AllocateSceneFromDepth(ITMScene<TVoxel,ITMVoxelBlockHash> *scene, const ITMView *view, const ITMPose *pose);
            
            void IntegrateIntoScene(ITMScene<TVoxel,ITMVoxelBlockHash> *scene, const ITMView *view, const ITMPose *pose);
            
            ITMSceneReconstructionEngine_Metal(void);
            ~ITMSceneReconstructionEngine_Metal(void);
        };
        
        template<class TVoxel>
        class ITMSceneReconstructionEngine_Metal<TVoxel,ITMPlainVoxelArray> : public ITMSceneReconstructionEngine<TVoxel,ITMPlainVoxelArray>
        {
        private:
            unsigned char *entriesAllocType;
            Vector3s *blockCoords;
            
        public:
            void AllocateSceneFromDepth(ITMScene<TVoxel,ITMPlainVoxelArray> *scene, const ITMView *view, const ITMPose *pose);
            
            void IntegrateIntoScene(ITMScene<TVoxel,ITMPlainVoxelArray> *scene, const ITMView *view, const ITMPose *pose);
            
            ITMSceneReconstructionEngine_Metal(void);
            ~ITMSceneReconstructionEngine_Metal(void);
        };
    }
}

#endif

struct IntegrateIntoScene_VH_Params
{
    Vector2i rgbImgSize, depthImgSize;
    Matrix4f M_d, M_rgb;
    Vector4f projParams_d, projParams_rgb;
    float _voxelSize, mu, maxW;
};