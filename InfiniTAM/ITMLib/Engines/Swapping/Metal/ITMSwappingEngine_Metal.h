// Copyright 2014 Isis Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../Interface/ITMSwappingEngine.h"

namespace ITMLib
{
    template<class TVoxel, class TIndex>
    class ITMSwappingEngine_Metal : public ITMSwappingEngine<TVoxel,TIndex>
    {
    public:
        void IntegrateGlobalIntoLocal(ITMScene<TVoxel,TIndex> *scene, ITMView *view) {}
        void SaveToGlobalMemory(ITMScene<TVoxel,TIndex> *scene, ITMView *view) {}
    };
    
    template<class TVoxel>
    class ITMSwappingEngine_Metal<TVoxel,ITMVoxelBlockHash> : public ITMSwappingEngine<TVoxel,ITMVoxelBlockHash>
    {
    protected:
        int DownloadFromGlobalMemory(ITMScene<TVoxel,ITMVoxelBlockHash> *scene, ITMView *view);
        
    public:
        // This class is currently just for debugging purposes -- swaps Metal memory to Metal memory.
        // Potentially this could stream into the host memory from somwhere else (disk, database, etc.).
        
        void IntegrateGlobalIntoLocal(ITMScene<TVoxel,ITMVoxelBlockHash> *scene, ITMView *view);
        void SaveToGlobalMemory(ITMScene<TVoxel,ITMVoxelBlockHash> *scene, ITMView *view);
        
        ITMSwappingEngine_Metal(void);
        ~ITMSwappingEngine_Metal(void);
    };
}
