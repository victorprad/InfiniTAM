// Copyright 2014 Isis Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../../ITMSceneReconstructionEngine.h"

namespace ITMLib
{
	namespace Engine
	{
		template<class TVoxel, class TIndex>
		class ITMSceneReconstructionEngine_CPU : public ITMSceneReconstructionEngine<TVoxel,TIndex>
		{};

		template<class TVoxel>
		class ITMSceneReconstructionEngine_CPU<TVoxel,ITMVoxelBlockHash> : public ITMSceneReconstructionEngine<TVoxel,ITMVoxelBlockHash>
		{
		private:
			unsigned char *entriesAllocType;
			Vector3s *blockCoords;

		public:
			void AllocateSceneFromDepth(ITMScene<TVoxel,ITMVoxelBlockHash> *scene, const ITMView *view, const ITMPose *pose);
			
			void IntegrateIntoScene(ITMScene<TVoxel,ITMVoxelBlockHash> *scene, const ITMView *view, const ITMPose *pose);

			ITMSceneReconstructionEngine_CPU(void);
			~ITMSceneReconstructionEngine_CPU(void);
		};

		template<class TVoxel>
		class ITMSceneReconstructionEngine_CPU<TVoxel,ITMPlainVoxelArray> : public ITMSceneReconstructionEngine<TVoxel,ITMPlainVoxelArray>
		{
		private:
			unsigned char *entriesAllocType;
			Vector3s *blockCoords;

		public:
			void AllocateSceneFromDepth(ITMScene<TVoxel,ITMPlainVoxelArray> *scene, const ITMView *view, const ITMPose *pose);
			
			void IntegrateIntoScene(ITMScene<TVoxel,ITMPlainVoxelArray> *scene, const ITMView *view, const ITMPose *pose);

			ITMSceneReconstructionEngine_CPU(void);
			~ITMSceneReconstructionEngine_CPU(void);
		};
	}
}
