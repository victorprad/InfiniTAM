// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#include "CPU/ITMMultiMeshingEngine_CPU.h"
#ifndef COMPILE_WITHOUT_CUDA
#include "CUDA/ITMMultiMeshingEngine_CUDA.h"
#endif

namespace ITMLib
{

	/**
	 * \brief This struct provides functions that can be used to construct meshing engines.
	 */
	struct ITMMultiMeshingEngineFactory
	{
		//#################### PUBLIC STATIC MEMBER FUNCTIONS ####################

		/**
		 * \brief Makes a meshing engine.
		 *
		 * \param deviceType  The device on which the meshing engine should operate.
		 */
		template <typename TVoxel, typename TIndex>
		static ITMMultiMeshingEngine<TVoxel, TIndex> *MakeMeshingEngine(ITMLibSettings::DeviceType deviceType)
		{
			ITMMultiMeshingEngine<TVoxel, TIndex> *meshingEngine = NULL;

			switch (deviceType)
			{
			case ITMLibSettings::DEVICE_CPU:
				meshingEngine = new ITMMultiMeshingEngine_CPU<TVoxel, TIndex>();
				break;
			case ITMLibSettings::DEVICE_CUDA:
#ifndef COMPILE_WITHOUT_CUDA
				meshingEngine = new ITMMultiMeshingEngine_CUDA<TVoxel, TIndex>();
#endif
				break;
			case ITMLibSettings::DEVICE_METAL:
#ifdef COMPILE_WITH_METAL
				meshingEngine = new ITMMultiMeshingEngine_CPU<TVoxel, TIndex>();
#endif
				break;
			}

			return meshingEngine;
		}
	};
}
