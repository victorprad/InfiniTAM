// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#include "CPU/ITMMultiVisualisationEngine_CPU.h"
#ifndef COMPILE_WITHOUT_CUDA
#include "CUDA/ITMMultiVisualisationEngine_CUDA.h"
#endif
#ifdef COMPILE_WITH_METAL
//#include "Metal/ITMMultiVisualisationEngine_Metal.h"
#endif

namespace ITMLib
{

	/**
	 * \brief This struct provides functions that can be used to construct visualisation engines.
	 */
	struct ITMMultiVisualisationEngineFactory
	{
		//#################### PUBLIC STATIC MEMBER FUNCTIONS ####################

		/**
		 * \brief Makes a visualisation engine.
		 *
		 * \param deviceType  The device on which the visualisation engine should operate.
		 */
		template <typename TVoxel, typename TIndex>
		static ITMMultiVisualisationEngine<TVoxel, TIndex> *MakeVisualisationEngine(ITMLibSettings::DeviceType deviceType)
		{
			ITMMultiVisualisationEngine<TVoxel, TIndex> *visualisationEngine = NULL;

			switch (deviceType)
			{
			case ITMLibSettings::DEVICE_CPU:
				visualisationEngine = new ITMMultiVisualisationEngine_CPU<TVoxel, TIndex>;
				break;
			case ITMLibSettings::DEVICE_CUDA:
#ifndef COMPILE_WITHOUT_CUDA
				visualisationEngine = new ITMMultiVisualisationEngine_CUDA<TVoxel, TIndex>;
#endif
				break;
			case ITMLibSettings::DEVICE_METAL:
#ifdef COMPILE_WITH_METAL
				visualisationEngine = new ITMMultiVisualisationEngine_CPU<TVoxel, TIndex>;
#endif
				break;
			}

			return visualisationEngine;
		}
	};

}
