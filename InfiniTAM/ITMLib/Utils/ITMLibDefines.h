// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM

#pragma once

#ifndef __METALC__
#ifdef NDEBUG
#undef NDEBUG
#include <assert.h>
#define NDEBUG
#else
#include <assert.h>
#endif // NDEBUG
#endif

/// Kinect2 support is disabled by default (to not add the Kinect2 SDK dependency)
#ifndef COMPILE_WITHOUT_Kinect2
#define COMPILE_WITHOUT_Kinect2
#endif

#include "ITMVoxelTypes.h"
#include "../Objects/ITMPlainVoxelArray.h"
#include "../Objects/ITMVoxelBlockHash.h"

/** This chooses the information stored at each voxel. At the moment, valid
    options are ITMVoxel_s, ITMVoxel_f, ITMVoxel_s_rgb and ITMVoxel_f_rgb 
*/
typedef ITMVoxel_s ITMVoxel;

/** This chooses the way the voxels are addressed and indexed. At the moment,
    valid options are ITMVoxelBlockHash and ITMPlainVoxelArray.
*/
typedef ITMLib::ITMVoxelBlockHash ITMVoxelIndex;
//typedef ITMLib::ITMPlainVoxelArray ITMVoxelIndex;

//debug
#ifndef DEBUGBREAK
#define DEBUGBREAK \
{ \
	int ryifrklaeybfcklarybckyar=0; \
	ryifrklaeybfcklarybckyar++; \
}
#endif
