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

#ifndef COMPILE_WITHOUT_CUDA

#include <cuda_runtime.h>

#ifndef ITMSafeCall
#define ITMSafeCall ORcudaSafeCall
#endif

#endif

#include "ITMVoxelTypes.h"
#include "../Objects/ITMHashSwapState.h"
#include "../Objects/ITMPlainVoxelArray.h"
#include "../Objects/ITMVoxelBlockHash.h"
#include "../../ORUtils/Image.h"

/** This chooses the information stored at each voxel. At the moment, valid
    options are ITMVoxel_s, ITMVoxel_f, ITMVoxel_s_rgb and ITMVoxel_f_rgb 
*/
typedef ITMVoxel_s ITMVoxel;

/** This chooses the way the voxels are addressed and indexed. At the moment,
    valid options are ITMVoxelBlockHash and ITMPlainVoxelArray.
*/
typedef ITMLib::ITMVoxelBlockHash ITMVoxelIndex;
//typedef ITMLib::ITMPlainVoxelArray ITMVoxelIndex;

//////////////////////////////////////////////////////////////////////////
// Do not change below this point
//////////////////////////////////////////////////////////////////////////
#ifndef ITMFloatImage
#define ITMFloatImage ORUtils::Image<float>
#endif

#ifndef ITMFloat2Image
#define ITMFloat2Image ORUtils::Image<Vector2f>
#endif

#ifndef ITMFloat4Image
#define ITMFloat4Image ORUtils::Image<Vector4f>
#endif

#ifndef ITMShortImage
#define ITMShortImage ORUtils::Image<short>
#endif

#ifndef ITMShort3Image
#define ITMShort3Image ORUtils::Image<Vector3s>
#endif

#ifndef ITMShort4Image
#define ITMShort4Image ORUtils::Image<Vector4s>
#endif

#ifndef ITMUShortImage
#define ITMUShortImage ORUtils::Image<ushort>
#endif

#ifndef ITMUIntImage
#define ITMUIntImage ORUtils::Image<uint>
#endif

#ifndef ITMIntImage
#define ITMIntImage ORUtils::Image<int>
#endif

#ifndef ITMUCharImage
#define ITMUCharImage ORUtils::Image<uchar>
#endif

#ifndef ITMUChar4Image
#define ITMUChar4Image ORUtils::Image<Vector4u>
#endif

#ifndef ITMBoolImage
#define ITMBoolImage ORUtils::Image<bool>
#endif

//debug
#ifndef DEBUGBREAK
#define DEBUGBREAK \
{ \
	int ryifrklaeybfcklarybckyar=0; \
	ryifrklaeybfcklarybckyar++; \
}
#endif
