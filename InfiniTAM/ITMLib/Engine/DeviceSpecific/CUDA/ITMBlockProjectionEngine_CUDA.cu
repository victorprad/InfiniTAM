// Copyright 2014 Isis Innovation Limited and the authors of InfiniTAM

#include "ITMBlockProjectionEngine_CUDA.h"
#include "ITMCUDAUtils.h"
#include "../../DeviceAgnostic/ITMBlockProjectionEngine.h"

using namespace ITMLib::Objects;
using namespace ITMLib::Engine;

template<typename T> __global__ void memsetKernel_device(T *devPtr, const T val, size_t nwords);

__global__ void projectAndSplitBlocks_device(const ITMHashEntry *hashEntries, const int *liveEntryIDs, int noLiveEntries,
	const Matrix4f pose_M, const Vector4f intrinsics, const Vector2i imgSize, float voxelSize, RenderingBlock *renderingBlocks,
	uint *noTotalBlocks);

__device__ static inline void atomicMin(float* address, float val);
__device__ static inline void atomicMax(float* address, float val);
__global__ void fillBlocks_device(const uint *noTotalBlocks, const RenderingBlock *renderingBlocks,
	Vector2i imgSize, Vector2f *minmaxData);

//host methods 

template<class TVoxel>
ITMBlockProjectionEngine_CUDA<TVoxel,ITMVoxelBlockHash>::ITMBlockProjectionEngine_CUDA(void)
{
	ITMSafeCall(cudaMalloc((void**)&renderingBlockList_device, sizeof(RenderingBlock) * MAX_RENDERING_BLOCKS));
	ITMSafeCall(cudaMalloc((void**)&noTotalBlocks_device, sizeof(uint)));
}

template<class TVoxel>
ITMBlockProjectionEngine_CUDA<TVoxel,ITMVoxelBlockHash>::~ITMBlockProjectionEngine_CUDA(void)
{
	ITMSafeCall(cudaFree(noTotalBlocks_device));
	ITMSafeCall(cudaFree(renderingBlockList_device));
}

template<class TVoxel>
void ITMBlockProjectionEngine_CUDA<TVoxel,ITMVoxelBlockHash>::CreateExpectedDepths(const ITMScene<TVoxel,ITMVoxelBlockHash> *scene, const ITMPose *pose,
	const ITMIntrinsics *intrinsics, ITMFloat2Image *minmaximg)
{
	float voxelSize = scene->sceneParams->voxelSize;

	Vector2i imgSize = minmaximg->noDims;
	Vector2f *minmaxData = minmaximg->GetData(true);

	{
		dim3 blockSize(256);
		dim3 gridSize((int)ceil((float)minmaximg->dataSize / (float)blockSize.x));
		Vector2f init;
		init.x = FAR_AWAY; init.y = VERY_CLOSE;
		memsetKernel_device<Vector2f> << <gridSize, blockSize >> >(minmaxData, init, minmaximg->dataSize);
	}

	//go through list of visible 8x8x8 blocks
	{
		const ITMHashEntry *hash_entries = scene->index.GetEntries();
		const int *liveEntryIDs = scene->index.GetLiveEntryIDs();
		int noLiveEntries = scene->index.noLiveEntries;

		dim3 blockSize(256);
		dim3 gridSize((int)ceil((float)noLiveEntries / (float)blockSize.x));
		ITMSafeCall(cudaMemset(noTotalBlocks_device, 0, sizeof(uint)));
		projectAndSplitBlocks_device << <gridSize, blockSize >> >(hash_entries, liveEntryIDs, noLiveEntries, pose->M,
			intrinsics->projectionParamsSimple.all, imgSize, voxelSize, renderingBlockList_device, noTotalBlocks_device);
	}

	uint noTotalBlocks;
	ITMSafeCall(cudaMemcpy(&noTotalBlocks, noTotalBlocks_device, sizeof(uint), cudaMemcpyDeviceToHost));
	if (noTotalBlocks > (unsigned)MAX_RENDERING_BLOCKS) noTotalBlocks = MAX_RENDERING_BLOCKS;

	// go through rendering blocks
	{
		// fill minmaxData
		dim3 blockSize(16, 16);
		dim3 gridSize(noTotalBlocks);
		fillBlocks_device << <gridSize, blockSize >> >(noTotalBlocks_device, renderingBlockList_device, imgSize, minmaxData);
	}
}

// device functions

template<typename T> __global__ void memsetKernel_device(T *devPtr, const T val, size_t nwords)
{
	size_t offset = threadIdx.x + blockDim.x * blockIdx.x;
	if (offset >= nwords) return;
	devPtr[offset] = val;
}


__global__ void projectAndSplitBlocks_device(const ITMHashEntry *hashEntries, const int *liveEntryIDs, int noLiveEntries,
	const Matrix4f pose_M, const Vector4f intrinsics, const Vector2i imgSize, float voxelSize, RenderingBlock *renderingBlocks,
	uint *noTotalBlocks)
{
	int in_offset = threadIdx.x + blockDim.x * blockIdx.x;

	const ITMHashEntry & blockData(hashEntries[liveEntryIDs[in_offset]]);

	Vector2i upperLeft, lowerRight;
	Vector2f zRange;
	bool validProjection = false;
	if (in_offset < noLiveEntries) if (blockData.ptr>=0) {
		validProjection = ProjectSingleBlock(blockData.pos, pose_M, intrinsics, imgSize, voxelSize, upperLeft, lowerRight, zRange);
	}

	Vector2i requiredRenderingBlocks(ceilf((float)(lowerRight.x - upperLeft.x + 1) / renderingBlockSizeX),
		ceilf((float)(lowerRight.y - upperLeft.y + 1) / renderingBlockSizeY));

	size_t requiredNumBlocks = requiredRenderingBlocks.x * requiredRenderingBlocks.y;
	if (!validProjection) requiredNumBlocks = 0;

	int out_offset = computePrefixSum_device<uint>(requiredNumBlocks, noTotalBlocks, blockDim.x, threadIdx.x);
	if (!validProjection) return;
	if ((out_offset == -1) || (out_offset + requiredNumBlocks > MAX_RENDERING_BLOCKS)) return;

	CreateRenderingBlocks(renderingBlocks, out_offset, upperLeft, lowerRight, zRange);
}

__device__ static inline void atomicMin(float* address, float val)
{
	int* address_as_i = (int*)address;
	int old = *address_as_i, assumed;
	do {
		assumed = old;
		old = ::atomicCAS(address_as_i, assumed,
			__float_as_int(::fminf(val, __int_as_float(assumed))));
	} while (assumed != old);
}

__device__ static inline void atomicMax(float* address, float val)
{
	int* address_as_i = (int*)address;
	int old = *address_as_i, assumed;
	do {
		assumed = old;
		old = ::atomicCAS(address_as_i, assumed,
			__float_as_int(::fmaxf(val, __int_as_float(assumed))));
	} while (assumed != old);
}

__global__ void fillBlocks_device(const uint *noTotalBlocks, const RenderingBlock *renderingBlocks,
	Vector2i imgSize, Vector2f *minmaxData)
{
	int x = threadIdx.x;
	int y = threadIdx.y;
	int block = blockIdx.x;

	const RenderingBlock & b(renderingBlocks[block]);
	int xpos = b.upperLeft.x + x;
	if (xpos > b.lowerRight.x) return;
	int ypos = b.upperLeft.y + y;
	if (ypos > b.lowerRight.y) return;

	Vector2f & pixel(minmaxData[xpos + ypos*imgSize.x]);
	atomicMin(&pixel.x, b.zRange.x);
	atomicMax(&pixel.y, b.zRange.y);
}

template<class TVoxel, class TIndex>
void ITMBlockProjectionEngine_CUDA<TVoxel,TIndex>::CreateExpectedDepths(const ITMScene<TVoxel,TIndex> *scene, const ITMPose *pose,
	const ITMIntrinsics *intrinsics, ITMFloat2Image *minmaximg)
{
	Vector2f *minmaxData = minmaximg->GetData(true);

	{
		dim3 blockSize(256);
		dim3 gridSize((int)ceil((float)minmaximg->dataSize / (float)blockSize.x));
		Vector2f init;
		//TODO : this could be improved a bit...
		init.x = 0.2f; init.y = 3.0f;
		memsetKernel_device<Vector2f> << <gridSize, blockSize >> >(minmaxData, init, minmaximg->dataSize);
	}
}

template class ITMBlockProjectionEngine_CUDA<ITMVoxel,ITMVoxelIndex>;
