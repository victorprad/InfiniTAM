// Copyright 2014 Isis Innovation Limited and the authors of InfiniTAM

#include "ITMVisualisationEngine_CUDA.h"
#include "ITMCUDAUtils.h"
#include "../../DeviceAgnostic/ITMSceneReconstructionEngine.h"
#include "../../DeviceAgnostic/ITMVisualisationEngine.h"

using namespace ITMLib::Engine;

inline dim3 getGridSize(dim3 taskSize, dim3 blockSize)
{
	return dim3((taskSize.x + blockSize.x - 1) / blockSize.x, (taskSize.y + blockSize.y - 1) / blockSize.y, (taskSize.z + blockSize.z - 1) / blockSize.z);
}

inline dim3 getGridSize(Vector2i taskSize, dim3 blockSize) { return getGridSize(dim3(taskSize.x, taskSize.y), blockSize); }

// declaration of device functions

__global__ void buildVisibleList_device(const ITMHashEntry *hashTable, /*ITMHashCacheState *cacheStates, bool useSwapping,*/ int noTotalEntries,
	int *liveEntryIDs, int *noLiveEntries, uchar *entriesVisibleType, Matrix4f M, Vector4f projParams, Vector2i imgSize, float voxelSize);

template<typename T>
__global__ void memsetKernel_device(T *devPtr, const T val, size_t nwords);

__global__ void projectAndSplitBlocks_device(const ITMHashEntry *hashEntries, const int *liveEntryIDs, int noLiveEntries,
	const Matrix4f pose_M, const Vector4f intrinsics, const Vector2i imgSize, float voxelSize, RenderingBlock *renderingBlocks,
	uint *noTotalBlocks);

__device__ static inline void atomicMin(float* address, float val);
__device__ static inline void atomicMax(float* address, float val);
__global__ void fillBlocks_device(const uint *noTotalBlocks, const RenderingBlock *renderingBlocks,
	Vector2i imgSize, Vector2f *minmaxData);

template<class TVoxel, class TIndex>
__global__ void renderImage_device(Vector4u *outRendering,
        const TVoxel *voxelData, const typename TIndex::IndexData *voxelIndex, Vector2i imgSize, Matrix4f invM, Vector4f projParams,
        float oneOverVoxelSize, const Vector2f *minmaxdata, float mu, Vector3f lightSource, bool useColour);

template<class TVoxel, class TIndex>
__global__ void createPointCloud_device(uint *noTotalPoints, Vector4f *colours, Vector4f *locations, Vector4u *outRendering,
	const TVoxel *voxelData, const typename TIndex::IndexData *voxelIndex, Vector2i imgSize, Matrix4f invM, Vector4f projParams, float voxelSize, 
	float oneOverVoxelSize, const Vector2f *minmaxdata, float mu, bool skipPoints, Vector3f lightSource);

template<class TVoxel, class TIndex>
__global__ void createICPMaps_device(float *depth, Vector4f *pointsMap, Vector4f *normalsMap, Vector4u *outRendering, 
	const TVoxel *voxelData, const typename TIndex::IndexData *voxelIndex, Vector2i imgSize, Matrix4f invM, Vector4f projParams, 
	float voxelSize, float oneOverVoxelSize, const Vector2f *minmaxdata, float mu, 
	Vector3f lightSource);

// class implementation

template<class TVoxel>
ITMVisualisationEngine_CUDA<TVoxel,ITMVoxelBlockHash>::State::State(const Vector2i & imgSize)
 : ITMVisualisationState(imgSize, true)
{
	static const int noTotalEntries = ITMVoxelBlockHash::noVoxelBlocks;
	ITMSafeCall(cudaMalloc((void**)&entriesVisibleType, sizeof(uchar) * noTotalEntries));
	ITMSafeCall(cudaMalloc((void**)&visibleEntryIDs, sizeof(int) * SDF_LOCAL_BLOCK_NUM));
	ITMSafeCall(cudaMalloc((void**)&visibleEntriesNum_ptr, sizeof(int)));
}

template<class TVoxel>
ITMVisualisationEngine_CUDA<TVoxel,ITMVoxelBlockHash>::State::~State(void)
{
	ITMSafeCall(cudaFree(entriesVisibleType));
	ITMSafeCall(cudaFree(visibleEntryIDs));
	ITMSafeCall(cudaFree(visibleEntriesNum_ptr));
}

template<class TVoxel, class TIndex>
ITMVisualisationEngine_CUDA<TVoxel,TIndex>::ITMVisualisationEngine_CUDA(void)
{
	ITMSafeCall(cudaMalloc((void**)&noTotalPoints_device, sizeof(uint)));
}

template<class TVoxel, class TIndex>
ITMVisualisationEngine_CUDA<TVoxel,TIndex>::~ITMVisualisationEngine_CUDA(void)
{
	ITMSafeCall(cudaFree(noTotalPoints_device));
}

template<class TVoxel>
ITMVisualisationEngine_CUDA<TVoxel,ITMVoxelBlockHash>::ITMVisualisationEngine_CUDA(void)
{
	ITMSafeCall(cudaMalloc((void**)&renderingBlockList_device, sizeof(RenderingBlock) * MAX_RENDERING_BLOCKS));
	ITMSafeCall(cudaMalloc((void**)&noTotalBlocks_device, sizeof(uint)));
	ITMSafeCall(cudaMalloc((void**)&noTotalPoints_device, sizeof(uint)));
}

template<class TVoxel>
ITMVisualisationEngine_CUDA<TVoxel,ITMVoxelBlockHash>::~ITMVisualisationEngine_CUDA(void)
{
	ITMSafeCall(cudaFree(noTotalPoints_device));
	ITMSafeCall(cudaFree(noTotalBlocks_device));
	ITMSafeCall(cudaFree(renderingBlockList_device));
}

template<class TVoxel, class TIndex>
void ITMVisualisationEngine_CUDA<TVoxel,TIndex>::FindVisibleBlocks(const ITMScene<TVoxel,TIndex> *scene, const ITMPose *pose, const ITMIntrinsics *intrinsics, ITMVisualisationState *_state)
{
}

template<class TVoxel>
void ITMVisualisationEngine_CUDA<TVoxel,ITMVoxelBlockHash>::FindVisibleBlocks(const ITMScene<TVoxel,ITMVoxelBlockHash> *scene, const ITMPose *pose, const ITMIntrinsics *intrinsics, ITMVisualisationState *_state)
{
	State *state = (State*)_state;
	const ITMHashEntry *hashTable = scene->index.GetEntries();
	int noTotalEntries = scene->index.noVoxelBlocks;
	float voxelSize = scene->sceneParams->voxelSize;
	Vector2i imgSize = state->minmaxImage->noDims;

	Matrix4f M = pose->M;
	Vector4f projParams = intrinsics->projectionParamsSimple.all;

	ITMSafeCall(cudaMemset(state->visibleEntriesNum_ptr, 0, sizeof(int)));

	dim3 cudaBlockSizeAL(256, 1);
	dim3 gridSizeAL((int)ceil((float)noTotalEntries / (float)cudaBlockSizeAL.x));
	buildVisibleList_device << <gridSizeAL, cudaBlockSizeAL >> >(hashTable, /*cacheStates, scene->useSwapping,*/ noTotalEntries, state->visibleEntryIDs,
		state->visibleEntriesNum_ptr, state->entriesVisibleType, M, projParams, imgSize, voxelSize);

/*	if (scene->useSwapping)
        {
                reAllocateSwappedOutVoxelBlocks_device << <gridSizeAL, cudaBlockSizeAL >> >(voxelAllocationList, hashTable, noTotalEntries,
                        noAllocatedVoxelEntries_device, entriesVisibleType);
        }*/

        ITMSafeCall(cudaMemcpy(&state->visibleEntriesNum, state->visibleEntriesNum_ptr, sizeof(int), cudaMemcpyDeviceToHost));
}

template<class TVoxel, class TIndex>
void ITMVisualisationEngine_CUDA<TVoxel,TIndex>::CreateExpectedDepths(const ITMScene<TVoxel,TIndex> *scene, const ITMPose *pose,
	const ITMIntrinsics *intrinsics, ITMFloat2Image *minmaximg, const ITMVisualisationState *state)
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

template<class TVoxel>
void ITMVisualisationEngine_CUDA<TVoxel,ITMVoxelBlockHash>::CreateExpectedDepths(const ITMScene<TVoxel,ITMVoxelBlockHash> *scene, const ITMPose *pose,
	const ITMIntrinsics *intrinsics, ITMFloat2Image *minmaximg, const ITMVisualisationState *state)
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
		if (state != NULL) {
			const State *s = (const State*)state;
			liveEntryIDs = s->visibleEntryIDs;
			noLiveEntries = s->visibleEntriesNum;
		}

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
		dim3 gridSize(ceil((float)noTotalBlocks/4.0f), 4);
		fillBlocks_device << <gridSize, blockSize >> >(noTotalBlocks_device, renderingBlockList_device, imgSize, minmaxData);
	}
}

template<class TVoxel, class TIndex>
static void RenderImage_common(const ITMScene<TVoxel,TIndex> *scene, const ITMPose *pose, const ITMIntrinsics *intrinsics, const ITMVisualisationState *state, ITMUChar4Image *outputImage, bool useColour)
{
	Vector2i imgSize = outputImage->noDims;
	float oneOverVoxelSize = 1.0f / scene->sceneParams->voxelSize;

	Matrix4f invM = pose->invM;
	Vector4f projParams = intrinsics->projectionParamsSimple.all;
	projParams.x = 1.0f / projParams.x;
	projParams.y = 1.0f / projParams.y;

	float mu = scene->sceneParams->mu;
	Vector3f lightSource = -Vector3f(invM.getColumn(2));

	Vector4u *outRendering = outputImage->GetData(true);
	const Vector2f *minmaximg = state->minmaxImage->GetData(true);

	dim3 cudaBlockSize(8, 8);
	dim3 gridSize((int)ceil((float)imgSize.x / (float)cudaBlockSize.x), (int)ceil((float)imgSize.y / (float)cudaBlockSize.y));

	renderImage_device<TVoxel,TIndex> << <gridSize, cudaBlockSize >> >(outRendering,
		scene->localVBA.GetVoxelBlocks(), scene->index.getIndexData(), imgSize, invM, projParams, oneOverVoxelSize, minmaximg,
		mu, lightSource, useColour);
}

template<class TVoxel, class TIndex>
static void CreatePointCloud_common(const ITMScene<TVoxel,TIndex> *scene, const ITMView *view, ITMTrackingState *trackingState, bool skipPoints, uint *noTotalPoints_device)
{
	Vector2i imgSize = view->rgb->noDims;
	float voxelSize = scene->sceneParams->voxelSize;
	float oneOverVoxelSize = 1.0f / scene->sceneParams->voxelSize;

	Matrix4f invM = trackingState->pose_d->invM * view->calib->trafo_rgb_to_depth.calib;
	Vector4f projParams = view->calib->intrinsics_rgb.projectionParamsSimple.all;
	projParams.x = 1.0f / projParams.x;
	projParams.y = 1.0f / projParams.y;

	Vector4f *locations = trackingState->pointCloud->locations->GetData(true);
	Vector4f *colours = trackingState->pointCloud->colours->GetData(true);
	Vector4u *outRendering = trackingState->rendering->GetData(true);
	Vector2f *minmaxdata = trackingState->renderingRangeImage->GetData(true);

	float mu = scene->sceneParams->mu;
	Vector3f lightSource = -Vector3f(invM.getColumn(2));

	dim3 cudaBlockSize(16, 16);
	dim3 gridSize = getGridSize(imgSize, cudaBlockSize);

	ITMSafeCall(cudaMemset(noTotalPoints_device, 0, sizeof(uint)));
	ITMSafeCall(cudaMemset(outRendering, 0, sizeof(Vector4u) * imgSize.x * imgSize.y));

	createPointCloud_device<TVoxel,TIndex> << <gridSize, cudaBlockSize >> >(noTotalPoints_device, colours, locations, outRendering,
		scene->localVBA.GetVoxelBlocks(), scene->index.getIndexData(), imgSize, invM, projParams, voxelSize, oneOverVoxelSize, minmaxdata,
		mu, skipPoints, lightSource);

	ITMSafeCall(cudaMemcpy(&trackingState->pointCloud->noTotalPoints, noTotalPoints_device, sizeof(uint), cudaMemcpyDeviceToHost));
}

template<class TVoxel, class TIndex>
void CreateICPMaps_common(const ITMScene<TVoxel,TIndex> *scene, const ITMView *view, ITMTrackingState *trackingState)
{
	Vector2i imgSize = view->depth->noDims;
	float voxelSize = scene->sceneParams->voxelSize;
	float oneOverVoxelSize = 1.0f / scene->sceneParams->voxelSize;

	Matrix4f invM = trackingState->pose_d->invM;
	Vector4f projParams = view->calib->intrinsics_d.projectionParamsSimple.all;
	projParams.x = 1.0f / projParams.x;
	projParams.y = 1.0f / projParams.y;

	float mu = scene->sceneParams->mu;
	Vector3f lightSource = -Vector3f(invM.getColumn(2));

	float *depth = view->depth->GetData(true);

	Vector4f *pointsMap = trackingState->pointCloud->locations->GetData(true);
	Vector4f *normalsMap = trackingState->pointCloud->colours->GetData(true);
	Vector4u *outRendering = trackingState->rendering->GetData(true);
	Vector2f *minmaxdata = trackingState->renderingRangeImage->GetData(true);

	dim3 cudaBlockSize(8, 8);
	dim3 gridSize((int)ceil((float)imgSize.x / (float)cudaBlockSize.x), (int)ceil((float)imgSize.y / (float)cudaBlockSize.y));

	ITMSafeCall(cudaMemset(outRendering, 0, sizeof(Vector4u)* imgSize.x * imgSize.y));
	ITMSafeCall(cudaMemset(pointsMap, 0, sizeof(Vector4f)* imgSize.x * imgSize.y));
	ITMSafeCall(cudaMemset(normalsMap, 0, sizeof(Vector4f)* imgSize.x * imgSize.y));

	createICPMaps_device<TVoxel,TIndex> << <gridSize, cudaBlockSize >> >(depth, pointsMap, normalsMap, outRendering,
		scene->localVBA.GetVoxelBlocks(), scene->index.getIndexData(), imgSize, invM, projParams, voxelSize, oneOverVoxelSize, minmaxdata, 
		mu, lightSource);

	ITMSafeCall(cudaThreadSynchronize());
}

template<class TVoxel, class TIndex>
void ITMVisualisationEngine_CUDA<TVoxel,TIndex>::RenderImage(const ITMScene<TVoxel,TIndex> *scene, const ITMPose *pose, const ITMIntrinsics *intrinsics, const ITMVisualisationState *state, ITMUChar4Image *outputImage, bool useColour)
{
	RenderImage_common(scene, pose, intrinsics, state, outputImage, useColour);
}

template<class TVoxel>
void ITMVisualisationEngine_CUDA<TVoxel,ITMVoxelBlockHash>::RenderImage(const ITMScene<TVoxel,ITMVoxelBlockHash> *scene, const ITMPose *pose, const ITMIntrinsics *intrinsics, const ITMVisualisationState *state, ITMUChar4Image *outputImage, bool useColour)
{
	RenderImage_common(scene, pose, intrinsics, state, outputImage, useColour);
}

template<class TVoxel, class TIndex>
void ITMVisualisationEngine_CUDA<TVoxel,TIndex>::CreatePointCloud(const ITMScene<TVoxel,TIndex> *scene, const ITMView *view, ITMTrackingState *trackingState, bool skipPoints)
{
	CreatePointCloud_common(scene, view, trackingState, skipPoints, noTotalPoints_device);
}

template<class TVoxel>
void ITMVisualisationEngine_CUDA<TVoxel,ITMVoxelBlockHash>::CreatePointCloud(const ITMScene<TVoxel,ITMVoxelBlockHash> *scene, const ITMView *view, ITMTrackingState *trackingState, bool skipPoints)
{
	CreatePointCloud_common(scene, view, trackingState, skipPoints, noTotalPoints_device);
}

template<class TVoxel, class TIndex>
void ITMVisualisationEngine_CUDA<TVoxel,TIndex>::CreateICPMaps(const ITMScene<TVoxel,TIndex> *scene, const ITMView *view, ITMTrackingState *trackingState)
{
	CreateICPMaps_common(scene, view, trackingState);
}

template<class TVoxel>
void ITMVisualisationEngine_CUDA<TVoxel,ITMVoxelBlockHash>::CreateICPMaps(const ITMScene<TVoxel,ITMVoxelBlockHash> *scene, const ITMView *view, ITMTrackingState *trackingState)
{
	CreateICPMaps_common(scene, view, trackingState);
}

//device implementations

__global__ void buildVisibleList_device(const ITMHashEntry *hashTable, /*ITMHashCacheState *cacheStates, bool useSwapping,*/ int noTotalEntries,
	int *liveEntryIDs, int *noLiveEntries, uchar *entriesVisibleType, Matrix4f M, Vector4f projParams, Vector2i imgSize, float voxelSize)
{
	int targetIdx = threadIdx.x + blockIdx.x * blockDim.x;
	if (targetIdx > noTotalEntries - 1) return;

	__shared__ bool shouldPrefix;

	unsigned char hashVisibleType = 0; //entriesVisibleType[targetIdx];
	const ITMHashEntry &hashEntry = hashTable[targetIdx];

	shouldPrefix = false;
	__syncthreads();

	if (hashEntry.ptr >= 0)
	{
		shouldPrefix = true;

		Vector3f pt_image, buff3f;

		int noInvisible = 0;//, noInvisibleEnlarged = 0;

/*		pt_image = hashEntry.pos.toFloat() * (float)SDF_BLOCK_SIZE * voxelSize;
		buff3f = M * pt_image;

		if (buff3f.z > 1e-10f)
		{
			shouldPrefix = true;*/

		for (int x = 0; x <= 1; x++) for (int y = 0; y <= 1; y++) for (int z = 0; z <= 1; z++)
		{
			Vector3f off((float)x, (float)y, (float)z);

			pt_image = (hashEntry.pos.toFloat() + off) * (float)SDF_BLOCK_SIZE * voxelSize;

			buff3f = M * pt_image;

			if (buff3f.z < 1e-10f) continue;

			pt_image.x = projParams.x * buff3f.x / buff3f.z + projParams.z;
			pt_image.y = projParams.y * buff3f.y / buff3f.z + projParams.w;

			if (!(pt_image.x >= 0 && pt_image.x < imgSize.x && pt_image.y >= 0 && pt_image.y < imgSize.y)) noInvisible++;

/*			if (useSwapping)
			{
				Vector4i lims;
				lims.x = -imgSize.x / 8; lims.y = imgSize.x + imgSize.x / 8;
				lims.z = -imgSize.y / 8; lims.w = imgSize.y + imgSize.y / 8;

				if (!(pt_image.x >= lims.x && pt_image.x < lims.y && pt_image.y >= lims.z && pt_image.y < lims.w)) noInvisibleEnlarged++;
			}*/
		}

		hashVisibleType = noInvisible < 8;

		//if (useSwapping) entriesVisibleType[targetIdx] = noInvisibleEnlarged < 8;
	}

	/*if (useSwapping)
	{
		if (entriesVisibleType[targetIdx] > 0 && cacheStates[targetIdx].cacheFromHost != 2) cacheStates[targetIdx].cacheFromHost = 1;
	}*/

	if (hashVisibleType > 0) shouldPrefix = true;

	__syncthreads();

	if (shouldPrefix)
	{
		int offset = computePrefixSum_device<int>(hashVisibleType > 0, noLiveEntries, blockDim.x * blockDim.y, threadIdx.x);
		if (offset != -1) liveEntryIDs[offset] = targetIdx;
	}
}

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
	int block = blockIdx.x*4 + blockIdx.y;
	if (block >= *noTotalBlocks) return;

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
__global__ void renderImage_device(Vector4u *outRendering,
	const TVoxel *voxelData, const typename TIndex::IndexData *voxelIndex, Vector2i imgSize, Matrix4f invM, Vector4f projParams,
	float oneOverVoxelSize, const Vector2f *minmaxdata, float mu, Vector3f lightSource, bool useColour)
{
	int x = (threadIdx.x + blockIdx.x * blockDim.x), y = (threadIdx.y + blockIdx.y * blockDim.y);

	if (x >= imgSize.x || y >= imgSize.y) return;

	Vector3f pt_ray; Vector4f tmp;
	Vector3f outNormal;
	float angle = 0.0f;

	int locId = x + y * imgSize.x;

	float viewFrustum_min = minmaxdata[locId].x;
	float viewFrustum_max = minmaxdata[locId].y;

	bool foundPoint = false;
	foundPoint = castRay<TVoxel,TIndex>(pt_ray, x, y, voxelData, voxelIndex, invM, projParams, imgSize, oneOverVoxelSize, mu, viewFrustum_min, viewFrustum_max);

	if (foundPoint)
	{
		outNormal = computeSingleNormalFromSDF(voxelData, voxelIndex, pt_ray);

		float normScale = 1.0f / sqrtf(outNormal.x * outNormal.x + outNormal.y * outNormal.y + outNormal.z * outNormal.z);
		outNormal *= normScale;

		angle = outNormal.x * lightSource.x + outNormal.y * lightSource.y + outNormal.z * lightSource.z;
		if (!(angle > 0.0)) foundPoint = false;
	}

	Vector4u& outRef = outRendering[locId];
	if (foundPoint)
	{
		if (useColour && TVoxel::hasColorInformation)
		{
			Vector4f clr = VoxelColorReader<TVoxel::hasColorInformation,TVoxel,typename TIndex::IndexData>::interpolate(voxelData, voxelIndex, pt_ray);
			outRef.x = (uchar)(clr.r * 255.0f);
			outRef.y = (uchar)(clr.g * 255.0f);
			outRef.z = (uchar)(clr.b * 255.0f);
			outRef.w = 255;
		}
		else
		{
			float outRes = (0.8f * angle + 0.2f) * 255.0f;
			outRef = Vector4u(outRes);
		}
	}
	else
	{
		outRef = Vector4u((const uchar&)0);
	}
}

template<class TVoxel, class TIndex>
__global__ void createPointCloud_device(uint *noTotalPoints, Vector4f *colours, Vector4f *locations, Vector4u *outRendering,
	const TVoxel *voxelData, const typename TIndex::IndexData *voxelIndex, Vector2i imgSize, Matrix4f invM, Vector4f projParams, float voxelSize, 
	float oneOverVoxelSize, const Vector2f *minmaxdata, float mu, bool skipPoints, Vector3f lightSource)
{
	int x = (threadIdx.x + blockIdx.x * blockDim.x), y = (threadIdx.y + blockIdx.y * blockDim.y);

	if (x >= imgSize.x || y >= imgSize.y) return;

	__shared__ bool shouldPrefix;

	Vector3f pt_ray; Vector4f tmp;
	float angle = 0.0f;

	int locId = x + y * imgSize.x;

	float viewFrustum_min = minmaxdata[locId].x;
	float viewFrustum_max = minmaxdata[locId].y;

	shouldPrefix = false;

	bool foundPoint = false;
	foundPoint = castRay<TVoxel,TIndex>(pt_ray, x, y, voxelData, voxelIndex, invM, projParams, imgSize, oneOverVoxelSize, mu, viewFrustum_min, viewFrustum_max);

	if (foundPoint)
	{
		Vector3f outNormal = computeSingleNormalFromSDF(voxelData, voxelIndex, pt_ray);

		float normScale = 1.0f / sqrtf(outNormal.x * outNormal.x + outNormal.y * outNormal.y + outNormal.z * outNormal.z);
		outNormal *= normScale;

		angle = outNormal.x * lightSource.x + outNormal.y * lightSource.y + outNormal.z * lightSource.z;
		if (!(angle>0.0f)) foundPoint = false;
	}
	if (foundPoint)
	{
		float outRes = (0.8f * angle + 0.2f) * 255.0f;

		outRendering[x + y * imgSize.x] = (uchar)outRes;

		shouldPrefix = true;
	}

	__syncthreads();

	if (skipPoints && ((x % 2 == 0) || (y % 2 == 0))) foundPoint = false;

	if (shouldPrefix)
	{
		int offset = computePrefixSum_device<uint>(foundPoint, noTotalPoints, blockDim.x * blockDim.y, threadIdx.x + threadIdx.y * blockDim.x);

		if (offset != -1)
		{
			Vector4f pt_ray_out;

			tmp = VoxelColorReader<TVoxel::hasColorInformation,TVoxel,typename TIndex::IndexData>::interpolate(voxelData, voxelIndex, pt_ray);
			if (tmp.w > 0.0f) { tmp.x /= tmp.w; tmp.y /= tmp.w; tmp.z /= tmp.w; tmp.w = 1.0f; }
			colours[offset] = tmp;

			pt_ray_out.x = pt_ray.x * voxelSize; pt_ray_out.y = pt_ray.y * voxelSize;
			pt_ray_out.z = pt_ray.z * voxelSize; pt_ray_out.w = 1.0f;
			locations[offset] = pt_ray_out;
		}
	}
}

template<class TVoxel, class TIndex>
__global__ void createICPMaps_device(float *depth, Vector4f *pointsMap, Vector4f *normalsMap, Vector4u *outRendering, 
	const TVoxel *voxelData, const typename TIndex::IndexData *voxelIndex, Vector2i imgSize, Matrix4f invM, Vector4f projParams, 
	float voxelSize, float oneOverVoxelSize, const Vector2f *minmaxdata, float mu, 
	Vector3f lightSource)
{
	int x = (threadIdx.x + blockIdx.x * blockDim.x), y = (threadIdx.y + blockIdx.y * blockDim.y);

	if (x >= imgSize.x || y >= imgSize.y) return;

	Vector3f pt_ray; Vector4f tmp;
	Vector3f outNormal;
	float angle = 0.0f;

	int locId = x + y * imgSize.x;

	float viewFrustum_min = minmaxdata[locId].x;
	float viewFrustum_max = minmaxdata[locId].y;

	bool foundPoint = false;
	foundPoint = castRay<TVoxel,TIndex>(pt_ray, x, y, voxelData, voxelIndex, invM, projParams, imgSize, oneOverVoxelSize, mu, viewFrustum_min, viewFrustum_max);

	if (foundPoint)
	{
		outNormal = computeSingleNormalFromSDF(voxelData, voxelIndex, pt_ray);

		float normScale = 1.0f / sqrtf(outNormal.x * outNormal.x + outNormal.y * outNormal.y + outNormal.z * outNormal.z);
		outNormal *= normScale;

		angle = outNormal.x * lightSource.x + outNormal.y * lightSource.y + outNormal.z * lightSource.z;
		if (!(angle > 0.0)) foundPoint = false;
	}

	if (foundPoint)
	{
		float outRes = (0.8f * angle + 0.2f) * 255.0f;

		outRendering[x + y * imgSize.x] = (uchar)outRes;

		Vector4f outNormal4;
		outNormal4.x = outNormal.x; outNormal4.y = outNormal.y; outNormal4.z = outNormal.z; outNormal4.w = 0.0f;

		Vector4f outPoint4;
		outPoint4.x = pt_ray.x * voxelSize; outPoint4.y = pt_ray.y * voxelSize;
		outPoint4.z = pt_ray.z * voxelSize; outPoint4.w = 1.0f;

		pointsMap[locId] = outPoint4;
		normalsMap[locId] = outNormal4;
	}
	else
	{
		Vector4f out4;
		out4.x = 0.0f; out4.y = 0.0f; out4.z = 0.0f; out4.w = -1.0f;

		pointsMap[locId] = out4;
		normalsMap[locId] = out4;
	}
}

template class ITMLib::Engine::ITMVisualisationEngine_CUDA<ITMVoxel, ITMVoxelIndex>;
