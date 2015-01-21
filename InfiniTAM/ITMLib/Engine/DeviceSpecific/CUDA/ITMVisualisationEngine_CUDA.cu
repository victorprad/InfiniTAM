// Copyright 2014 Isis Innovation Limited and the authors of InfiniTAM

#include "ITMVisualisationEngine_CUDA.h"
#include "ITMCUDAUtils.h"
#include "../../DeviceAgnostic/ITMRepresentationAccess.h"
#include "../../DeviceAgnostic/ITMVisualisationEngine.h"
#include "../../DeviceAgnostic/ITMSceneReconstructionEngine.h"

#include "../../../Objects/ITMRenderState_VH.h"

using namespace ITMLib::Engine;

inline dim3 getGridSize(dim3 taskSize, dim3 blockSize)
{
	return dim3((taskSize.x + blockSize.x - 1) / blockSize.x, (taskSize.y + blockSize.y - 1) / blockSize.y, (taskSize.z + blockSize.z - 1) / blockSize.z);
}

inline dim3 getGridSize(Vector2i taskSize, dim3 blockSize) { return getGridSize(dim3(taskSize.x, taskSize.y), blockSize); }

// declaration of device functions

__global__ void buildVisibleList_device(const ITMHashEntry *hashTable, /*ITMHashCacheState *cacheStates, bool useSwapping,*/ int noTotalEntries,
	int *visibleEntryIDs, int *noVisibleEntries, uchar *entriesVisibleType, Matrix4f M, Vector4f projParams, Vector2i imgSize, float voxelSize);

template<typename T>
__global__ void memsetKernel_device(T *devPtr, const T val, size_t nwords);

__global__ void projectAndSplitBlocks_device(const ITMHashEntry *hashEntries, const int *visibleEntryIDs, int noVisibleEntries,
	const Matrix4f pose_M, const Vector4f intrinsics, const Vector2i imgSize, float voxelSize, RenderingBlock *renderingBlocks,
	uint *noTotalBlocks);

__global__ void fillBlocks_device(const uint *noTotalBlocks, const RenderingBlock *renderingBlocks,
	Vector2i imgSize, Vector2f *minmaxData);

template<class TVoxel, class TIndex>
__global__ void genericRaycast_device(Vector4f *out_ptsRay, const TVoxel *voxelData, const typename TIndex::IndexData *voxelIndex,
	Vector2i imgSize, Matrix4f invM, Vector4f projParams, float oneOverVoxelSize, const Vector2f *minmaxdata, float mu);

template<class TVoxel, class TIndex>
__global__ void renderICP_device(Vector4u *outRendering, Vector4f *pointsMap, Vector4f *normalsMap, const Vector4f *ptsRay,
	const TVoxel *voxelData, const typename TIndex::IndexData *voxelIndex, float voxelSize, Vector2i imgSize, Vector3f lightSource);

template<class TVoxel, class TIndex>
__global__ void renderGrey_device(Vector4u *outRendering, const Vector4f *ptsRay, const TVoxel *voxelData,
	const typename TIndex::IndexData *voxelIndex, Vector2i imgSize, Vector3f lightSource);

template<class TVoxel, class TIndex>
__global__ void renderPointCloud_device(Vector4u *outRendering, Vector4f *locations, Vector4f *colours, uint *noTotalPoints,
	const Vector4f *ptsRay, const TVoxel *voxelData, const typename TIndex::IndexData *voxelIndex, bool skipPoints,
	float voxelSize, Vector2i imgSize, Vector3f lightSource);

template<class TVoxel, class TIndex>
__global__ void renderColour_device(Vector4u *outRendering, const Vector4f *ptsRay, const TVoxel *voxelData,
	const typename TIndex::IndexData *voxelIndex, Vector2i imgSize, Vector3f lightSource);

// class implementation

template<class TVoxel, class TIndex>
ITMVisualisationEngine_CUDA<TVoxel, TIndex>::ITMVisualisationEngine_CUDA(void)
{
	ITMSafeCall(cudaMalloc((void**)&noTotalPoints_device, sizeof(uint)));
}

template<class TVoxel, class TIndex>
ITMVisualisationEngine_CUDA<TVoxel, TIndex>::~ITMVisualisationEngine_CUDA(void)
{
	ITMSafeCall(cudaFree(noTotalPoints_device));
}

template<class TVoxel>
ITMVisualisationEngine_CUDA<TVoxel, ITMVoxelBlockHash>::ITMVisualisationEngine_CUDA(void)
{
	ITMSafeCall(cudaMalloc((void**)&renderingBlockList_device, sizeof(RenderingBlock) * MAX_RENDERING_BLOCKS));
	ITMSafeCall(cudaMalloc((void**)&noTotalBlocks_device, sizeof(uint)));
	ITMSafeCall(cudaMalloc((void**)&noTotalPoints_device, sizeof(uint)));
	ITMSafeCall(cudaMalloc((void**)&noVisibleEntries_device, sizeof(uint)));
}

template<class TVoxel>
ITMVisualisationEngine_CUDA<TVoxel, ITMVoxelBlockHash>::~ITMVisualisationEngine_CUDA(void)
{
	ITMSafeCall(cudaFree(noTotalPoints_device));
	ITMSafeCall(cudaFree(noTotalBlocks_device));
	ITMSafeCall(cudaFree(renderingBlockList_device));
	ITMSafeCall(cudaFree(noVisibleEntries_device));
}

template<class TVoxel, class TIndex>
ITMRenderState* ITMVisualisationEngine_CUDA<TVoxel, TIndex>::CreateRenderState(const ITMScene<TVoxel, TIndex> *scene, const Vector2i & imgSize)
{
	return new ITMRenderState(imgSize, scene->sceneParams->viewFrustum_min, scene->sceneParams->viewFrustum_max, MEMORYDEVICE_CUDA);
}

template<class TVoxel>
ITMRenderState* ITMVisualisationEngine_CUDA<TVoxel, ITMVoxelBlockHash>::CreateRenderState(const ITMScene<TVoxel, ITMVoxelBlockHash> *scene, const Vector2i & imgSize)
{
	return new ITMRenderState_VH(ITMHashTable::noTotalEntries, imgSize, scene->sceneParams->viewFrustum_min, scene->sceneParams->viewFrustum_max, MEMORYDEVICE_CUDA);
}

template<class TVoxel, class TIndex>
void ITMVisualisationEngine_CUDA<TVoxel, TIndex>::FindVisibleBlocks(const ITMScene<TVoxel, TIndex> *scene, const ITMPose *pose,
	const ITMIntrinsics *intrinsics, ITMRenderState *renderState)
{
}

template<class TVoxel>
void ITMVisualisationEngine_CUDA<TVoxel, ITMVoxelBlockHash>::FindVisibleBlocks(const ITMScene<TVoxel, ITMVoxelBlockHash> *scene, const ITMPose *pose,
	const ITMIntrinsics *intrinsics, ITMRenderState *renderState)
{
	const ITMHashEntry *hashTable = scene->index.GetEntries();
	int noTotalEntries = scene->index.noVoxelBlocks;
	float voxelSize = scene->sceneParams->voxelSize;
	Vector2i imgSize = renderState->renderingRangeImage->noDims;

	Matrix4f M = pose->M;
	Vector4f projParams = intrinsics->projectionParamsSimple.all;

	ITMRenderState_VH *renderState_vh = (ITMRenderState_VH*)renderState;

	ITMSafeCall(cudaMemset(noVisibleEntries_device, 0, sizeof(int)));

	dim3 cudaBlockSizeAL(256, 1);
	dim3 gridSizeAL((int)ceil((float)noTotalEntries / (float)cudaBlockSizeAL.x));
	buildVisibleList_device << <gridSizeAL, cudaBlockSizeAL >> >(hashTable, /*cacheStates, scene->useSwapping,*/ noTotalEntries, 
		renderState_vh->GetVisibleEntryIDs(), noVisibleEntries_device, renderState_vh->GetEntriesVisibleType(), M, projParams, 
		imgSize, voxelSize);

	/*	if (scene->useSwapping)
			{
			reAllocateSwappedOutVoxelBlocks_device << <gridSizeAL, cudaBlockSizeAL >> >(voxelAllocationList, hashTable, noTotalEntries,
			noAllocatedVoxelEntries_device, entriesVisibleType);
			}*/

	ITMSafeCall(cudaMemcpy(&renderState_vh->noVisibleEntries, noVisibleEntries_device, sizeof(int), cudaMemcpyDeviceToHost));
}

template<class TVoxel, class TIndex>
void ITMVisualisationEngine_CUDA<TVoxel, TIndex>::CreateExpectedDepths(const ITMScene<TVoxel, TIndex> *scene, const ITMPose *pose,
	const ITMIntrinsics *intrinsics, ITMRenderState *renderState)
{
	Vector2f *minmaxData = renderState->renderingRangeImage->GetData(MEMORYDEVICE_CUDA);

	{
		dim3 blockSize(256);
		dim3 gridSize((int)ceil((float)renderState->renderingRangeImage->dataSize / (float)blockSize.x));
		Vector2f init;
		//TODO : this could be improved a bit...
		init.x = 0.2f; init.y = 3.0f;
		memsetKernel_device<Vector2f> << <gridSize, blockSize >> >(minmaxData, init, renderState->renderingRangeImage->dataSize);
	}
}

template<class TVoxel>
void ITMVisualisationEngine_CUDA<TVoxel, ITMVoxelBlockHash>::CreateExpectedDepths(const ITMScene<TVoxel, ITMVoxelBlockHash> *scene, const ITMPose *pose,
	const ITMIntrinsics *intrinsics, ITMRenderState *renderState)
{
	float voxelSize = scene->sceneParams->voxelSize;

	Vector2i imgSize = renderState->renderingRangeImage->noDims;
	Vector2f *minmaxData = renderState->renderingRangeImage->GetData(MEMORYDEVICE_CUDA);

	{
		dim3 blockSize(256);
		dim3 gridSize((int)ceil((float)renderState->renderingRangeImage->dataSize / (float)blockSize.x));
		Vector2f init;
		init.x = FAR_AWAY; init.y = VERY_CLOSE;
		memsetKernel_device<Vector2f> << <gridSize, blockSize >> >(minmaxData, init, renderState->renderingRangeImage->dataSize);
	}

	ITMRenderState_VH* renderState_vh = (ITMRenderState_VH*)renderState;

	//go through list of visible 8x8x8 blocks
	{
		const ITMHashEntry *hash_entries = scene->index.GetEntries();
		const int *visibleEntryIDs = renderState_vh->GetVisibleEntryIDs();
		int noVisibleEntries = renderState_vh->noVisibleEntries;

		dim3 blockSize(256);
		dim3 gridSize((int)ceil((float)noVisibleEntries / (float)blockSize.x));
		ITMSafeCall(cudaMemset(noTotalBlocks_device, 0, sizeof(uint)));
		projectAndSplitBlocks_device << <gridSize, blockSize >> >(hash_entries, visibleEntryIDs, noVisibleEntries, pose->M,
			intrinsics->projectionParamsSimple.all, imgSize, voxelSize, renderingBlockList_device, noTotalBlocks_device);
	}

	uint noTotalBlocks;
	ITMSafeCall(cudaMemcpy(&noTotalBlocks, noTotalBlocks_device, sizeof(uint), cudaMemcpyDeviceToHost));
	if (noTotalBlocks > (unsigned)MAX_RENDERING_BLOCKS) noTotalBlocks = MAX_RENDERING_BLOCKS;

	// go through rendering blocks
	{
		// fill minmaxData
		dim3 blockSize(16, 16);
		dim3 gridSize((unsigned int)ceil((float)noTotalBlocks / 4.0f), 4);
		fillBlocks_device << <gridSize, blockSize >> >(noTotalBlocks_device, renderingBlockList_device, imgSize, minmaxData);
	}
}

template <class TVoxel, class TIndex>
static void GenericRaycast(const ITMScene<TVoxel, TIndex> *scene, const Vector2i& imgSize, const Matrix4f& invM, Vector4f projParams, const ITMRenderState *renderState, dim3 cudaBlockSize, dim3 gridSize)
{
	float voxelSize = scene->sceneParams->voxelSize;
	float oneOverVoxelSize = 1.0f / voxelSize;

	projParams.x = 1.0f / projParams.x;
	projParams.y = 1.0f / projParams.y;

	genericRaycast_device<TVoxel, TIndex> <<<gridSize, cudaBlockSize>>>(
		renderState->raycastResult->GetData(MEMORYDEVICE_CUDA),
		scene->localVBA.GetVoxelBlocks(),
		scene->index.getIndexData(),
		imgSize,
		invM,
		projParams,
		oneOverVoxelSize,
		renderState->renderingRangeImage->GetData(MEMORYDEVICE_CUDA),
		scene->sceneParams->mu
	);
}

template<class TVoxel, class TIndex>
static void FindSurface_common(const ITMScene<TVoxel, TIndex> *scene, const ITMPose *pose, const ITMIntrinsics *intrinsics, const ITMRenderState *renderState)
{
	Vector2i imgSize = renderState->raycastResult->noDims;
	dim3 cudaBlockSize(8, 8);
	dim3 gridSize((int)ceil((float)imgSize.x / (float)cudaBlockSize.x), (int)ceil((float)imgSize.y / (float)cudaBlockSize.y));
	GenericRaycast(scene, imgSize, pose->invM, intrinsics->projectionParamsSimple.all, renderState, cudaBlockSize, gridSize);
}

template<class TVoxel, class TIndex>
static void RenderImage_common(const ITMScene<TVoxel, TIndex> *scene, const ITMPose *pose, const ITMIntrinsics *intrinsics, const ITMRenderState *renderState,
	ITMUChar4Image *outputImage, bool useColour)
{
	Vector2i imgSize = outputImage->noDims;
	dim3 cudaBlockSize(8, 8);
	dim3 gridSize((int)ceil((float)imgSize.x / (float)cudaBlockSize.x), (int)ceil((float)imgSize.y / (float)cudaBlockSize.y));
	GenericRaycast(scene, imgSize, pose->invM, intrinsics->projectionParamsSimple.all, renderState, cudaBlockSize, gridSize);

	Vector4u *outRendering = outputImage->GetData(MEMORYDEVICE_CUDA);
	Vector4f *pointsRay = renderState->raycastResult->GetData(MEMORYDEVICE_CUDA);
	Vector3f lightSource = ComputeLightSource(pose->invM);

	if (useColour && TVoxel::hasColorInformation)
		renderColour_device<TVoxel, TIndex> <<<gridSize, cudaBlockSize>>>(outRendering, pointsRay, scene->localVBA.GetVoxelBlocks(),
		scene->index.getIndexData(), imgSize, lightSource);
	else
		renderGrey_device<TVoxel, TIndex> <<<gridSize, cudaBlockSize>>>(outRendering, pointsRay, scene->localVBA.GetVoxelBlocks(),
		scene->index.getIndexData(), imgSize, lightSource);
}

template<class TVoxel, class TIndex>
static void CreatePointCloud_common(const ITMScene<TVoxel, TIndex> *scene, const ITMView *view, ITMTrackingState *trackingState, ITMRenderState *renderState,
	bool skipPoints, uint *noTotalPoints_device)
{
	Vector2i imgSize = view->rgb->noDims;
	Matrix4f invM = trackingState->pose_d->invM * view->calib->trafo_rgb_to_depth.calib;
	dim3 cudaBlockSize(16, 16);
	dim3 gridSize = getGridSize(imgSize, cudaBlockSize);
	GenericRaycast(scene, imgSize, invM, view->calib->intrinsics_rgb.projectionParamsSimple.all, renderState, cudaBlockSize, gridSize);

	ITMSafeCall(cudaMemset(noTotalPoints_device, 0, sizeof(uint)));

	Vector4f *locations = trackingState->pointCloud->locations->GetData(MEMORYDEVICE_CUDA);
	Vector4f *colours = trackingState->pointCloud->colours->GetData(MEMORYDEVICE_CUDA);
	Vector4u *outRendering = renderState->raycastImage->GetData(MEMORYDEVICE_CUDA);
	Vector4f *pointsRay = renderState->raycastResult->GetData(MEMORYDEVICE_CUDA);
	Vector3f lightSource = ComputeLightSource(invM);

	renderPointCloud_device<TVoxel, TIndex> << <gridSize, cudaBlockSize >> >(outRendering, locations, colours, noTotalPoints_device,
		pointsRay, scene->localVBA.GetVoxelBlocks(), scene->index.getIndexData(), skipPoints, scene->sceneParams->voxelSize, imgSize, lightSource);

	ITMSafeCall(cudaMemcpy(&trackingState->pointCloud->noTotalPoints, noTotalPoints_device, sizeof(uint), cudaMemcpyDeviceToHost));
}

template<class TVoxel, class TIndex>
void CreateICPMaps_common(const ITMScene<TVoxel, TIndex> *scene, const ITMView *view, ITMTrackingState *trackingState, ITMRenderState *renderState)
{
	Vector2i imgSize = view->depth->noDims;
	Matrix4f invM = trackingState->pose_d->invM;

	{
		dim3 cudaBlockSize(16, 16);
		dim3 gridSize((int)ceil((float)imgSize.x / (float)cudaBlockSize.x), (int)ceil((float)imgSize.y / (float)cudaBlockSize.y));
		GenericRaycast(scene, imgSize, invM, view->calib->intrinsics_d.projectionParamsSimple.all, renderState, cudaBlockSize, gridSize);
	}

	{
		Vector4f *pointsMap = trackingState->pointCloud->locations->GetData(MEMORYDEVICE_CUDA);
		Vector4f *normalsMap = trackingState->pointCloud->colours->GetData(MEMORYDEVICE_CUDA);
		Vector4u *outRendering = renderState->raycastImage->GetData(MEMORYDEVICE_CUDA);
		Vector4f *pointsRay = renderState->raycastResult->GetData(MEMORYDEVICE_CUDA);
		Vector3f lightSource = ComputeLightSource(invM);

		dim3 cudaBlockSize(16, 12);
		dim3 gridSize((int)ceil((float)imgSize.x / (float)cudaBlockSize.x), (int)ceil((float)imgSize.y / (float)cudaBlockSize.y));
		renderICP_device<TVoxel, TIndex> <<<gridSize, cudaBlockSize>>>(outRendering, pointsMap, normalsMap, pointsRay,
			scene->localVBA.GetVoxelBlocks(), scene->index.getIndexData(), scene->sceneParams->voxelSize, imgSize, lightSource);
	}

	ITMSafeCall(cudaThreadSynchronize());
}

template<class TVoxel, class TIndex>
void ITMVisualisationEngine_CUDA<TVoxel, TIndex>::RenderImage(const ITMScene<TVoxel, TIndex> *scene, const ITMPose *pose, const ITMIntrinsics *intrinsics,
	const ITMRenderState *renderState, ITMUChar4Image *outputImage, bool useColour)
{
	RenderImage_common(scene, pose, intrinsics, renderState, outputImage, useColour);
}

template<class TVoxel>
void ITMVisualisationEngine_CUDA<TVoxel, ITMVoxelBlockHash>::RenderImage(const ITMScene<TVoxel, ITMVoxelBlockHash> *scene, const ITMPose *pose,
	const ITMIntrinsics *intrinsics, const ITMRenderState *renderState, ITMUChar4Image *outputImage, bool useColour)
{
	RenderImage_common(scene, pose, intrinsics, renderState, outputImage, useColour);
}

template<class TVoxel, class TIndex>
void ITMVisualisationEngine_CUDA<TVoxel, TIndex>::FindSurface(const ITMScene<TVoxel, TIndex> *scene, const ITMPose *pose, const ITMIntrinsics *intrinsics,
	const ITMRenderState *renderState)
{
	FindSurface_common(scene, pose, intrinsics, renderState);
}

template<class TVoxel>
void ITMVisualisationEngine_CUDA<TVoxel, ITMVoxelBlockHash>::FindSurface(const ITMScene<TVoxel, ITMVoxelBlockHash> *scene, const ITMPose *pose,
	const ITMIntrinsics *intrinsics, const ITMRenderState *renderState)
{
	FindSurface_common(scene, pose, intrinsics, renderState);
}

template<class TVoxel, class TIndex>
void ITMVisualisationEngine_CUDA<TVoxel, TIndex>::CreatePointCloud(const ITMScene<TVoxel, TIndex> *scene, const ITMView *view, ITMTrackingState *trackingState, 
	ITMRenderState *renderState, bool skipPoints)
{
	CreatePointCloud_common(scene, view, trackingState, renderState, skipPoints, noTotalPoints_device);
}

template<class TVoxel>
void ITMVisualisationEngine_CUDA<TVoxel, ITMVoxelBlockHash>::CreatePointCloud(const ITMScene<TVoxel, ITMVoxelBlockHash> *scene, const ITMView *view,
	ITMTrackingState *trackingState, ITMRenderState *renderState, bool skipPoints)
{
	CreatePointCloud_common(scene, view, trackingState, renderState, skipPoints, noTotalPoints_device);
}

template<class TVoxel, class TIndex>
void ITMVisualisationEngine_CUDA<TVoxel, TIndex>::CreateICPMaps(const ITMScene<TVoxel, TIndex> *scene, const ITMView *view, ITMTrackingState *trackingState,
	ITMRenderState *renderState)
{
	CreateICPMaps_common(scene, view, trackingState, renderState);
}

template<class TVoxel>
void ITMVisualisationEngine_CUDA<TVoxel, ITMVoxelBlockHash>::CreateICPMaps(const ITMScene<TVoxel, ITMVoxelBlockHash> *scene, const ITMView *view,
	ITMTrackingState *trackingState, ITMRenderState *renderState)
{
	CreateICPMaps_common(scene, view, trackingState, renderState);
}

//device implementations

__global__ void buildVisibleList_device(const ITMHashEntry *hashTable, /*ITMHashCacheState *cacheStates, bool useSwapping,*/ int noTotalEntries,
	int *visibleEntryIDs, int *noVisibleEntries, uchar *entriesVisibleType, Matrix4f M, Vector4f projParams, Vector2i imgSize, float voxelSize)
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

		bool isVisible, isVisibleEnlarged;
		checkBlockVisibility<false>(isVisible, isVisibleEnlarged, hashEntry.pos, M, projParams, voxelSize, imgSize);

		hashVisibleType = isVisible;
	}

	if (hashVisibleType > 0) shouldPrefix = true;

	__syncthreads();

	if (shouldPrefix)
	{
		int offset = computePrefixSum_device<int>(hashVisibleType > 0, noVisibleEntries, blockDim.x * blockDim.y, threadIdx.x);
		if (offset != -1) visibleEntryIDs[offset] = targetIdx;
	}
}

template<typename T> __global__ void memsetKernel_device(T *devPtr, const T val, size_t nwords)
{
	size_t offset = threadIdx.x + blockDim.x * blockIdx.x;
	if (offset >= nwords) return;
	devPtr[offset] = val;
}

__global__ void projectAndSplitBlocks_device(const ITMHashEntry *hashEntries, const int *visibleEntryIDs, int noVisibleEntries,
	const Matrix4f pose_M, const Vector4f intrinsics, const Vector2i imgSize, float voxelSize, RenderingBlock *renderingBlocks,
	uint *noTotalBlocks)
{
	int in_offset = threadIdx.x + blockDim.x * blockIdx.x;

	const ITMHashEntry & blockData(hashEntries[visibleEntryIDs[in_offset]]);

	Vector2i upperLeft, lowerRight;
	Vector2f zRange;
	bool validProjection = false;
	if (in_offset < noVisibleEntries) if (blockData.ptr >= 0)
		validProjection = ProjectSingleBlock(blockData.pos, pose_M, intrinsics, imgSize, voxelSize, upperLeft, lowerRight, zRange);

	Vector2i requiredRenderingBlocks(ceilf((float)(lowerRight.x - upperLeft.x + 1) / renderingBlockSizeX),
		ceilf((float)(lowerRight.y - upperLeft.y + 1) / renderingBlockSizeY));

	size_t requiredNumBlocks = requiredRenderingBlocks.x * requiredRenderingBlocks.y;
	if (!validProjection) requiredNumBlocks = 0;

	int out_offset = computePrefixSum_device<uint>(requiredNumBlocks, noTotalBlocks, blockDim.x, threadIdx.x);
	if (!validProjection) return;
	if ((out_offset == -1) || (out_offset + requiredNumBlocks > MAX_RENDERING_BLOCKS)) return;

	CreateRenderingBlocks(renderingBlocks, out_offset, upperLeft, lowerRight, zRange);
}

__global__ void fillBlocks_device(const uint *noTotalBlocks, const RenderingBlock *renderingBlocks,
	Vector2i imgSize, Vector2f *minmaxData)
{
	int x = threadIdx.x;
	int y = threadIdx.y;
	int block = blockIdx.x * 4 + blockIdx.y;
	if (block >= *noTotalBlocks) return;

	const RenderingBlock & b(renderingBlocks[block]);
	int xpos = b.upperLeft.x + x;
	if (xpos > b.lowerRight.x) return;
	int ypos = b.upperLeft.y + y;
	if (ypos > b.lowerRight.y) return;

	Vector2f & pixel(minmaxData[xpos + ypos*imgSize.x]);
	atomicMin(&pixel.x, b.zRange.x); atomicMax(&pixel.y, b.zRange.y);
}

template<class TVoxel, class TIndex>
__global__ void genericRaycast_device(Vector4f *out_ptsRay, const TVoxel *voxelData, const typename TIndex::IndexData *voxelIndex,
	Vector2i imgSize, Matrix4f invM, Vector4f projParams, float oneOverVoxelSize, const Vector2f *minmaxdata, float mu)
{
	int x = (threadIdx.x + blockIdx.x * blockDim.x), y = (threadIdx.y + blockIdx.y * blockDim.y);

	if (x >= imgSize.x || y >= imgSize.y) return;

	int locId = x + y * imgSize.x;
	int locId2 = (int)floor((float)x / minmaximg_subsample) + (int)floor((float)y / minmaximg_subsample) * imgSize.x;

	castRay<TVoxel, TIndex>(out_ptsRay[locId], x, y, voxelData, voxelIndex, invM, projParams, oneOverVoxelSize, mu, minmaxdata[locId2]);
}

template<class TVoxel, class TIndex>
__global__ void renderICP_device(Vector4u *outRendering, Vector4f *pointsMap, Vector4f *normalsMap, const Vector4f *ptsRay,
	const TVoxel *voxelData, const typename TIndex::IndexData *voxelIndex, float voxelSize, Vector2i imgSize, Vector3f lightSource)
{
	int x = (threadIdx.x + blockIdx.x * blockDim.x), y = (threadIdx.y + blockIdx.y * blockDim.y);

	if (x >= imgSize.x || y >= imgSize.y) return;

	int locId = x + y * imgSize.x;

	Vector4f ptRay = ptsRay[locId];

	processPixelICP<TVoxel, TIndex>(outRendering[locId], pointsMap[locId], normalsMap[locId], ptRay.toVector3(), ptRay.w > 0, voxelData,
		voxelIndex, voxelSize, lightSource);
}

template<class TVoxel, class TIndex>
__global__ void renderGrey_device(Vector4u *outRendering, const Vector4f *ptsRay, const TVoxel *voxelData,
	const typename TIndex::IndexData *voxelIndex, Vector2i imgSize, Vector3f lightSource)
{
	int x = (threadIdx.x + blockIdx.x * blockDim.x), y = (threadIdx.y + blockIdx.y * blockDim.y);

	if (x >= imgSize.x || y >= imgSize.y) return;

	int locId = x + y * imgSize.x;

	Vector4f ptRay = ptsRay[locId];

	processPixelGrey<TVoxel, TIndex>(outRendering[locId], ptRay.toVector3(), ptRay.w > 0, voxelData, voxelIndex, lightSource);
}

template<class TVoxel, class TIndex>
__global__ void renderPointCloud_device(Vector4u *outRendering, Vector4f *locations, Vector4f *colours, uint *noTotalPoints,
	const Vector4f *ptsRay, const TVoxel *voxelData, const typename TIndex::IndexData *voxelIndex, bool skipPoints,
	float voxelSize, Vector2i imgSize, Vector3f lightSource)
{
	__shared__ bool shouldPrefix;
	shouldPrefix = false;
	__syncthreads();

	bool foundPoint = false; Vector3f point(0.0f);

	int x = (threadIdx.x + blockIdx.x * blockDim.x), y = (threadIdx.y + blockIdx.y * blockDim.y);

	if (x < imgSize.x && y < imgSize.y)
	{
		int locId = x + y * imgSize.x;
		Vector3f outNormal; float angle; Vector4f pointRay;

		pointRay = ptsRay[locId];
		point = pointRay.toVector3();
		foundPoint = pointRay.w > 0;

		computeNormalAndAngle<TVoxel, TIndex>(foundPoint, point, voxelData, voxelIndex, lightSource, outNormal, angle);

		if (foundPoint) drawPixelGrey(outRendering[locId], angle);
		else outRendering[locId] = Vector4u((uchar)0);

		if (skipPoints && ((x % 2 == 0) || (y % 2 == 0))) foundPoint = false;

		if (foundPoint) shouldPrefix = true;
	}

	__syncthreads();

	if (shouldPrefix)
	{
		int offset = computePrefixSum_device<uint>(foundPoint, noTotalPoints, blockDim.x * blockDim.y, threadIdx.x + threadIdx.y * blockDim.x);

		if (offset != -1)
		{
			Vector4f tmp;
			tmp = VoxelColorReader<TVoxel::hasColorInformation, TVoxel, TIndex>::interpolate(voxelData, voxelIndex, point);
			if (tmp.w > 0.0f) { tmp.x /= tmp.w; tmp.y /= tmp.w; tmp.z /= tmp.w; tmp.w = 1.0f; }
			colours[offset] = tmp;

			Vector4f pt_ray_out;
			pt_ray_out.x = point.x * voxelSize; pt_ray_out.y = point.y * voxelSize;
			pt_ray_out.z = point.z * voxelSize; pt_ray_out.w = 1.0f;
			locations[offset] = pt_ray_out;
		}
	}
}

template<class TVoxel, class TIndex>
__global__ void renderColour_device(Vector4u *outRendering, const Vector4f *ptsRay, const TVoxel *voxelData,
	const typename TIndex::IndexData *voxelIndex, Vector2i imgSize, Vector3f lightSource)
{
	int x = (threadIdx.x + blockIdx.x * blockDim.x), y = (threadIdx.y + blockIdx.y * blockDim.y);

	if (x >= imgSize.x || y >= imgSize.y) return;

	int locId = x + y * imgSize.x;

	Vector4f ptRay = ptsRay[locId];

	processPixelColour<TVoxel, TIndex>(outRendering[locId], ptRay.toVector3(), ptRay.w > 0, voxelData, voxelIndex, lightSource);
}

template class ITMLib::Engine::ITMVisualisationEngine_CUDA < ITMVoxel, ITMVoxelIndex > ;
