// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM

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

__global__ void projectAndSplitBlocks_device(const ITMHashEntry *hashEntries, const int *visibleEntryIDs, int noVisibleEntries,
	const Matrix4f pose_M, const Vector4f intrinsics, const Vector2i imgSize, float voxelSize, RenderingBlock *renderingBlocks,
	uint *noTotalBlocks);

__global__ void fillBlocks_device(const uint *noTotalBlocks, const RenderingBlock *renderingBlocks,
	Vector2i imgSize, Vector2f *minmaxData);

__global__ void findMissingPoints_device(int *fwdProjMissingPoints, uint *noMissingPoints, const Vector2f *minmaximg,
	Vector4f *forwardProjection, float *currentDepth, Vector2i imgSize);

template<class TVoxel, class TIndex>
__global__ void genericRaycast_device(Vector4f *out_ptsRay, const TVoxel *voxelData, const typename TIndex::IndexData *voxelIndex,
	Vector2i imgSize, Matrix4f invM, Vector4f projParams, float oneOverVoxelSize, const Vector2f *minmaxdata, float mu);

template<class TVoxel, class TIndex>
__global__ void genericRaycastMissingPoints_device(Vector4f *forwardProjection, const TVoxel *voxelData, const typename TIndex::IndexData *voxelIndex,
	Vector2i imgSize, Matrix4f invM, Vector4f invProjParams, float oneOverVoxelSize, int *fwdProjMissingPoints, int noMissingPoints,
	const Vector2f *minmaximg, float mu);

__global__ void forwardProject_device(Vector4f *forwardProjection, const Vector4f *pointsRay, Vector2i imgSize, Matrix4f M,
	Vector4f projParams, float voxelSize);

__global__ void renderICP_device(Vector4u *outRendering, Vector4f *pointsMap, Vector4f *normalsMap, const Vector4f *ptsRay,
	float voxelSize, Vector2i imgSize, Vector3f lightSource);

__global__ void renderForward_device(Vector4u *outRendering, const Vector4f *pointsRay, float voxelSize, Vector2i imgSize, Vector3f lightSource);

template<class TVoxel, class TIndex>
__global__ void renderGrey_device(Vector4u *outRendering, const Vector4f *ptsRay, const TVoxel *voxelData,
	const typename TIndex::IndexData *voxelIndex, Vector2i imgSize, Vector3f lightSource);

template<class TVoxel, class TIndex>
__global__ void renderColourFromNormal_device(Vector4u *outRendering, const Vector4f *ptsRay, const TVoxel *voxelData,
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
ITMVisualisationEngine_CUDA<TVoxel, TIndex>::ITMVisualisationEngine_CUDA(ITMScene<TVoxel, TIndex> *scene)
	: ITMVisualisationEngine<TVoxel, TIndex>(scene)
{
	ITMSafeCall(cudaMalloc((void**)&noTotalPoints_device, sizeof(uint)));
}

template<class TVoxel, class TIndex>
ITMVisualisationEngine_CUDA<TVoxel, TIndex>::~ITMVisualisationEngine_CUDA(void)
{
	ITMSafeCall(cudaFree(noTotalPoints_device));
}

template<class TVoxel>
ITMVisualisationEngine_CUDA<TVoxel, ITMVoxelBlockHash>::ITMVisualisationEngine_CUDA(ITMScene<TVoxel, ITMVoxelBlockHash> *scene)
	: ITMVisualisationEngine<TVoxel, ITMVoxelBlockHash>(scene)
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
ITMRenderState* ITMVisualisationEngine_CUDA<TVoxel, TIndex>::CreateRenderState(const Vector2i & imgSize) const
{
	return new ITMRenderState(
		imgSize, this->scene->sceneParams->viewFrustum_min, this->scene->sceneParams->viewFrustum_max, MEMORYDEVICE_CUDA
	);
}

template<class TVoxel>
ITMRenderState_VH* ITMVisualisationEngine_CUDA<TVoxel, ITMVoxelBlockHash>::CreateRenderState(const Vector2i & imgSize) const
{
	return new ITMRenderState_VH(
		ITMVoxelBlockHash::noTotalEntries, imgSize, this->scene->sceneParams->viewFrustum_min, this->scene->sceneParams->viewFrustum_max, MEMORYDEVICE_CUDA
	);
}

template<class TVoxel, class TIndex>
void ITMVisualisationEngine_CUDA<TVoxel, TIndex>::FindVisibleBlocks(const ITMPose *pose, const ITMIntrinsics *intrinsics, ITMRenderState *renderState) const
{
}

template<class TVoxel>
void ITMVisualisationEngine_CUDA<TVoxel, ITMVoxelBlockHash>::FindVisibleBlocks(const ITMPose *pose, const ITMIntrinsics *intrinsics, ITMRenderState *renderState) const
{
	const ITMHashEntry *hashTable = this->scene->index.GetEntries();
	int noTotalEntries = this->scene->index.noTotalEntries;
	float voxelSize = this->scene->sceneParams->voxelSize;
	Vector2i imgSize = renderState->renderingRangeImage->noDims;

	Matrix4f M = pose->GetM();
	Vector4f projParams = intrinsics->projectionParamsSimple.all;

	ITMRenderState_VH *renderState_vh = (ITMRenderState_VH*)renderState;

	ITMSafeCall(cudaMemset(noVisibleEntries_device, 0, sizeof(int)));

	dim3 cudaBlockSizeAL(256, 1);
	dim3 gridSizeAL((int)ceil((float)noTotalEntries / (float)cudaBlockSizeAL.x));
	buildVisibleList_device << <gridSizeAL, cudaBlockSizeAL >> >(hashTable, /*cacheStates, this->scene->useSwapping,*/ noTotalEntries,
		renderState_vh->GetVisibleEntryIDs(), noVisibleEntries_device, renderState_vh->GetEntriesVisibleType(), M, projParams, 
		imgSize, voxelSize);

	/*	if (this->scene->useSwapping)
			{
			reAllocateSwappedOutVoxelBlocks_device << <gridSizeAL, cudaBlockSizeAL >> >(voxelAllocationList, hashTable, noTotalEntries,
			noAllocatedVoxelEntries_device, entriesVisibleType);
			}*/

	ITMSafeCall(cudaMemcpy(&renderState_vh->noVisibleEntries, noVisibleEntries_device, sizeof(int), cudaMemcpyDeviceToHost));
}

template<class TVoxel, class TIndex>
void ITMVisualisationEngine_CUDA<TVoxel, TIndex>::CreateExpectedDepths(const ITMPose *pose,	const ITMIntrinsics *intrinsics, ITMRenderState *renderState) const
{
	Vector2f *minmaxData = renderState->renderingRangeImage->GetData(MEMORYDEVICE_CUDA);

	Vector2f init;
	//TODO : this could be improved a bit...
	init.x = 0.2f; init.y = 3.0f;
	memsetKernel<Vector2f>(minmaxData, init, renderState->renderingRangeImage->dataSize);
}

template<class TVoxel>
void ITMVisualisationEngine_CUDA<TVoxel, ITMVoxelBlockHash>::CreateExpectedDepths(const ITMPose *pose, const ITMIntrinsics *intrinsics, 
	ITMRenderState *renderState) const
{
	float voxelSize = this->scene->sceneParams->voxelSize;

	Vector2i imgSize = renderState->renderingRangeImage->noDims;
	Vector2f *minmaxData = renderState->renderingRangeImage->GetData(MEMORYDEVICE_CUDA);

	Vector2f init;
	init.x = FAR_AWAY; init.y = VERY_CLOSE;
	memsetKernel<Vector2f>(minmaxData, init, renderState->renderingRangeImage->dataSize);

	ITMRenderState_VH* renderState_vh = (ITMRenderState_VH*)renderState;

	//go through list of visible 8x8x8 blocks
	{
		const ITMHashEntry *hash_entries = this->scene->index.GetEntries();
		const int *visibleEntryIDs = renderState_vh->GetVisibleEntryIDs();
		int noVisibleEntries = renderState_vh->noVisibleEntries;

		dim3 blockSize(256);
		dim3 gridSize((int)ceil((float)noVisibleEntries / (float)blockSize.x));
		ITMSafeCall(cudaMemset(noTotalBlocks_device, 0, sizeof(uint)));
		projectAndSplitBlocks_device << <gridSize, blockSize >> >(hash_entries, visibleEntryIDs, noVisibleEntries, pose->GetM(),
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
static void GenericRaycast(const ITMScene<TVoxel, TIndex> *scene, const Vector2i& imgSize, const Matrix4f& invM, Vector4f projParams, const ITMRenderState *renderState)
{
	float voxelSize = scene->sceneParams->voxelSize;
	float oneOverVoxelSize = 1.0f / voxelSize;

	projParams.x = 1.0f / projParams.x;
	projParams.y = 1.0f / projParams.y;

	dim3 cudaBlockSize(16, 12);
	dim3 gridSize((int)ceil((float)imgSize.x / (float)cudaBlockSize.x), (int)ceil((float)imgSize.y / (float)cudaBlockSize.y));
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
static void RenderImage_common(const ITMScene<TVoxel, TIndex> *scene, const ITMPose *pose, const ITMIntrinsics *intrinsics, const ITMRenderState *renderState,
	ITMUChar4Image *outputImage, IITMVisualisationEngine::RenderImageType type)
{
	Vector2i imgSize = outputImage->noDims;
	Matrix4f invM = pose->GetInvM();

	GenericRaycast(scene, imgSize, invM, intrinsics->projectionParamsSimple.all, renderState);

	Vector3f lightSource = -Vector3f(invM.getColumn(2));
	Vector4u *outRendering = outputImage->GetData(MEMORYDEVICE_CUDA);
	Vector4f *pointsRay = renderState->raycastResult->GetData(MEMORYDEVICE_CUDA);

	dim3 cudaBlockSize(8, 8);
	dim3 gridSize((int)ceil((float)imgSize.x / (float)cudaBlockSize.x), (int)ceil((float)imgSize.y / (float)cudaBlockSize.y));

	if ((type == IITMVisualisationEngine::RENDER_COLOUR_FROM_VOLUME)&&
	    (!TVoxel::hasColorInformation)) type = IITMVisualisationEngine::RENDER_SHADED_GREYSCALE;

	switch (type) {
	case IITMVisualisationEngine::RENDER_COLOUR_FROM_VOLUME:
		renderColour_device<TVoxel, TIndex> <<<gridSize, cudaBlockSize>>>(outRendering, pointsRay, scene->localVBA.GetVoxelBlocks(),
			scene->index.getIndexData(), imgSize, lightSource);
		break;
	case IITMVisualisationEngine::RENDER_COLOUR_FROM_NORMAL:
		renderColourFromNormal_device<TVoxel, TIndex> <<<gridSize, cudaBlockSize>>>(outRendering, pointsRay, scene->localVBA.GetVoxelBlocks(),
			scene->index.getIndexData(), imgSize, lightSource);
		break;
	case IITMVisualisationEngine::RENDER_SHADED_GREYSCALE:
	default:
		renderGrey_device<TVoxel, TIndex> <<<gridSize, cudaBlockSize>>>(outRendering, pointsRay, scene->localVBA.GetVoxelBlocks(),
			scene->index.getIndexData(), imgSize, lightSource);
		break;
	}
}

template<class TVoxel, class TIndex>
static void CreatePointCloud_common(const ITMScene<TVoxel, TIndex> *scene, const ITMView *view, ITMTrackingState *trackingState, ITMRenderState *renderState,
	bool skipPoints, uint *noTotalPoints_device)
{
	Vector2i imgSize = renderState->raycastResult->noDims;
	Matrix4f invM = trackingState->pose_d->GetInvM() * view->calib->trafo_rgb_to_depth.calib;

	GenericRaycast(scene, imgSize, invM, view->calib->intrinsics_rgb.projectionParamsSimple.all, renderState);
	trackingState->pose_pointCloud->SetFrom(trackingState->pose_d);

	ITMSafeCall(cudaMemsetAsync(noTotalPoints_device, 0, sizeof(uint)));

	Vector3f lightSource = -Vector3f(invM.getColumn(2));
	Vector4f *locations = trackingState->pointCloud->locations->GetData(MEMORYDEVICE_CUDA);
	Vector4f *colours = trackingState->pointCloud->colours->GetData(MEMORYDEVICE_CUDA);
	Vector4u *outRendering = renderState->raycastImage->GetData(MEMORYDEVICE_CUDA);
	Vector4f *pointsRay = renderState->raycastResult->GetData(MEMORYDEVICE_CUDA);

	dim3 cudaBlockSize(16, 16);
	dim3 gridSize = getGridSize(imgSize, cudaBlockSize);
	renderPointCloud_device<TVoxel, TIndex> << <gridSize, cudaBlockSize >> >(outRendering, locations, colours, noTotalPoints_device,
		pointsRay, scene->localVBA.GetVoxelBlocks(), scene->index.getIndexData(), skipPoints, scene->sceneParams->voxelSize, imgSize, lightSource);

	ITMSafeCall(cudaMemcpy(&trackingState->pointCloud->noTotalPoints, noTotalPoints_device, sizeof(uint), cudaMemcpyDeviceToHost));
}

template<class TVoxel, class TIndex>
void CreateICPMaps_common(const ITMScene<TVoxel, TIndex> *scene, const ITMView *view, ITMTrackingState *trackingState, ITMRenderState *renderState)
{
	Vector2i imgSize = renderState->raycastResult->noDims;
	Matrix4f invM = trackingState->pose_d->GetInvM();

	GenericRaycast(scene, imgSize, invM, view->calib->intrinsics_d.projectionParamsSimple.all, renderState);
	trackingState->pose_pointCloud->SetFrom(trackingState->pose_d);

	Vector4f *pointsMap = trackingState->pointCloud->locations->GetData(MEMORYDEVICE_CUDA);
	Vector4f *normalsMap = trackingState->pointCloud->colours->GetData(MEMORYDEVICE_CUDA);
	Vector4u *outRendering = renderState->raycastImage->GetData(MEMORYDEVICE_CUDA);
	Vector4f *pointsRay = renderState->raycastResult->GetData(MEMORYDEVICE_CUDA);
	Vector3f lightSource = -Vector3f(invM.getColumn(2));

	dim3 cudaBlockSize(16, 12);
	dim3 gridSize((int)ceil((float)imgSize.x / (float)cudaBlockSize.x), (int)ceil((float)imgSize.y / (float)cudaBlockSize.y));
	renderICP_device<<<gridSize, cudaBlockSize>>>(outRendering, pointsMap, normalsMap, pointsRay,
		scene->sceneParams->voxelSize, imgSize, lightSource);
}

template<class TVoxel, class TIndex>
static void ForwardRender_common(const ITMScene<TVoxel, TIndex> *scene, const ITMView *view, ITMTrackingState *trackingState, ITMRenderState *renderState, 
	uint *noTotalPoints_device)
{
	Vector2i imgSize = renderState->raycastResult->noDims;
	Matrix4f M = trackingState->pose_d->GetM();
	Matrix4f invM = trackingState->pose_d->GetInvM();
	Vector4f projParams = view->calib->intrinsics_d.projectionParamsSimple.all;
	Vector4f invProjParams = view->calib->intrinsics_d.projectionParamsSimple.all;
	invProjParams.x = 1.0f / invProjParams.x;
	invProjParams.y = 1.0f / invProjParams.y;

	Vector3f lightSource = -Vector3f(invM.getColumn(2));
	const Vector4f *pointsRay = renderState->raycastResult->GetData(MEMORYDEVICE_CUDA);
	float *currentDepth = view->depth->GetData(MEMORYDEVICE_CUDA);
	Vector4f *forwardProjection = renderState->forwardProjection->GetData(MEMORYDEVICE_CUDA);
	int *fwdProjMissingPoints = renderState->fwdProjMissingPoints->GetData(MEMORYDEVICE_CUDA);
	Vector4u *outRendering = renderState->raycastImage->GetData(MEMORYDEVICE_CUDA);
	const Vector2f *minmaximg = renderState->renderingRangeImage->GetData(MEMORYDEVICE_CUDA);
	float oneOverVoxelSize = 1.0f / scene->sceneParams->voxelSize;
	float voxelSize = scene->sceneParams->voxelSize;
	const TVoxel *voxelData = scene->localVBA.GetVoxelBlocks();
	const typename TIndex::IndexData *voxelIndex = scene->index.getIndexData();

	renderState->forwardProjection->Clear();

	dim3 blockSize, gridSize;

	{ // forward projection
		blockSize = dim3(16, 16);
		gridSize = dim3((int)ceil((float)imgSize.x / (float)blockSize.x), (int)ceil((float)imgSize.y / (float)blockSize.y));

		forwardProject_device << <gridSize, blockSize >> >(forwardProjection, pointsRay, imgSize, M, projParams, voxelSize);
	}

	ITMSafeCall(cudaMemset(noTotalPoints_device, 0, sizeof(uint)));

	{ // find missing points
		blockSize = dim3(16, 16);
		gridSize = dim3((int)ceil((float)imgSize.x / (float)blockSize.x), (int)ceil((float)imgSize.y / (float)blockSize.y));

		findMissingPoints_device << <gridSize, blockSize >> >(fwdProjMissingPoints, noTotalPoints_device, minmaximg, 
			forwardProjection, currentDepth, imgSize);
	}

	ITMSafeCall(cudaMemcpy(&renderState->noFwdProjMissingPoints, noTotalPoints_device, sizeof(uint), cudaMemcpyDeviceToHost));

	{ // render missing points
		blockSize = dim3(256);
		gridSize = dim3((int)ceil((float)renderState->noFwdProjMissingPoints / blockSize.x));

		genericRaycastMissingPoints_device<TVoxel, TIndex> << <gridSize, blockSize >> >(forwardProjection, voxelData, voxelIndex, imgSize, invM,
			invProjParams, oneOverVoxelSize, fwdProjMissingPoints, renderState->noFwdProjMissingPoints, minmaximg, scene->sceneParams->mu);
	}

	{
		blockSize = dim3(16, 16);
		gridSize = dim3((int)ceil((float)imgSize.x / (float)blockSize.x), (int)ceil((float)imgSize.y / (float)blockSize.y));

		renderForward_device << <gridSize, blockSize >> >(outRendering, forwardProjection, voxelSize, imgSize, lightSource);
	}
}

template<class TVoxel, class TIndex>
void ITMVisualisationEngine_CUDA<TVoxel, TIndex>::RenderImage(const ITMPose *pose, const ITMIntrinsics *intrinsics, const ITMRenderState *renderState, 
	ITMUChar4Image *outputImage, IITMVisualisationEngine::RenderImageType type) const
{
	RenderImage_common(this->scene, pose, intrinsics, renderState, outputImage, type);
}

template<class TVoxel>
void ITMVisualisationEngine_CUDA<TVoxel, ITMVoxelBlockHash>::RenderImage(const ITMPose *pose, const ITMIntrinsics *intrinsics, 
	const ITMRenderState *renderState, ITMUChar4Image *outputImage, IITMVisualisationEngine::RenderImageType type) const
{
	RenderImage_common(this->scene, pose, intrinsics, renderState, outputImage, type);
}

template<class TVoxel, class TIndex>
void ITMVisualisationEngine_CUDA<TVoxel, TIndex>::FindSurface(const ITMPose *pose, const ITMIntrinsics *intrinsics, const ITMRenderState *renderState) const
{
	GenericRaycast(this->scene, renderState->raycastResult->noDims, pose->GetInvM(), intrinsics->projectionParamsSimple.all, renderState);
}

template<class TVoxel>
void ITMVisualisationEngine_CUDA<TVoxel, ITMVoxelBlockHash>::FindSurface(const ITMPose *pose, const ITMIntrinsics *intrinsics,
	const ITMRenderState *renderState) const
{
	GenericRaycast(this->scene, renderState->raycastResult->noDims, pose->GetInvM(), intrinsics->projectionParamsSimple.all, renderState);
}

template<class TVoxel, class TIndex>
void ITMVisualisationEngine_CUDA<TVoxel, TIndex>::CreatePointCloud(const ITMView *view, ITMTrackingState *trackingState, ITMRenderState *renderState, 
	bool skipPoints) const
{
	CreatePointCloud_common(this->scene, view, trackingState, renderState, skipPoints, noTotalPoints_device);
}

template<class TVoxel>
void ITMVisualisationEngine_CUDA<TVoxel, ITMVoxelBlockHash>::CreatePointCloud(const ITMView *view, ITMTrackingState *trackingState, 
	ITMRenderState *renderState, bool skipPoints) const
{
	CreatePointCloud_common(this->scene, view, trackingState, renderState, skipPoints, noTotalPoints_device);
}

template<class TVoxel, class TIndex>
void ITMVisualisationEngine_CUDA<TVoxel, TIndex>::CreateICPMaps(const ITMView *view, ITMTrackingState *trackingState, 
	ITMRenderState *renderState) const
{
	CreateICPMaps_common(this->scene, view, trackingState, renderState);
}

template<class TVoxel>
void ITMVisualisationEngine_CUDA<TVoxel, ITMVoxelBlockHash>::CreateICPMaps(const ITMView *view, ITMTrackingState *trackingState, 
	ITMRenderState *renderState) const
{
	CreateICPMaps_common(this->scene, view, trackingState, renderState);
}

template<class TVoxel, class TIndex>
void ITMVisualisationEngine_CUDA<TVoxel, TIndex>::ForwardRender(const ITMView *view, ITMTrackingState *trackingState, 
	ITMRenderState *renderState) const
{
	ForwardRender_common(this->scene, view, trackingState, renderState, this->noTotalPoints_device);
}

template<class TVoxel>
void ITMVisualisationEngine_CUDA<TVoxel, ITMVoxelBlockHash>::ForwardRender(const ITMView *view, ITMTrackingState *trackingState, 
	ITMRenderState *renderState) const
{
	ForwardRender_common(this->scene, view, trackingState, renderState, this->noTotalPoints_device);
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

__global__ void findMissingPoints_device(int *fwdProjMissingPoints, uint *noMissingPoints, const Vector2f *minmaximg,
	Vector4f *forwardProjection, float *currentDepth, Vector2i imgSize)
{
	int x = (threadIdx.x + blockIdx.x * blockDim.x), y = (threadIdx.y + blockIdx.y * blockDim.y);

	if (x >= imgSize.x || y >= imgSize.y) return;

	int locId = x + y * imgSize.x;
	int locId2 = (int)floor((float)x / minmaximg_subsample) + (int)floor((float)y / minmaximg_subsample) * imgSize.x;

	Vector4f fwdPoint = forwardProjection[locId];
	Vector2f minmaxval = minmaximg[locId2];
	float depth = currentDepth[locId];

	bool hasPoint = false;

	__shared__ bool shouldPrefix;
	shouldPrefix = false;
	__syncthreads();

	if ((fwdPoint.w <= 0) && ((fwdPoint.x == 0 && fwdPoint.y == 0 && fwdPoint.z == 0) || (depth > 0)) && (minmaxval.x < minmaxval.y))
	//if ((fwdPoint.w <= 0) && (minmaxval.x < minmaxval.y))
	{ shouldPrefix = true; hasPoint = true; }

	__syncthreads();

	if (shouldPrefix)
	{
		int offset = computePrefixSum_device(hasPoint, noMissingPoints, blockDim.x * blockDim.y, threadIdx.x + threadIdx.y * blockDim.x);
		if (offset != -1) fwdProjMissingPoints[offset] = locId;
	}
}

template<class TVoxel, class TIndex>
__global__ void genericRaycast_device(Vector4f *out_ptsRay, const TVoxel *voxelData, const typename TIndex::IndexData *voxelIndex,
	Vector2i imgSize, Matrix4f invM, Vector4f invProjParams, float oneOverVoxelSize, const Vector2f *minmaximg, float mu)
{
	int x = (threadIdx.x + blockIdx.x * blockDim.x), y = (threadIdx.y + blockIdx.y * blockDim.y);

	if (x >= imgSize.x || y >= imgSize.y) return;

	int locId = x + y * imgSize.x;
	int locId2 = (int)floor((float)x / minmaximg_subsample) + (int)floor((float)y / minmaximg_subsample) * imgSize.x;

	castRay<TVoxel, TIndex>(out_ptsRay[locId], x, y, voxelData, voxelIndex, invM, invProjParams, oneOverVoxelSize, mu, minmaximg[locId2]);
}

template<class TVoxel, class TIndex>
__global__ void genericRaycastMissingPoints_device(Vector4f *forwardProjection, const TVoxel *voxelData, const typename TIndex::IndexData *voxelIndex,
	Vector2i imgSize, Matrix4f invM, Vector4f invProjParams, float oneOverVoxelSize, int *fwdProjMissingPoints, int noMissingPoints,
	const Vector2f *minmaximg, float mu)
{
	int pointId = threadIdx.x + blockIdx.x * blockDim.x;

	if (pointId >= noMissingPoints) return;

	int locId = fwdProjMissingPoints[pointId];
	int y = locId / imgSize.x, x = locId - y*imgSize.x;
	int locId2 = (int)floor((float)x / minmaximg_subsample) + (int)floor((float)y / minmaximg_subsample) * imgSize.x;

	castRay<TVoxel, TIndex>(forwardProjection[locId], x, y, voxelData, voxelIndex, invM, invProjParams, oneOverVoxelSize, mu, minmaximg[locId2]);
}

__global__ void forwardProject_device(Vector4f *forwardProjection, const Vector4f *pointsRay, Vector2i imgSize, Matrix4f M,
	Vector4f projParams, float voxelSize)
{
	int x = (threadIdx.x + blockIdx.x * blockDim.x), y = (threadIdx.y + blockIdx.y * blockDim.y);

	if (x >= imgSize.x || y >= imgSize.y) return;

	int locId = x + y * imgSize.x;
	Vector4f pixel = pointsRay[locId];

	int locId_new = forwardProjectPixel(pixel * voxelSize, M, projParams, imgSize);
	if (locId_new >= 0) forwardProjection[locId_new] = pixel;
}

__global__ void renderICP_device(Vector4u *outRendering, Vector4f *pointsMap, Vector4f *normalsMap, const Vector4f *pointsRay,
	float voxelSize, Vector2i imgSize, Vector3f lightSource)
{
	int x = (threadIdx.x + blockIdx.x * blockDim.x), y = (threadIdx.y + blockIdx.y * blockDim.y);

	if (x >= imgSize.x || y >= imgSize.y) return;

	processPixelICP<true>(outRendering, pointsMap, normalsMap, pointsRay, imgSize, x, y, voxelSize, lightSource);
}

__global__ void renderForward_device(Vector4u *outRendering, const Vector4f *pointsRay, float voxelSize, Vector2i imgSize, Vector3f lightSource)
{
	int x = (threadIdx.x + blockIdx.x * blockDim.x), y = (threadIdx.y + blockIdx.y * blockDim.y);

	if (x >= imgSize.x || y >= imgSize.y) return;

	processPixelForwardRender<true>(outRendering, pointsRay, imgSize, x, y, voxelSize, lightSource);
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
__global__ void renderColourFromNormal_device(Vector4u *outRendering, const Vector4f *ptsRay, const TVoxel *voxelData,
	const typename TIndex::IndexData *voxelIndex, Vector2i imgSize, Vector3f lightSource)
{
	int x = (threadIdx.x + blockIdx.x * blockDim.x), y = (threadIdx.y + blockIdx.y * blockDim.y);

	if (x >= imgSize.x || y >= imgSize.y) return;

	int locId = x + y * imgSize.x;

	Vector4f ptRay = ptsRay[locId];

	processPixelNormal<TVoxel, TIndex>(outRendering[locId], ptRay.toVector3(), ptRay.w > 0, voxelData, voxelIndex, lightSource);
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
