// Copyright 2014 Isis Innovation Limited and the authors of InfiniTAM

#include "ITMSceneReconstructionEngine_CUDA.h"
#include "ITMCUDAUtils.h"
#include "../../DeviceAgnostic/ITMSceneReconstructionEngine.h"

using namespace ITMLib::Engine;

inline dim3 getGridSize(dim3 taskSize, dim3 blockSize)
{
	return dim3((taskSize.x + blockSize.x - 1) / blockSize.x, (taskSize.y + blockSize.y - 1) / blockSize.y, (taskSize.z + blockSize.z - 1) / blockSize.z);
}

inline dim3 getGridSize(Vector2i taskSize, dim3 blockSize) { return getGridSize(dim3(taskSize.x, taskSize.y), blockSize); }

template<class TVoxel>
__global__ void integrateIntoScene_device(TVoxel *localVBA, const ITMHashEntry *hashTable, int *noLiveEntryIDs, ITMHashCacheState *cacheStates,
	bool useSwapping, const Vector4u *rgb, Vector2i rgbImgSize, const float *depth, Vector2i imgSize, Matrix4f M_d, Matrix4f M_rgb, Vector4f projParams_d, 
	Vector4f projParams_rgb, float _voxelSize, float mu, int maxW);

template<class TVoxel>
__global__ void integrateIntoScene_device(TVoxel *voxelArray, const ITMPlainVoxelArray::ITMVoxelArrayInfo *arrayInfo,
	const Vector4u *rgb, Vector2i rgbImgSize, const float *depth, Vector2i depthImgSize, Matrix4f M_d, Matrix4f M_rgb, Vector4f projParams_d, 
	Vector4f projParams_rgb, float _voxelSize, float mu, int maxW);

template<class TVoxel, class TIndex>
__global__ void createPointCloud_device(uint *noTotalPoints, Vector4f *colours, Vector4f *locations, Vector4u *outRendering,
	const TVoxel *voxelData, const TIndex *voxelIndex, Vector2i imgSize, Matrix4f invM, Vector4f projParams, float voxelSize, 
	float oneOverVoxelSize, const Vector2f *minmaxdata, float mu, bool skipPoints, Vector3f lightSource);

template<class TVoxel, class TIndex>
__global__ void createICPMaps_device(float *depth, Vector4f *pointsMap, Vector4f *normalsMap, Vector4u *outRendering, 
	const TVoxel *voxelData, const TIndex *voxelIndex, Vector2i imgSize, Matrix4f invM, Vector4f projParams, 
	float voxelSize, float oneOverVoxelSize, const Vector2f *minmaxdata, float mu, 
	Vector3f lightSource);

__global__ void buildHashAllocAndVisibleType_device(uchar *entriesAllocType, uchar *entriesVisibleType, Vector3s *blockCoords, const float *depth,
	Matrix4f invM_d, Vector4f projParams_d, float mu, Vector2i _imgSize, float _voxelSize, ITMHashEntry *hashTable, float viewFrustum_min,
	float viewFrustrum_max);

__global__ void allocateVoxelBlocksList_device(int *voxelAllocationList, int *excessAllocationList, ITMHashEntry *hashTable, int noTotalEntries,
	int *noAllocatedVoxelEntries, int *noAllocatedExcessEntries, uchar *entriesAllocType, uchar *entriesVisibleType, Vector3s *blockCoords);

__global__ void reAllocateSwappedOutVoxelBlocks_device(int *voxelAllocationList, ITMHashEntry *hashTable, int noTotalEntries,
	int *noAllocatedVoxelEntries, uchar *entriesVisibleType);

__global__ void buildVisibleList_device(ITMHashEntry *hashTable, ITMHashCacheState *cacheStates, bool useSwapping, int noTotalEntries, 
	int *liveEntryIDs, int *noLiveEntries, uchar *entriesVisibleType, Matrix4f M_d, Vector4f projParams_d, Vector2i imgSize, float voxelSize);

// host methods

template<class TVoxel>
ITMSceneReconstructionEngine_CUDA<TVoxel,ITMVoxelBlockHash>::ITMSceneReconstructionEngine_CUDA(void) 
{
	ITMSafeCall(cudaMalloc((void**)&noTotalPoints_device, sizeof(uint)));
	ITMSafeCall(cudaMalloc((void**)&noLiveEntries_device, sizeof(int)));
	ITMSafeCall(cudaMalloc((void**)&noAllocatedVoxelEntries_device, sizeof(int)));
	ITMSafeCall(cudaMalloc((void**)&noAllocatedExcessEntries_device, sizeof(int)));

	int noTotalEntries = ITMVoxelBlockHash::noVoxelBlocks;
	ITMSafeCall(cudaMalloc((void**)&entriesAllocType_device, noTotalEntries));
	ITMSafeCall(cudaMalloc((void**)&blockCoords_device, noTotalEntries * sizeof(Vector3s)));
}

template<class TVoxel>
ITMSceneReconstructionEngine_CUDA<TVoxel,ITMVoxelBlockHash>::~ITMSceneReconstructionEngine_CUDA(void) 
{
	ITMSafeCall(cudaFree(noTotalPoints_device));
	ITMSafeCall(cudaFree(noLiveEntries_device));
	ITMSafeCall(cudaFree(noAllocatedVoxelEntries_device));
	ITMSafeCall(cudaFree(noAllocatedExcessEntries_device));

	ITMSafeCall(cudaFree(entriesAllocType_device));
	ITMSafeCall(cudaFree(blockCoords_device));
}

template<class TVoxel>
void ITMSceneReconstructionEngine_CUDA<TVoxel,ITMVoxelBlockHash>::AllocateSceneFromDepth(ITMScene<TVoxel,ITMVoxelBlockHash> *scene, const ITMView *view, const ITMPose *pose_d)
{
	Vector2i depthImgSize = view->depth->noDims;
	float voxelSize = scene->sceneParams->voxelSize;

	Matrix4f M_d, invM_d;
	Vector4f projParams_d, invProjParams_d;

	M_d = pose_d->M; M_d.inv(invM_d);

	projParams_d = view->calib->intrinsics_d.projectionParamsSimple.all;
	invProjParams_d = projParams_d;
	invProjParams_d.x = 1.0f / invProjParams_d.x;
	invProjParams_d.y = 1.0f / invProjParams_d.y;

	float mu = scene->sceneParams->mu;

	float *depth = view->depth->GetData(true);
	int *voxelAllocationList = scene->localVBA.GetAllocationList();
	int *excessAllocationList = scene->index.GetExcessAllocationList();
	uchar *entriesVisibleType = scene->index.GetEntriesVisibleType();
	ITMHashEntry *hashTable = scene->index.GetEntries();
	ITMHashCacheState *cacheStates = scene->useSwapping ? scene->globalCache->GetCacheStates(true) : 0;
	int *liveEntryIDs = scene->index.GetLiveEntryIDs();
	int noTotalEntries = scene->index.noVoxelBlocks;

	float oneOverVoxelSize = 1.0f / (voxelSize * SDF_BLOCK_SIZE);

	dim3 cudaBlockSizeHV(16, 16);
	dim3 gridSizeHV((int)ceil((float)depthImgSize.x / (float)cudaBlockSizeHV.x), (int)ceil((float)depthImgSize.y / (float)cudaBlockSizeHV.y));

	ITMSafeCall(cudaMemcpy(noAllocatedVoxelEntries_device, &scene->localVBA.lastFreeBlockId, sizeof(int), cudaMemcpyHostToDevice));
	ITMSafeCall(cudaMemcpy(noAllocatedExcessEntries_device, &scene->index.lastFreeExcessListId, sizeof(int), cudaMemcpyHostToDevice));
	ITMSafeCall(cudaMemset(noLiveEntries_device, 0, sizeof(int)));

	ITMSafeCall(cudaMemset(entriesAllocType_device, 0, sizeof(unsigned char)* noTotalEntries));
	ITMSafeCall(cudaMemset(entriesVisibleType, 0, sizeof(unsigned char)* noTotalEntries));
	ITMSafeCall(cudaMemset(blockCoords_device, 0, sizeof(Vector3s)* noTotalEntries));

	buildHashAllocAndVisibleType_device << <gridSizeHV, cudaBlockSizeHV >> >(entriesAllocType_device, entriesVisibleType, 
		blockCoords_device, depth, invM_d, invProjParams_d, mu, depthImgSize, oneOverVoxelSize, hashTable,
		scene->sceneParams->viewFrustum_min, scene->sceneParams->viewFrustum_max);

	dim3 cudaBlockSizeAL(256, 1);
	dim3 gridSizeAL((int)ceil((float)noTotalEntries / (float)cudaBlockSizeAL.x));

	allocateVoxelBlocksList_device << <gridSizeAL, cudaBlockSizeAL >> >(voxelAllocationList, excessAllocationList, hashTable,
		noTotalEntries, noAllocatedVoxelEntries_device, noAllocatedExcessEntries_device, entriesAllocType_device, entriesVisibleType, 
		blockCoords_device);

	buildVisibleList_device << <gridSizeAL, cudaBlockSizeAL >> >(hashTable, cacheStates, scene->useSwapping, noTotalEntries, liveEntryIDs,
		noLiveEntries_device, entriesVisibleType, M_d, projParams_d, depthImgSize, voxelSize);

	if (scene->useSwapping)
	{
		reAllocateSwappedOutVoxelBlocks_device << <gridSizeAL, cudaBlockSizeAL >> >(voxelAllocationList, hashTable, noTotalEntries, 
			noAllocatedVoxelEntries_device, entriesVisibleType);
	}

	ITMSafeCall(cudaMemcpy(&scene->index.noLiveEntries, noLiveEntries_device, sizeof(int), cudaMemcpyDeviceToHost));
	ITMSafeCall(cudaMemcpy(&scene->localVBA.lastFreeBlockId, noAllocatedVoxelEntries_device, sizeof(int), cudaMemcpyDeviceToHost));
	ITMSafeCall(cudaMemcpy(&scene->index.lastFreeExcessListId, noAllocatedExcessEntries_device, sizeof(int), cudaMemcpyDeviceToHost));
}

template<class TVoxel>
void ITMSceneReconstructionEngine_CUDA<TVoxel,ITMVoxelBlockHash>::IntegrateIntoScene(ITMScene<TVoxel,ITMVoxelBlockHash> *scene, const ITMView *view, const ITMPose *pose_d)
{
	Vector2i rgbImgSize = view->rgb->noDims;
	Vector2i depthImgSize = view->depth->noDims;
	float voxelSize = scene->sceneParams->voxelSize;

	Matrix4f M_d, M_rgb;
	Vector4f projParams_d, projParams_rgb;

	M_d = pose_d->M;
	if (TVoxel::hasColorInformation) M_rgb = view->calib->trafo_rgb_to_depth.calib_inv * pose_d->M;

	projParams_d = view->calib->intrinsics_d.projectionParamsSimple.all;
	projParams_rgb = view->calib->intrinsics_rgb.projectionParamsSimple.all;

	float mu = scene->sceneParams->mu; int maxW = scene->sceneParams->maxW;

	float *depth = view->depth->GetData(true);
	Vector4u *rgb = view->rgb->GetData(true);
	TVoxel *localVBA = scene->localVBA.GetVoxelBlocks();
	ITMHashEntry *hashTable = scene->index.GetEntries();
	ITMHashCacheState *cacheStates = scene->useSwapping ? scene->globalCache->GetCacheStates(true) : 0;
	int *liveEntryIDs = scene->index.GetLiveEntryIDs();

	dim3 cudaBlockSize(SDF_BLOCK_SIZE, SDF_BLOCK_SIZE, SDF_BLOCK_SIZE);
	dim3 gridSize(scene->index.noLiveEntries);

	integrateIntoScene_device << <gridSize, cudaBlockSize >> >(localVBA, hashTable, liveEntryIDs, cacheStates,
		scene->useSwapping, rgb, rgbImgSize, depth, depthImgSize, M_d, M_rgb, projParams_d, projParams_rgb, voxelSize, mu, maxW);
}


template<class TVoxel, class TIndex>
static void CreatePointCloud_common(const ITMScene<TVoxel,TIndex> *scene, const ITMView *view, ITMTrackingState *trackingState, bool skipPoints, uint *noTotalPoints_device)
{
	Vector2i imgSize = view->depth->noDims;
	float voxelSize = scene->sceneParams->voxelSize;
	float oneOverVoxelSize = 1.0f / scene->sceneParams->voxelSize;

	Matrix4f invM = trackingState->pose_d->invM * view->calib->trafo_rgb_to_depth.calib;
	Vector4f projParams = view->calib->intrinsics_rgb.projectionParamsSimple.all;
	projParams.x = 1.0f / projParams.x;
	projParams.y = 1.0f / projParams.y;

	//float presetViewFrustum_min = scene->sceneParams->viewFrustum_min;
	//float presetViewFrustum_max = scene->sceneParams->viewFrustum_max;

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

	createPointCloud_device << <gridSize, cudaBlockSize >> >(noTotalPoints_device, colours, locations, outRendering,
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

	//float presetViewFrustum_min = scene->sceneParams->viewFrustum_min;
	//float presetViewFrustum_max = scene->sceneParams->viewFrustum_max;

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

	createICPMaps_device << <gridSize, cudaBlockSize >> >(depth, pointsMap, normalsMap, outRendering,
		scene->localVBA.GetVoxelBlocks(), scene->index.getIndexData(), imgSize, invM, projParams, voxelSize, oneOverVoxelSize, minmaxdata, 
		mu, lightSource);

	ITMSafeCall(cudaThreadSynchronize());
}

template<class TVoxel>
void ITMSceneReconstructionEngine_CUDA<TVoxel,ITMVoxelBlockHash>::CreatePointCloud(const ITMScene<TVoxel,ITMVoxelBlockHash> *scene, const ITMView *view, ITMTrackingState *trackingState, bool skipPoints)
{
	CreatePointCloud_common(scene, view, trackingState, skipPoints, noTotalPoints_device);
}

template<class TVoxel>
void ITMSceneReconstructionEngine_CUDA<TVoxel,ITMVoxelBlockHash>::CreateICPMaps(const ITMScene<TVoxel,ITMVoxelBlockHash> *scene, const ITMView *view, ITMTrackingState *trackingState)
{
	CreateICPMaps_common(scene, view, trackingState);
}

// plain voxel array

template<class TVoxel>
ITMSceneReconstructionEngine_CUDA<TVoxel,ITMPlainVoxelArray>::ITMSceneReconstructionEngine_CUDA(void) 
{
	ITMSafeCall(cudaMalloc((void**)&noTotalPoints_device, sizeof(uint)));
}

template<class TVoxel>
ITMSceneReconstructionEngine_CUDA<TVoxel,ITMPlainVoxelArray>::~ITMSceneReconstructionEngine_CUDA(void) 
{
	ITMSafeCall(cudaFree(noTotalPoints_device));
}

template<class TVoxel>
void ITMSceneReconstructionEngine_CUDA<TVoxel,ITMPlainVoxelArray>::AllocateSceneFromDepth(ITMScene<TVoxel,ITMPlainVoxelArray> *scene, const ITMView *view, const ITMPose *pose_d)
{
}

template<class TVoxel>
void ITMSceneReconstructionEngine_CUDA<TVoxel,ITMPlainVoxelArray>::IntegrateIntoScene(ITMScene<TVoxel,ITMPlainVoxelArray> *scene, const ITMView *view, const ITMPose *pose_d)
{
	Vector2i rgbImgSize = view->rgb->noDims;
	Vector2i depthImgSize = view->depth->noDims;
	float voxelSize = scene->sceneParams->voxelSize;

	Matrix4f M_d, M_rgb;
	Vector4f projParams_d, projParams_rgb;

	M_d = pose_d->M;
	if (TVoxel::hasColorInformation) M_rgb = view->calib->trafo_rgb_to_depth.calib_inv * pose_d->M;

	projParams_d = view->calib->intrinsics_d.projectionParamsSimple.all;
	projParams_rgb = view->calib->intrinsics_rgb.projectionParamsSimple.all;

	float mu = scene->sceneParams->mu; int maxW = scene->sceneParams->maxW;

	float *depth = view->depth->GetData(true);
	Vector4u *rgb = view->rgb->GetData(true);
	TVoxel *localVBA = scene->localVBA.GetVoxelBlocks();
	const ITMPlainVoxelArray::ITMVoxelArrayInfo *arrayInfo = scene->index.getIndexData();

	dim3 cudaBlockSize(8, 8, 8);
	dim3 gridSize(scene->index.getVolumeSize().x / cudaBlockSize.x, scene->index.getVolumeSize().y / cudaBlockSize.y, scene->index.getVolumeSize().z / cudaBlockSize.z);

	integrateIntoScene_device << <gridSize, cudaBlockSize >> >(localVBA, arrayInfo,
		rgb, rgbImgSize, depth, depthImgSize, M_d, M_rgb, projParams_d, projParams_rgb, voxelSize, mu, maxW);
}


template<class TVoxel>
void ITMSceneReconstructionEngine_CUDA<TVoxel,ITMPlainVoxelArray>::CreatePointCloud(const ITMScene<TVoxel,ITMPlainVoxelArray> *scene, const ITMView *view, ITMTrackingState *trackingState, bool skipPoints)
{
	CreatePointCloud_common(scene, view, trackingState, skipPoints, noTotalPoints_device);
}

template<class TVoxel>
void ITMSceneReconstructionEngine_CUDA<TVoxel,ITMPlainVoxelArray>::CreateICPMaps(const ITMScene<TVoxel,ITMPlainVoxelArray> *scene, const ITMView *view, ITMTrackingState *trackingState)
{
	CreateICPMaps_common(scene, view, trackingState);
}

// device functions

template<class TVoxel>
__global__ void integrateIntoScene_device(TVoxel *voxelArray, const ITMPlainVoxelArray::ITMVoxelArrayInfo *arrayInfo,
	const Vector4u *rgb, Vector2i rgbImgSize, const float *depth, Vector2i depthImgSize, Matrix4f M_d, Matrix4f M_rgb, Vector4f projParams_d, 
	Vector4f projParams_rgb, float _voxelSize, float mu, int maxW)
{
	int x = blockIdx.x*blockDim.x+threadIdx.x;
	int y = blockIdx.y*blockDim.y+threadIdx.y;
	int z = blockIdx.z*blockDim.z+threadIdx.z;

	Vector4f pt_model; int locId;

	locId = x + y * arrayInfo->size.x + z * arrayInfo->size.x * arrayInfo->size.y;

	pt_model.x = (float)(x + arrayInfo->offset.x) * _voxelSize;
	pt_model.y = (float)(y + arrayInfo->offset.y) * _voxelSize;
	pt_model.z = (float)(z + arrayInfo->offset.z) * _voxelSize;
	pt_model.w = 1.0f;

	ComputeUpdatedVoxelInfo<TVoxel::hasColorInformation,TVoxel>::compute(voxelArray[locId], pt_model, M_d, projParams_d, M_rgb, projParams_rgb, mu, maxW, depth, depthImgSize, rgb, rgbImgSize);
}

template<class TVoxel>
__global__ void integrateIntoScene_device(TVoxel *localVBA, const ITMHashEntry *hashTable, int *liveEntryIDs, ITMHashCacheState *cacheStates,
	bool useSwapping, const Vector4u *rgb, Vector2i rgbImgSize, const float *depth, Vector2i depthImgSize, Matrix4f M_d, Matrix4f M_rgb, Vector4f projParams_d, 
	Vector4f projParams_rgb, float _voxelSize, float mu, int maxW)
{
	Vector3i globalPos;
	int entryId = liveEntryIDs[blockIdx.x];

	const ITMHashEntry &currentHashEntry = hashTable[entryId];

	if (currentHashEntry.ptr < 0) return;

	if (useSwapping) { if (cacheStates[entryId].cacheToHost != 2) cacheStates[entryId].cacheToHost = 1; }

	globalPos = currentHashEntry.pos.toInt() * SDF_BLOCK_SIZE;

	TVoxel *localVoxelBlock = &(localVBA[currentHashEntry.ptr * SDF_BLOCK_SIZE3]);

	int x = threadIdx.x, y = threadIdx.y, z = threadIdx.z;

	Vector4f pt_model; int locId;

	locId = x + y * SDF_BLOCK_SIZE + z * SDF_BLOCK_SIZE * SDF_BLOCK_SIZE;

	pt_model.x = (float)(globalPos.x + x) * _voxelSize;
	pt_model.y = (float)(globalPos.y + y) * _voxelSize;
	pt_model.z = (float)(globalPos.z + z) * _voxelSize;
	pt_model.w = 1.0f;

	ComputeUpdatedVoxelInfo<TVoxel::hasColorInformation,TVoxel>::compute(localVoxelBlock[locId], pt_model, M_d, projParams_d, M_rgb, projParams_rgb, mu, maxW, depth, depthImgSize, rgb, rgbImgSize);
}

template<class TVoxel, class TAccess>
__global__ void createPointCloud_device(uint *noTotalPoints, Vector4f *colours, Vector4f *locations, Vector4u *outRendering,
	const TVoxel *voxelData, const TAccess *voxelIndex, Vector2i imgSize, Matrix4f invM, Vector4f projParams, float voxelSize, 
	float oneOverVoxelSize, const Vector2f *minmaxdata, float mu, bool skipPoints, Vector3f lightSource)
{
	int x = (threadIdx.x + blockIdx.x * blockDim.x), y = (threadIdx.y + blockIdx.y * blockDim.y);

	if (x >= imgSize.x || y >= imgSize.y) return;

	__shared__ bool shouldPrefix;

	Vector3f pt_ray; Vector4f tmp;
	float angle;

	int locId = x + y * imgSize.x;

//	float viewFrustum_min = presetViewFrustum_min;
//	float viewFrustum_max = presetViewFrustum_max;
	float viewFrustum_min = minmaxdata[locId].x;
	float viewFrustum_max = minmaxdata[locId].y;

	shouldPrefix = false;

	bool foundPoint = false;
	foundPoint = castRay(pt_ray, x, y, voxelData, voxelIndex, invM, projParams, imgSize, oneOverVoxelSize, mu, viewFrustum_min, viewFrustum_max);

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

			tmp = VoxelColorReader<TVoxel::hasColorInformation,TVoxel,TAccess>::interpolate(voxelData, voxelIndex, pt_ray);
			if (tmp.w > 0.0f) { tmp.x /= tmp.w; tmp.y /= tmp.w; tmp.z /= tmp.w; tmp.w = 1.0f; }
			colours[offset] = tmp;

			pt_ray_out.x = pt_ray.x * voxelSize; pt_ray_out.y = pt_ray.y * voxelSize;
			pt_ray_out.z = pt_ray.z * voxelSize; pt_ray_out.w = 1.0f;
			locations[offset] = pt_ray_out;
		}
	}
}

template<class TVoxel, class TAccess>
__global__ void createICPMaps_device(float *depth, Vector4f *pointsMap, Vector4f *normalsMap, Vector4u *outRendering, 
	const TVoxel *voxelData, const TAccess *voxelIndex, Vector2i imgSize, Matrix4f invM, Vector4f projParams, 
	float voxelSize, float oneOverVoxelSize, const Vector2f *minmaxdata, float mu, 
	Vector3f lightSource)
{
	int x = (threadIdx.x + blockIdx.x * blockDim.x), y = (threadIdx.y + blockIdx.y * blockDim.y);

	if (x >= imgSize.x || y >= imgSize.y) return;

	Vector3f pt_ray; Vector4f tmp;
	Vector3f outNormal;
	float angle;

	int locId = x + y * imgSize.x;

	//float viewFrustum_min = 0.20; //presetViewFrustum_min
	//float viewFrustum_max = 3.00; //presetViewFrustum_max
	float viewFrustum_min = minmaxdata[locId].x;
	float viewFrustum_max = minmaxdata[locId].y;

	bool foundPoint = false;
	foundPoint = castRay(pt_ray, x, y, voxelData, voxelIndex, invM, projParams, imgSize, oneOverVoxelSize, mu, viewFrustum_min, viewFrustum_max);

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

		pointsMap[x + y * imgSize.x] = out4;
		normalsMap[x + y * imgSize.x] = out4;
	}
}

__global__ void buildHashAllocAndVisibleType_device(uchar *entriesAllocType, uchar *entriesVisibleType, Vector3s *blockCoords, const float *depth,
	Matrix4f invM_d, Vector4f projParams_d, float mu, Vector2i _imgSize, float _voxelSize, ITMHashEntry *hashTable, float viewFrustum_min,
	float viewFrustum_max)
{
	int x = threadIdx.x + blockIdx.x * blockDim.x, y = threadIdx.y + blockIdx.y * blockDim.y;

	if (x > _imgSize.x - 1 || y > _imgSize.y - 1) return;

	buildHashAllocAndVisibleTypePP(entriesAllocType, entriesVisibleType, x, y, blockCoords, depth, invM_d,
		projParams_d, mu, _imgSize, _voxelSize, hashTable, viewFrustum_min, viewFrustum_max);
}

__global__ void allocateVoxelBlocksList_device(int *voxelAllocationList, int *excessAllocationList, ITMHashEntry *hashTable, int noTotalEntries,
	int *noAllocatedVoxelEntries, int *noAllocatedExcessEntries, uchar *entriesAllocType, uchar *entriesVisibleType, Vector3s *blockCoords)
{
	int targetIdx = threadIdx.x + blockIdx.x * blockDim.x;
	if (targetIdx > noTotalEntries - 1) return;

	int vbaIdx, exlIdx;
	ITMHashEntry hashEntry = hashTable[targetIdx];

	switch (entriesAllocType[targetIdx])
	{
	case 1: //needs allocation, fits in the ordered list
		vbaIdx = atomicSub(&noAllocatedVoxelEntries[0], 1);

		if (vbaIdx >= 0) //there is room in the voxel block array
		{
			Vector3s pt_block_all = blockCoords[targetIdx];

			hashEntry.pos.x = pt_block_all.x; hashEntry.pos.y = pt_block_all.y; hashEntry.pos.z = pt_block_all.z;
			hashEntry.ptr = voxelAllocationList[vbaIdx];

			hashTable[targetIdx] = hashEntry;
		}
		break;

	case 2: //needs allocation in the excess list
		vbaIdx = atomicSub(&noAllocatedVoxelEntries[0], 1);
		exlIdx = atomicSub(&noAllocatedExcessEntries[0], 1);

		if (vbaIdx >= 0 && exlIdx >= 0) //there is room in the voxel block array and excess list
		{
			Vector3s pt_block_all = blockCoords[targetIdx];

			hashEntry.pos.x = pt_block_all.x; hashEntry.pos.y = pt_block_all.y; hashEntry.pos.z = pt_block_all.z;
			hashEntry.ptr = voxelAllocationList[vbaIdx];

			int exlOffset = excessAllocationList[exlIdx];

			hashTable[targetIdx].offset = exlOffset + 1; //connect to child

			hashTable[SDF_BUCKET_NUM * SDF_ENTRY_NUM_PER_BUCKET + exlOffset] = hashEntry; //add child to the excess list

			entriesVisibleType[SDF_BUCKET_NUM * SDF_ENTRY_NUM_PER_BUCKET + exlOffset] = 1; //make child visible
		}

		break;
	}
}

__global__ void reAllocateSwappedOutVoxelBlocks_device(int *voxelAllocationList, ITMHashEntry *hashTable, int noTotalEntries,
	int *noAllocatedVoxelEntries, uchar *entriesVisibleType)
{
	int targetIdx = threadIdx.x + blockIdx.x * blockDim.x;
	if (targetIdx > noTotalEntries - 1) return;

	int vbaIdx;
	ITMHashEntry hashEntry = hashTable[targetIdx];

	if (entriesVisibleType[targetIdx] > 0 && hashEntry.ptr == -1) //it is visible and has been previously allocated inside the hash, but deallocated from VBA
	{
		vbaIdx = atomicSub(&noAllocatedVoxelEntries[0], 1);
		if (vbaIdx >= 0) hashTable[targetIdx].ptr = voxelAllocationList[vbaIdx];
	}
}

__global__ void buildVisibleList_device(ITMHashEntry *hashTable, ITMHashCacheState *cacheStates, bool useSwapping, int noTotalEntries, 
	int *liveEntryIDs, int *noLiveEntries, uchar *entriesVisibleType, Matrix4f M_d, Vector4f projParams_d, Vector2i imgSize, float voxelSize)
{
	int targetIdx = threadIdx.x + blockIdx.x * blockDim.x;
	if (targetIdx > noTotalEntries - 1) return;

	__shared__ bool shouldPrefix;

	unsigned char hashVisibleType = entriesVisibleType[targetIdx];
	const ITMHashEntry &hashEntry = hashTable[targetIdx];

	shouldPrefix = false;
	__syncthreads();

	if (hashVisibleType > 0) shouldPrefix = true;
	else
	{
		if (hashEntry.ptr >= -1)
		{
			shouldPrefix = true;

			Vector3f pt_image, buff3f;

			int noInvisible = 0, noInvisibleEnlarged = 0;

			pt_image = hashEntry.pos.toFloat() * (float)SDF_BLOCK_SIZE * voxelSize;
			buff3f = M_d * pt_image;

			if (buff3f.z > 1e-10f)
			{
				shouldPrefix = true;

				for (int x = 0; x <= 1; x++) for (int y = 0; y <= 1; y++) for (int z = 0; z <= 1; z++)
				{
					Vector3f off((float)x, (float)y, (float)z);

					pt_image = (hashEntry.pos.toFloat() + off) * (float)SDF_BLOCK_SIZE * voxelSize;

					buff3f = M_d * pt_image;

					pt_image.x = projParams_d.x * buff3f.x / buff3f.z + projParams_d.z;
					pt_image.y = projParams_d.y * buff3f.y / buff3f.z + projParams_d.w;

					if (!(pt_image.x >= 0 && pt_image.x < imgSize.x && pt_image.y >= 0 && pt_image.y < imgSize.y)) noInvisible++;

					if (useSwapping)
					{
						Vector4i lims;
						lims.x = -imgSize.x / 8; lims.y = imgSize.x + imgSize.x / 8;
						lims.z = -imgSize.y / 8; lims.w = imgSize.y + imgSize.y / 8;

						if (!(pt_image.x >= lims.x && pt_image.x < lims.y && pt_image.y >= lims.z && pt_image.y < lims.w)) noInvisibleEnlarged++;
					}
				}

				hashVisibleType = noInvisible < 8;

				if (useSwapping) entriesVisibleType[targetIdx] = noInvisibleEnlarged < 8;
			}
		}
	}

	if (useSwapping)
	{
		if (entriesVisibleType[targetIdx] > 0 && cacheStates[targetIdx].cacheFromHost != 2) cacheStates[targetIdx].cacheFromHost = 1;
	}

	__syncthreads();

	if (shouldPrefix)
	{
		int offset = computePrefixSum_device<int>(hashVisibleType > 0, noLiveEntries, blockDim.x * blockDim.y, threadIdx.x);
		if (offset != -1) liveEntryIDs[offset] = targetIdx;
	}
}

template class ITMSceneReconstructionEngine_CUDA<ITMVoxel,ITMVoxelIndex>;

