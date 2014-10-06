// Copyright 2014 Isis Innovation Limited and the authors of InfiniTAM

#include "ITMSceneReconstructionEngine_CPU.h"
#include "../../DeviceAgnostic/ITMSceneReconstructionEngine.h"

using namespace ITMLib::Engine;

template<class TVoxel>
ITMSceneReconstructionEngine_CPU<TVoxel,ITMVoxelBlockHash>::ITMSceneReconstructionEngine_CPU(void) 
{
	int noTotalEntries = ITMVoxelBlockHash::noVoxelBlocks;
	entriesAllocType = (uchar*)malloc(noTotalEntries);
	blockCoords = (Vector3s*)malloc(noTotalEntries * sizeof(Vector3s));
}

template<class TVoxel>
ITMSceneReconstructionEngine_CPU<TVoxel,ITMVoxelBlockHash>::~ITMSceneReconstructionEngine_CPU(void) 
{
	free(entriesAllocType);
	free(blockCoords);
}

template<class TVoxel>
void ITMSceneReconstructionEngine_CPU<TVoxel,ITMVoxelBlockHash>::IntegrateIntoScene(ITMScene<TVoxel,ITMVoxelBlockHash> *scene, const ITMView *view, const ITMPose *pose_d)
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

	float *depth = view->depth->GetData(false);
	Vector4u *rgb = view->rgb->GetData(false);
	TVoxel *localVBA = scene->localVBA.GetVoxelBlocks();
	ITMHashEntry *hashTable = scene->index.GetEntries();
	ITMHashCacheState *cacheStates = scene->useSwapping ? scene->globalCache->GetCacheStates(false) : 0;
	int *liveEntryIDs = scene->index.GetLiveEntryIDs();

	bool useSwapping = scene->useSwapping;

	for (int entryId = 0; entryId < scene->index.noLiveEntries; entryId++)
	{
		Vector3i globalPos;
		const ITMHashEntry &currentHashEntry = hashTable[liveEntryIDs[entryId]];

		if (currentHashEntry.ptr < 0) continue;

		if (useSwapping)
		{
			if (cacheStates[liveEntryIDs[entryId]].cacheToHost != 2)
				cacheStates[liveEntryIDs[entryId]].cacheToHost = 1;
		}

		globalPos.x = currentHashEntry.pos.x;
		globalPos.y = currentHashEntry.pos.y;
		globalPos.z = currentHashEntry.pos.z;
		globalPos *= SDF_BLOCK_SIZE;

		TVoxel *localVoxelBlock = &(localVBA[currentHashEntry.ptr * (SDF_BLOCK_SIZE3)]);

		for (int z = 0; z < SDF_BLOCK_SIZE; z++) for (int y = 0; y < SDF_BLOCK_SIZE; y++) for (int x = 0; x < SDF_BLOCK_SIZE; x++)
		{
			Vector4f pt_model; int locId;

			locId = x + y * SDF_BLOCK_SIZE + z * SDF_BLOCK_SIZE * SDF_BLOCK_SIZE;

			pt_model.x = (float)(globalPos.x + x) * voxelSize;
			pt_model.y = (float)(globalPos.y + y) * voxelSize;
			pt_model.z = (float)(globalPos.z + z) * voxelSize;
			pt_model.w = 1.0f;

			ComputeUpdatedVoxelInfo<TVoxel::hasColorInformation,TVoxel>::compute(localVoxelBlock[locId], pt_model, M_d, projParams_d, M_rgb, projParams_rgb, mu, maxW, depth, depthImgSize, rgb, rgbImgSize);
		}
	}
}

template<class TVoxel>
void ITMSceneReconstructionEngine_CPU<TVoxel,ITMVoxelBlockHash>::AllocateSceneFromDepth(ITMScene<TVoxel,ITMVoxelBlockHash> *scene, const ITMView *view, const ITMPose *pose_d)
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

	float *depth = view->depth->GetData(false);
	int *voxelAllocationList = scene->localVBA.GetAllocationList();
	int *excessAllocationList = scene->index.GetExcessAllocationList();
	uchar *entriesVisibleType = scene->index.GetEntriesVisibleType();
	ITMHashEntry *hashTable = scene->index.GetEntries();
	ITMHashCacheState *cacheStates = scene->useSwapping ? scene->globalCache->GetCacheStates(false) : 0;
	int *liveEntryIDs = scene->index.GetLiveEntryIDs();
	int noTotalEntries = scene->index.noVoxelBlocks;

	bool useSwapping = scene->useSwapping;

	float oneOverVoxelSize = 1.0f / (voxelSize * SDF_BLOCK_SIZE);

	int lastFreeVoxelBlockId = scene->localVBA.lastFreeBlockId;
	int lastFreeExcessListId = scene->index.lastFreeExcessListId;

	Vector3s pt_block_prev;
	pt_block_prev.x = 0; pt_block_prev.y = 0; pt_block_prev.z = 0;

	int hashIdxLive = 0;

	memset(entriesAllocType, 0, noTotalEntries);
	memset(entriesVisibleType, 0, noTotalEntries);
	memset(blockCoords, 0, noTotalEntries * sizeof(Vector3s));

	//build hashVisibility
	for (int y = 0; y < depthImgSize.y; y++) for (int x = 0; x < depthImgSize.x; x++)
	{
		buildHashAllocAndVisibleTypePP(entriesAllocType, entriesVisibleType, x, y, blockCoords, depth, invM_d,
			invProjParams_d, mu, depthImgSize, oneOverVoxelSize, hashTable, scene->sceneParams->viewFrustum_min,
			scene->sceneParams->viewFrustum_max);
	}

	//allocate
	for (int targetIdx = 0; targetIdx < noTotalEntries; targetIdx++)
	{
		int vbaIdx, exlIdx;
		unsigned char hashChangeType = entriesAllocType[targetIdx];
		ITMHashEntry hashEntry = hashTable[targetIdx];

		switch (hashChangeType)
		{
		case 1: //needs allocation, fits in the ordered list
			vbaIdx = lastFreeVoxelBlockId; lastFreeVoxelBlockId--;

			if (vbaIdx >= 0) //there is room in the voxel block array
			{
				Vector3s pt_block_all = blockCoords[targetIdx];

				hashEntry.pos.x = pt_block_all.x; hashEntry.pos.y = pt_block_all.y; hashEntry.pos.z = pt_block_all.z;
				hashEntry.ptr = voxelAllocationList[vbaIdx];

				hashTable[targetIdx] = hashEntry;
			}

			break;
		case 2: //needs allocation in the excess list
			vbaIdx = lastFreeVoxelBlockId; lastFreeVoxelBlockId--;
			exlIdx = lastFreeExcessListId; lastFreeExcessListId--;

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

	//build visible list
	for (int targetIdx = 0; targetIdx < noTotalEntries; targetIdx++)
	{
		unsigned char hashVisibleType = entriesVisibleType[targetIdx];
		const ITMHashEntry &hashEntry = hashTable[targetIdx];

		if ((hashVisibleType == 0 && hashEntry.ptr >= 0) || hashVisibleType == 2)
		{
			Vector3f pt_image, buff3f;

			int noInvisible = 0, noInvisibleEnlarged = 0;

			pt_image = hashEntry.pos.toFloat() * (float)SDF_BLOCK_SIZE * voxelSize;
			buff3f = M_d * pt_image;

			if (buff3f.z > 1e-10f)
			{
				for (int x = 0; x <= 1; x++) for (int y = 0; y <= 1; y++) for (int z = 0; z <= 1; z++)
				{
					Vector3f off((float)x, (float)y, (float)z);

					pt_image = (hashEntry.pos.toFloat() + off) * (float)SDF_BLOCK_SIZE * voxelSize;

					buff3f = M_d * pt_image;

					pt_image.x = projParams_d.x * buff3f.x / buff3f.z + projParams_d.z;
					pt_image.y = projParams_d.y * buff3f.y / buff3f.z + projParams_d.w;

					if (!(pt_image.x >= 0 && pt_image.x < depthImgSize.x && pt_image.y >= 0 && pt_image.y < depthImgSize.y)) noInvisible++;

					if (useSwapping)
					{
						Vector4i lims;
						lims.x = -depthImgSize.x / 8; lims.y = depthImgSize.x + depthImgSize.x / 8;
						lims.z = -depthImgSize.y / 8; lims.w = depthImgSize.y + depthImgSize.y / 8;

						if (!(pt_image.x >= lims.x && pt_image.x < lims.y && pt_image.y >= lims.z && pt_image.y < lims.w)) noInvisibleEnlarged++;
					}
				}

				hashVisibleType = noInvisible < 8;

				if (useSwapping) entriesVisibleType[targetIdx] = noInvisibleEnlarged < 8;
			}
		}

		if (useSwapping)
		{
			if (entriesVisibleType[targetIdx] > 0 && cacheStates[targetIdx].cacheFromHost != 2) cacheStates[targetIdx].cacheFromHost = 1;
		}

		if (hashVisibleType > 0)
		{
			liveEntryIDs[hashIdxLive] = targetIdx;
			hashIdxLive++;
		}
	}

	//reallocate deletes ones from previous swap operation
	if (useSwapping)
	{
		for (int targetIdx = 0; targetIdx < noTotalEntries; targetIdx++)
		{
			int vbaIdx;
			ITMHashEntry hashEntry = hashTable[targetIdx];

			if (entriesVisibleType[targetIdx] > 0 && hashEntry.ptr == -1) 
			{
				vbaIdx = lastFreeVoxelBlockId; lastFreeVoxelBlockId--;
				if (vbaIdx >= 0) hashTable[targetIdx].ptr = voxelAllocationList[vbaIdx];
			}
		}
	}

	scene->index.noLiveEntries = hashIdxLive;
	scene->localVBA.lastFreeBlockId = lastFreeVoxelBlockId;
	scene->index.lastFreeExcessListId = lastFreeExcessListId;
}

template<class TVoxel, class TIndex>
static void CreatePointCloud_common(const ITMScene<TVoxel,TIndex> *scene, const ITMView *view, ITMTrackingState *trackingState, bool skipPoints)
{
	Vector2i imgSize = view->depth->noDims;
	float voxelSize = scene->sceneParams->voxelSize;
	float oneOverVoxelSize = 1.0f / scene->sceneParams->voxelSize;

	//Matrix4f invM = trackingState->pose->invM;
	Matrix4f invM = trackingState->pose_d->invM * view->calib->trafo_rgb_to_depth.calib;
	Vector4f projParams = view->calib->intrinsics_rgb.projectionParamsSimple.all;
	projParams.x = 1.0f / projParams.x;
	projParams.y = 1.0f / projParams.y;

	//float presetViewFrustum_min = scene->sceneParams->viewFrustum_min;
	//float presetViewFrustum_max = scene->sceneParams->viewFrustum_max;

	Vector4f *locations = trackingState->pointCloud->locations->GetData(false);
	Vector4f *colours = trackingState->pointCloud->colours->GetData(false);
	Vector4u *outRendering = trackingState->rendering->GetData(false);
	const Vector2f *minmaximg = trackingState->renderingRangeImage->GetData(false);

	float mu = scene->sceneParams->mu;
	Vector3f lightSource = -Vector3f(invM.getColumn(2));

	memset(outRendering, 0, sizeof(Vector4u)* imgSize.x * imgSize.y);

	int offset = 0;
	for (int y = 0; y < imgSize.y; y++) for (int x = 0; x < imgSize.x; x++)
	{
		Vector3f pt_ray; Vector4f tmp;
		float angle;

		int locId = x + y * imgSize.x;

//		float viewFrustum_min = presetViewFrustum_min;
//		float viewFrustum_max = presetViewFrustum_max;
		float viewFrustum_min = minmaximg[locId].x;
		float viewFrustum_max = minmaximg[locId].y;

		bool foundPoint = false;
		foundPoint = castRay(pt_ray, x, y, scene->localVBA.GetVoxelBlocks(), scene->index.getIndexData(), invM, projParams, imgSize, oneOverVoxelSize, mu, viewFrustum_min, viewFrustum_max);

		if (foundPoint)
		{
			Vector3f outNormal = computeSingleNormalFromSDF(scene->localVBA.GetVoxelBlocks(), scene->index.getIndexData(), pt_ray);

			float normScale = 1.0f / sqrtf(outNormal.x * outNormal.x + outNormal.y * outNormal.y + outNormal.z * outNormal.z);
			outNormal *= normScale;

			angle = outNormal.x * lightSource.x + outNormal.y * lightSource.y + outNormal.z * lightSource.z;
			if (!(angle>0.0f)) foundPoint = false;
		}

		if (foundPoint) {
			float outRes = (0.8f * angle + 0.2f) * 255.0f;
			outRendering[x + y * imgSize.x] = (uchar)outRes;
		}

		if (skipPoints && ((x % 2 == 0) || (y % 2 == 0))) foundPoint = false;

		if (foundPoint)
		{
			Vector4f pt_ray_out;

			tmp = VoxelColorReader<TVoxel::hasColorInformation,TVoxel, typename TIndex::IndexData>::interpolate(scene->localVBA.GetVoxelBlocks(), scene->index.getIndexData(), pt_ray);
			if (tmp.w > 0.0f) { tmp.x /= tmp.w; tmp.y /= tmp.w; tmp.z /= tmp.w; tmp.w = 1.0f; }
			colours[offset] = tmp;

			pt_ray_out.x = pt_ray.x * voxelSize; pt_ray_out.y = pt_ray.y * voxelSize;
			pt_ray_out.z = pt_ray.z * voxelSize; pt_ray_out.w = 1.0f;
			locations[offset] = pt_ray_out;

			offset++;
		}
	}

	trackingState->pointCloud->noTotalPoints = offset;
}

template<class TVoxel, class TIndex>
static void CreateICPMaps_common(const ITMScene<TVoxel,TIndex> *scene, const ITMView *view, ITMTrackingState *trackingState)
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

	Vector4f *pointsMap = trackingState->pointCloud->locations->GetData(false);
	Vector4f *normalsMap = trackingState->pointCloud->colours->GetData(false);
	Vector4u *outRendering = trackingState->rendering->GetData(false);
	const Vector2f *minmaximg = trackingState->renderingRangeImage->GetData(false);

	memset(outRendering, 0, sizeof(Vector4u)* imgSize.x * imgSize.y);
	memset(pointsMap, 0, sizeof(Vector4f)* imgSize.x * imgSize.y);
	memset(normalsMap, 0, sizeof(Vector4f)* imgSize.x * imgSize.y);

	for (int y = 0; y < imgSize.y; y++) for (int x = 0; x < imgSize.x; x++)
	{
		Vector3f pt_ray; Vector4f tmp;
		Vector3f outNormal;
		float angle;

		int locId = x + y * imgSize.x;

//		float viewFrustum_min = presetViewFrustum_min;
//		float viewFrustum_max = presetViewFrustum_max;
		float viewFrustum_min = minmaximg[locId].x;
		float viewFrustum_max = minmaximg[locId].y;

		bool foundPoint = false;
		foundPoint = castRay(pt_ray, x, y, scene->localVBA.GetVoxelBlocks(), scene->index.getIndexData(), invM, projParams, imgSize, oneOverVoxelSize, mu, viewFrustum_min, viewFrustum_max);

		if (foundPoint)
		{
			outNormal = computeSingleNormalFromSDF(scene->localVBA.GetVoxelBlocks(), scene->index.getIndexData(), pt_ray);

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
}

template<class TVoxel>
void ITMSceneReconstructionEngine_CPU<TVoxel,ITMVoxelBlockHash>::CreatePointCloud(const ITMScene<TVoxel,ITMVoxelBlockHash> *scene, const ITMView *view, ITMTrackingState *trackingState, bool skipPoints)
{
	CreatePointCloud_common(scene, view, trackingState, skipPoints);
}

template<class TVoxel>
void ITMSceneReconstructionEngine_CPU<TVoxel,ITMVoxelBlockHash>::CreateICPMaps(const ITMScene<TVoxel,ITMVoxelBlockHash> *scene, const ITMView *view, ITMTrackingState *trackingState)
{
	CreateICPMaps_common(scene, view, trackingState);
}

template<class TVoxel>
ITMSceneReconstructionEngine_CPU<TVoxel,ITMPlainVoxelArray>::ITMSceneReconstructionEngine_CPU(void) 
{}

template<class TVoxel>
ITMSceneReconstructionEngine_CPU<TVoxel,ITMPlainVoxelArray>::~ITMSceneReconstructionEngine_CPU(void) 
{}

template<class TVoxel>
void ITMSceneReconstructionEngine_CPU<TVoxel,ITMPlainVoxelArray>::AllocateSceneFromDepth(ITMScene<TVoxel,ITMPlainVoxelArray> *scene, const ITMView *view, const ITMPose *pose_d)
{}

template<class TVoxel>
void ITMSceneReconstructionEngine_CPU<TVoxel,ITMPlainVoxelArray>::IntegrateIntoScene(ITMScene<TVoxel,ITMPlainVoxelArray> *scene, const ITMView *view, const ITMPose *pose_d)
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

	float *depth = view->depth->GetData(false);
	Vector4u *rgb = view->rgb->GetData(false);
	TVoxel *voxelArray = scene->localVBA.GetVoxelBlocks();

	const ITMPlainVoxelArray::IndexData *arrayInfo = scene->index.getIndexData();

	for (int z = 0; z < scene->index.getVolumeSize().z; z++) for (int y = 0; y < scene->index.getVolumeSize().y; y++) for (int x = 0; x < scene->index.getVolumeSize().x; x++)
	{
		Vector4f pt_model; int locId;

		locId = x + y * scene->index.getVolumeSize().x + z * scene->index.getVolumeSize().x * scene->index.getVolumeSize().y;

		pt_model.x = (float)(x + arrayInfo->offset.x) * voxelSize;
		pt_model.y = (float)(y + arrayInfo->offset.y) * voxelSize;
		pt_model.z = (float)(z + arrayInfo->offset.z) * voxelSize;
		pt_model.w = 1.0f;

		ComputeUpdatedVoxelInfo<TVoxel::hasColorInformation,TVoxel>::compute(voxelArray[locId], pt_model, M_d, projParams_d, M_rgb, projParams_rgb, mu, maxW, depth, depthImgSize, rgb, rgbImgSize);
	}
}

template<class TVoxel>
void ITMSceneReconstructionEngine_CPU<TVoxel,ITMPlainVoxelArray>::CreatePointCloud(const ITMScene<TVoxel,ITMPlainVoxelArray> *scene, const ITMView *view, ITMTrackingState *trackingState, bool skipPoints)
{
	CreatePointCloud_common(scene, view, trackingState, skipPoints);
}

template<class TVoxel>
void ITMSceneReconstructionEngine_CPU<TVoxel,ITMPlainVoxelArray>::CreateICPMaps(const ITMScene<TVoxel,ITMPlainVoxelArray> *scene, const ITMView *view, ITMTrackingState *trackingState)
{
	CreateICPMaps_common(scene, view, trackingState);
}

template class ITMSceneReconstructionEngine_CPU<ITMVoxel,ITMVoxelIndex>;
