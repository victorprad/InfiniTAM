// Copyright 2014 Isis Innovation Limited and the authors of InfiniTAM

#include "ITMSceneReconstructionEngine_CPU.h"
#include "../../DeviceAgnostic/ITMSceneReconstructionEngine.h"
#include "../../../Objects/ITMRenderState_VH.h"

using namespace ITMLib::Engine;

template<class TVoxel>
ITMSceneReconstructionEngine_CPU<TVoxel,ITMVoxelBlockHash>::ITMSceneReconstructionEngine_CPU(void) 
{
	int noTotalEntries = ITMVoxelBlockHash::noVoxelBlocks;
	entriesAllocType = new ORUtils::MemoryBlock<unsigned char>(noTotalEntries, MEMORYDEVICE_CPU);
	blockCoords = new ORUtils::MemoryBlock<Vector4s>(noTotalEntries, MEMORYDEVICE_CPU);
}

template<class TVoxel>
ITMSceneReconstructionEngine_CPU<TVoxel,ITMVoxelBlockHash>::~ITMSceneReconstructionEngine_CPU(void) 
{
	delete entriesAllocType;
	delete blockCoords;
}

template<class TVoxel>
void ITMSceneReconstructionEngine_CPU<TVoxel, ITMVoxelBlockHash>::IntegrateIntoScene(ITMScene<TVoxel, ITMVoxelBlockHash> *scene, const ITMView *view,
	const ITMTrackingState *trackingState, const ITMRenderState *renderState)
{
	Vector2i rgbImgSize = view->rgb->noDims;
	Vector2i depthImgSize = view->depth->noDims;
	float voxelSize = scene->sceneParams->voxelSize;

	Matrix4f M_d, M_rgb;
	Vector4f projParams_d, projParams_rgb;

	ITMRenderState_VH *renderState_vh = (ITMRenderState_VH*)renderState;

	M_d = trackingState->pose_d->M;
	if (TVoxel::hasColorInformation) M_rgb = view->calib->trafo_rgb_to_depth.calib_inv * trackingState->pose_d->M;

	projParams_d = view->calib->intrinsics_d.projectionParamsSimple.all;
	projParams_rgb = view->calib->intrinsics_rgb.projectionParamsSimple.all;

	float mu = scene->sceneParams->mu; int maxW = scene->sceneParams->maxW;

	float *depth = view->depth->GetData(MEMORYDEVICE_CPU);
	Vector4u *rgb = view->rgb->GetData(MEMORYDEVICE_CPU);
	TVoxel *localVBA = scene->localVBA.GetVoxelBlocks();
	ITMHashEntry *hashTable = scene->index.GetEntries();

	int *liveEntryIDs = renderState_vh->GetLiveEntryIDs();
	int noLiveEntries = renderState_vh->noLiveEntries;

	for (int entryId = 0; entryId < noLiveEntries; entryId++)
	{
		Vector3i globalPos;
		const ITMHashEntry &currentHashEntry = hashTable[liveEntryIDs[entryId]];

		if (currentHashEntry.ptr < 0) continue;

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

			ComputeUpdatedVoxelInfo<TVoxel::hasColorInformation,TVoxel>::compute(localVoxelBlock[locId], pt_model, M_d, 
				projParams_d, M_rgb, projParams_rgb, mu, maxW, depth, depthImgSize, rgb, rgbImgSize);
		}
	}
}

template<class TVoxel>
void ITMSceneReconstructionEngine_CPU<TVoxel, ITMVoxelBlockHash>::AllocateSceneFromDepth(ITMScene<TVoxel, ITMVoxelBlockHash> *scene, const ITMView *view,
	const ITMTrackingState *trackingState, const ITMRenderState *renderState)
{
	Vector2i depthImgSize = view->depth->noDims;
	float voxelSize = scene->sceneParams->voxelSize;

	Matrix4f M_d, invM_d;
	Vector4f projParams_d, invProjParams_d;

	ITMRenderState_VH *renderState_vh = (ITMRenderState_VH*)renderState;

	M_d = trackingState->pose_d->M; M_d.inv(invM_d);

	projParams_d = view->calib->intrinsics_d.projectionParamsSimple.all;
	invProjParams_d = projParams_d;
	invProjParams_d.x = 1.0f / invProjParams_d.x;
	invProjParams_d.y = 1.0f / invProjParams_d.y;

	float mu = scene->sceneParams->mu;

	float *depth = view->depth->GetData(MEMORYDEVICE_CPU);
	int *voxelAllocationList = scene->localVBA.GetAllocationList();
	int *excessAllocationList = scene->index.GetExcessAllocationList();
	ITMHashEntry *hashTable = scene->index.GetEntries();
	ITMHashCacheState *cacheStates = scene->useSwapping ? scene->globalCache->GetCacheStates(false) : 0;
	int *liveEntryIDs = renderState_vh->GetLiveEntryIDs();
	uchar *entriesVisibleType = renderState_vh->GetEntriesVisibleType();
	uchar *entriesAllocType = this->entriesAllocType->GetData(MEMORYDEVICE_CPU);
	Vector4s *blockCoords = this->blockCoords->GetData(MEMORYDEVICE_CPU);
	int noTotalEntries = scene->index.noVoxelBlocks;

	bool useSwapping = scene->useSwapping;

	float oneOverVoxelSize = 1.0f / (voxelSize * SDF_BLOCK_SIZE);

	int lastFreeVoxelBlockId = scene->localVBA.lastFreeBlockId;
	int lastFreeExcessListId = scene->index.lastFreeExcessListId;

	int hashIdxLive = 0;

	memset(entriesAllocType, 0, noTotalEntries);
	memset(entriesVisibleType, 0, noTotalEntries);
	memset(blockCoords, 0, noTotalEntries * sizeof(Vector4s));

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
				Vector4s pt_block_all = blockCoords[targetIdx];

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
				Vector4s pt_block_all = blockCoords[targetIdx];

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
			bool isVisibleEnlarged = false;

			if (useSwapping)
			{
				checkBlockVisibility<true>(hashVisibleType, isVisibleEnlarged, hashEntry.pos, M_d, projParams_d, voxelSize, depthImgSize);
				entriesVisibleType[targetIdx] = isVisibleEnlarged;
			}
			else checkBlockVisibility<false>(hashVisibleType, isVisibleEnlarged, hashEntry.pos, M_d, projParams_d, voxelSize, depthImgSize);
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

	renderState_vh->noLiveEntries = hashIdxLive;
	scene->localVBA.lastFreeBlockId = lastFreeVoxelBlockId;
	scene->index.lastFreeExcessListId = lastFreeExcessListId;
}

template<class TVoxel>
ITMSceneReconstructionEngine_CPU<TVoxel,ITMPlainVoxelArray>::ITMSceneReconstructionEngine_CPU(void) 
{}

template<class TVoxel>
ITMSceneReconstructionEngine_CPU<TVoxel,ITMPlainVoxelArray>::~ITMSceneReconstructionEngine_CPU(void) 
{}

template<class TVoxel>
void ITMSceneReconstructionEngine_CPU<TVoxel, ITMPlainVoxelArray>::AllocateSceneFromDepth(ITMScene<TVoxel, ITMPlainVoxelArray> *scene, const ITMView *view,
	const ITMTrackingState *trackingState, const ITMRenderState *renderState)
{}

template<class TVoxel>
void ITMSceneReconstructionEngine_CPU<TVoxel, ITMPlainVoxelArray>::IntegrateIntoScene(ITMScene<TVoxel, ITMPlainVoxelArray> *scene, const ITMView *view,
	const ITMTrackingState *trackingState, const ITMRenderState *renderState)
{
	Vector2i rgbImgSize = view->rgb->noDims;
	Vector2i depthImgSize = view->depth->noDims;
	float voxelSize = scene->sceneParams->voxelSize;

	Matrix4f M_d, M_rgb;
	Vector4f projParams_d, projParams_rgb;

	M_d = trackingState->pose_d->M;
	if (TVoxel::hasColorInformation) M_rgb = view->calib->trafo_rgb_to_depth.calib_inv * trackingState->pose_d->M;

	projParams_d = view->calib->intrinsics_d.projectionParamsSimple.all;
	projParams_rgb = view->calib->intrinsics_rgb.projectionParamsSimple.all;

	float mu = scene->sceneParams->mu; int maxW = scene->sceneParams->maxW;

	float *depth = view->depth->GetData(MEMORYDEVICE_CPU);
	Vector4u *rgb = view->rgb->GetData(MEMORYDEVICE_CPU);
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

template class ITMLib::Engine::ITMSceneReconstructionEngine_CPU<ITMVoxel, ITMVoxelIndex>;
