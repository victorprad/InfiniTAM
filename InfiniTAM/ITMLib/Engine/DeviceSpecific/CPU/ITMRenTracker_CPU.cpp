// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM

#include "ITMRenTracker_CPU.h"
#include "../../DeviceAgnostic/ITMRenTracker.h"
#include "../../DeviceAgnostic/ITMRepresentationAccess.h" 

using namespace ITMLib::Engine;


template<class TVoxel, class TIndex>
ITMLib::Engine::ITMRenTracker_CPU<TVoxel, TIndex>::ITMRenTracker_CPU(Vector2i imgSize, TrackerIterationType *trackingRegime, int noHierarchyLevels, const ITMLowLevelEngine *lowLevelEngine, const ITMScene<TVoxel, TIndex> *scene)
	: ITMRenTracker<TVoxel, TIndex>(imgSize, trackingRegime, noHierarchyLevels, lowLevelEngine, scene, MEMORYDEVICE_CPU){}

template<class TVoxel, class TIndex>
ITMRenTracker_CPU<TVoxel,TIndex>::~ITMRenTracker_CPU(void) { }

template<class TVoxel, class TIndex>
void ITMRenTracker_CPU<TVoxel,TIndex>::F_oneLevel(float *f, Matrix4f invM)
{
	int count = static_cast<int>(this->viewHierarchy->levels[this->levelId]->depth->dataSize);
	Vector4f *ptList = this->viewHierarchy->levels[this->levelId]->depth->GetData(MEMORYDEVICE_CPU);

	const TVoxel *voxelBlocks = this->scene->localVBA.GetVoxelBlocks();
	const typename TIndex::IndexData *index = this->scene->index.getIndexData();
	float oneOverVoxelSize = 1.0f / (float)this->scene->sceneParams->voxelSize;

	float energy = 0;

	for (int i = 0; i < count; i++)
	{
		Vector4f inpt = ptList[i];
		if (inpt.w > -1.0f) energy += computePerPixelEnergy<TVoxel,TIndex>(inpt, voxelBlocks, index, oneOverVoxelSize, invM);
	}

	f[0] = -energy;
}

template<class TVoxel, class TIndex>
void ITMRenTracker_CPU<TVoxel,TIndex>::G_oneLevel(float *gradient, float *hessian, Matrix4f invM) const
{
	int count = static_cast<int>(this->viewHierarchy->levels[this->levelId]->depth->dataSize);
	Vector4f *ptList = this->viewHierarchy->levels[this->levelId]->depth->GetData(MEMORYDEVICE_CPU);

	const TVoxel *voxelBlocks = this->scene->localVBA.GetVoxelBlocks();
	const typename TIndex::IndexData *index = this->scene->index.getIndexData();
	float oneOverVoxelSize = 1.0f / (float)this->scene->sceneParams->voxelSize;

	int noPara = 6, noParaSQ = 21;

	float globalGradient[6], globalHessian[21];
	for (int i = 0; i < noPara; i++) globalGradient[i] = 0.0f;
	for (int i = 0; i < noParaSQ; i++) globalHessian[i] = 0.0f;

	for (int i = 0; i < count; i++)
	{
		Vector4f cPt = ptList[i];
		if (cPt.w == -1.0f) continue;

		float jacobian[6];

		if (computePerPixelJacobian<TVoxel,TIndex>(jacobian, cPt, voxelBlocks, index, oneOverVoxelSize, invM))
		{
			for (int r = 0, counter = 0; r < noPara; r++)
			{
				globalGradient[r] -= jacobian[r];
				for (int c = 0; c <= r; c++, counter++) globalHessian[counter] += jacobian[r] * jacobian[c];
			}
		}
	}

	for (int r = 0, counter = 0; r < noPara; r++) for (int c = 0; c <= r; c++, counter++) hessian[r + c * 6] = globalHessian[counter];
	for (int r = 0; r < noPara; ++r) for (int c = r + 1; c < noPara; c++) hessian[r + c * 6] = hessian[c + r * 6];
	for (int r = 0; r < noPara; ++r) gradient[r] = globalGradient[r];
}

template<class TVoxel, class TIndex>
void ITMRenTracker_CPU<TVoxel,TIndex>::UnprojectDepthToCam(ITMFloatImage *depth, ITMFloat4Image *upPtCloud, const Vector4f &intrinsic)
{
	float *depthMap = depth->GetData(MEMORYDEVICE_CPU);
	Vector4f *camPoints = upPtCloud->GetData(MEMORYDEVICE_CPU);

	Vector4f ooIntrinsics;
	ooIntrinsics.x = 1.0f / intrinsic.x; ooIntrinsics.y = 1.0f / intrinsic.y;
	ooIntrinsics.z = -intrinsic.z * ooIntrinsics.x; ooIntrinsics.w = -intrinsic.w * ooIntrinsics.y;

	Vector2i imgSize = depth->noDims;
	upPtCloud->noDims = imgSize; upPtCloud->dataSize = depth->dataSize;

	for (int y = 0; y < imgSize.y; y++) for (int x = 0; x < imgSize.x; x++)
	{
		Vector3f inpt; int locId = x + y * imgSize.x;

		inpt.z = depthMap[locId];

		if (inpt.z > 0.0f)
		{
			inpt.x = x * inpt.z; inpt.y = y * inpt.z;
			unprojectPtWithIntrinsic(ooIntrinsics, inpt, camPoints[locId]);
		}
		else camPoints[locId] = Vector4f(0.0f, 0.0f, 0.0f, -1.0f);
	}
}

template class ITMLib::Engine::ITMRenTracker_CPU<ITMVoxel, ITMVoxelIndex>;
