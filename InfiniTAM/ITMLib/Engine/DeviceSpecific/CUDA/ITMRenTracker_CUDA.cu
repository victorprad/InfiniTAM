// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM

#include "ITMRenTracker_CUDA.h"
#include "ITMCUDAUtils.h"

#include "../../../../ORUtils/CUDADefines.h"
#include "../../DeviceAgnostic/ITMRenTracker.h"
#include "../../DeviceAgnostic/ITMRepresentationAccess.h" 

using namespace ITMLib::Engine;

__global__ void unprojectDepthToCam_device(Vector4f *camPoints, float *depthMap, Vector2i imgSize, Vector4f ooIntrinsics);

template<class TVoxel, class TIndex>
__global__ void renTrackerOneLevel_f_device(float *f_device, Vector4f *ptList, int count, const TVoxel *voxelBlocks,
	const typename TIndex::IndexData *index, float oneOverVoxelSize, Matrix4f invM);

template<class TVoxel, class TIndex>
__global__ void renTrackerOneLevel_g_device(float *g_device, float *h_device, Vector4f *ptList, int count, const TVoxel *voxelBlocks,
	const typename TIndex::IndexData *index, float oneOverVoxelSize, Matrix4f invM);

// host functions

template<class TVoxel, class TIndex>
ITMRenTracker_CUDA<TVoxel, TIndex>::ITMRenTracker_CUDA(Vector2i imgSize, TrackerIterationType *trackingRegime, int noHierarchyLevels, const ITMLowLevelEngine *lowLevelEngine, const ITMScene<TVoxel, TIndex> *scene)
	: ITMRenTracker<TVoxel, TIndex>(imgSize,trackingRegime,noHierarchyLevels,lowLevelEngine,scene, MEMORYDEVICE_CUDA)
{ 
	int dim_f = 1, dim_g = 6, dim_h = 6 + 5 + 4 + 3 + 2 + 1;

	Vector2i gridSize((imgSize.x + 15) / 16, (imgSize.y + 15) / 16);

	f_host = new float[dim_f * gridSize.x * gridSize.y];
	g_host = new float[dim_g * gridSize.x * gridSize.y];
	h_host = new float[dim_h * gridSize.x * gridSize.y];

	ITMSafeCall(cudaMalloc((void**)&f_device, sizeof(float)* dim_f * gridSize.x * gridSize.y));
	ITMSafeCall(cudaMalloc((void**)&g_device, sizeof(float)* dim_g * gridSize.x * gridSize.y));
	ITMSafeCall(cudaMalloc((void**)&h_device, sizeof(float)* dim_h * gridSize.x * gridSize.y));
}

template<class TVoxel, class TIndex>
ITMRenTracker_CUDA<TVoxel,TIndex>::~ITMRenTracker_CUDA(void) 
{ 
	delete[] f_host;
	delete[] g_host;
	delete[] h_host;

	ITMSafeCall(cudaFree(f_device));
	ITMSafeCall(cudaFree(g_device));
	ITMSafeCall(cudaFree(h_device));
}

template<class TVoxel, class TIndex>
void ITMRenTracker_CUDA<TVoxel,TIndex>::F_oneLevel(float *f, Matrix4f invM)
{
	int count = static_cast<int>(this->viewHierarchy->levels[this->levelId]->depth->dataSize);

	dim3 blockSize(256, 1);
	dim3 gridSize((int)ceil((float)count / (float)blockSize.x), 1);

	const TVoxel *voxelBlocks = this->scene->localVBA.GetVoxelBlocks();
	const typename TIndex::IndexData *index = this->scene->index.getIndexData();
	float oneOverVoxelSize = 1.0f / (float)this->scene->sceneParams->voxelSize;

	ITMSafeCall(cudaMemset(f_device, 0, sizeof(float) * gridSize.x));

	renTrackerOneLevel_f_device<TVoxel,TIndex> << <gridSize, blockSize >> >(f_device, this->viewHierarchy->levels[this->levelId]->depth->GetData(MEMORYDEVICE_CUDA), 
		count, voxelBlocks, index, oneOverVoxelSize, invM);

	ITMSafeCall(cudaMemcpy(f_host, f_device, sizeof(float)* gridSize.x, cudaMemcpyDeviceToHost));

	float energy = 0;
	for (size_t i = 0; i < gridSize.x; i++) energy += f_host[i];

	f[0] = -energy;
}

template<class TVoxel, class TIndex>
void ITMRenTracker_CUDA<TVoxel,TIndex>::G_oneLevel(float *gradient, float *hessian, Matrix4f invM) const
{
	int count = static_cast<int>(this->viewHierarchy->levels[this->levelId]->depth->dataSize);
	Vector4f *ptList = this->viewHierarchy->levels[this->levelId]->depth->GetData(MEMORYDEVICE_CUDA);

	const TVoxel *voxelBlocks = this->scene->localVBA.GetVoxelBlocks();
	const typename TIndex::IndexData *index = this->scene->index.getIndexData();
	float oneOverVoxelSize = 1.0f / (float)this->scene->sceneParams->voxelSize;

	int noPara = 6, noParaSQ = 21;

	float globalGradient[6], globalHessian[21];
	for (int i = 0; i < noPara; i++) globalGradient[i] = 0.0f;
	for (int i = 0; i < noParaSQ; i++) globalHessian[i] = 0.0f;

	dim3 blockSize(256, 1);
	dim3 gridSize((int)ceil((float)count / (float)blockSize.x), 1);

	ITMSafeCall(cudaMemset(g_device, 0, sizeof(float) * gridSize.x * noPara));
	ITMSafeCall(cudaMemset(h_device, 0, sizeof(float) * gridSize.x * noParaSQ));

	renTrackerOneLevel_g_device<TVoxel,TIndex> << <gridSize, blockSize >> >(g_device, h_device, ptList, count, voxelBlocks, index, oneOverVoxelSize, invM);

	ITMSafeCall(cudaMemcpy(g_host, g_device, sizeof(float)* gridSize.x * noPara, cudaMemcpyDeviceToHost));
	ITMSafeCall(cudaMemcpy(h_host, h_device, sizeof(float)* gridSize.x * noParaSQ, cudaMemcpyDeviceToHost));

	for (size_t i = 0; i < gridSize.x; i++)
	{
		for (int p = 0; p < noPara; p++) globalGradient[p] += g_host[i * noPara + p];
		for (int p = 0; p < noParaSQ; p++) globalHessian[p] += h_host[i * noParaSQ + p];
	}

	for (int r = 0, counter = 0; r < noPara; r++) for (int c = 0; c <= r; c++, counter++) hessian[r + c * 6] = globalHessian[counter];
	for (int r = 0; r < noPara; ++r) for (int c = r + 1; c < noPara; c++) hessian[r + c * 6] = hessian[c + r * 6];
	for (int r = 0; r < noPara; ++r) gradient[r] = -globalGradient[r];

	//for (int i = 0; i < noPara; i++) printf("%f ", gradient[i]);
	//printf("\n");

	//for (int i = 0; i < 6 * 6; i++) printf("%f ", hessian[i]);
	//printf("\n");

	//getchar();
}

template<class TVoxel, class TIndex>
void ITMRenTracker_CUDA<TVoxel,TIndex>::UnprojectDepthToCam(ITMFloatImage *depth, ITMFloat4Image *upPtCloud, const Vector4f &intrinsic)
{
	float *depthMap = depth->GetData(MEMORYDEVICE_CUDA);
	Vector4f *camPoints = upPtCloud->GetData(MEMORYDEVICE_CUDA);

	Vector4f ooIntrinsics;
	ooIntrinsics.x = 1.0f / intrinsic.x; ooIntrinsics.y = 1.0f / intrinsic.y;
	ooIntrinsics.z = -intrinsic.z * ooIntrinsics.x; ooIntrinsics.w = -intrinsic.w * ooIntrinsics.y;

	Vector2i imgSize = depth->noDims;
	upPtCloud->noDims = imgSize; upPtCloud->dataSize = depth->dataSize;

	dim3 blockSize(16, 16);
	dim3 gridSize((int)ceil((float)imgSize.x / (float)blockSize.x), (int)ceil((float)imgSize.y / (float)blockSize.y));

	unprojectDepthToCam_device << <gridSize, blockSize >> >(camPoints, depthMap, imgSize, ooIntrinsics);
}

// device functions

__global__ void unprojectDepthToCam_device(Vector4f *camPoints, float *depthMap, Vector2i imgSize, Vector4f ooIntrinsics)
{
	int x = threadIdx.x + blockIdx.x * blockDim.x, y = threadIdx.y + blockIdx.y * blockDim.y;

	if (x < imgSize.x && y < imgSize.y)
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

template<class TVoxel, class TIndex>
__global__ void renTrackerOneLevel_f_device(float *f_device, Vector4f *ptList, int count, const TVoxel *voxelBlocks, 
	const typename TIndex::IndexData *index, float oneOverVoxelSize, Matrix4f invM)
{
	int locId_global = threadIdx.x + blockIdx.x * blockDim.x, locId_local = threadIdx.x;

	__shared__ float dim_shared[256];

	dim_shared[locId_local] = 0.0f;

	if (locId_global < count)
	{
		Vector4f inpt = ptList[locId_global];
		if (inpt.w > -1.0f) dim_shared[locId_local] = computePerPixelEnergy<TVoxel,TIndex>(inpt, voxelBlocks, index, oneOverVoxelSize, invM);
	}
	
	{ //reduction for f_device
		__syncthreads();

		if (locId_local < 128) dim_shared[locId_local] += dim_shared[locId_local + 128];
		__syncthreads();
		if (locId_local < 64) dim_shared[locId_local] += dim_shared[locId_local + 64];
		__syncthreads();

		if (locId_local < 32) warpReduce(dim_shared, locId_local);

		if (locId_local == 0) f_device[blockIdx.x] = dim_shared[locId_local];
	}
}

template<class TVoxel, class TIndex>
__global__ void renTrackerOneLevel_g_device(float *g_device, float *h_device, Vector4f *ptList, int count, const TVoxel *voxelBlocks,
	const typename TIndex::IndexData *index, float oneOverVoxelSize, Matrix4f invM)
{
	int locId_global = threadIdx.x + blockIdx.x * blockDim.x, locId_local = threadIdx.x;

	__shared__ float dim_shared[256];
	__shared__ bool shouldAdd; bool hasValidData = false;

	float localGradient[6], localHessian[21];

	int noPara = 6, noParaSQ = 6 + 5 + 4 + 3 + 2 + 1;

	shouldAdd = false;
	__syncthreads();

	if (locId_global < count)
	{
		Vector4f cPt = ptList[locId_global];
		if (cPt.w > -1.0f)
		{
			if (computePerPixelJacobian<TVoxel,TIndex>(localGradient, cPt, voxelBlocks, index, oneOverVoxelSize, invM))
			{
				shouldAdd = true; hasValidData = true;
				for (int r = 0, counter = 0; r < noPara; r++) for (int c = 0; c <= r; c++, counter++)
					localHessian[counter] = localGradient[r] * localGradient[c];
			}
		}
	}

	__syncthreads();

	if (!hasValidData && shouldAdd)
	{
		for (int i = 0; i < noPara; i++) localGradient[i] = 0.0f;
		for (int i = 0; i < noParaSQ; i++) localHessian[i] = 0.0f;
	}

	if (shouldAdd)
	{
		//reduction for gradient
		for (int paraId = 0; paraId < noPara; paraId++)
		{
			dim_shared[locId_local] = localGradient[paraId];
			__syncthreads();

			if (locId_local < 128) dim_shared[locId_local] += dim_shared[locId_local + 128];
			__syncthreads();
			if (locId_local < 64) dim_shared[locId_local] += dim_shared[locId_local + 64];
			__syncthreads();

			if (locId_local < 32) warpReduce(dim_shared, locId_local);

			if (locId_local == 0) g_device[blockIdx.x * noPara + paraId] = dim_shared[locId_local];
		}

		__syncthreads();

		//reduction for hessian
		for (int paraId = 0; paraId < noParaSQ; paraId++)
		{
			dim_shared[locId_local] = localHessian[paraId];
			__syncthreads();

			if (locId_local < 128) dim_shared[locId_local] += dim_shared[locId_local + 128];
			__syncthreads();
			if (locId_local < 64) dim_shared[locId_local] += dim_shared[locId_local + 64];
			__syncthreads();

			if (locId_local < 32) warpReduce(dim_shared, locId_local);

			if (locId_local == 0) h_device[blockIdx.x * noParaSQ + paraId] = dim_shared[locId_local];
		}
	}

	//for (int paraId = 0; paraId < noPara; paraId++)
	//{
	//	atomicAdd(&g_device[blockIdx.x * noPara + paraId], localGradient[paraId]);
	//}

	//for (int paraId = 0; paraId < noParaSQ; paraId++)
	//{
	//	atomicAdd(&h_device[blockIdx.x * noParaSQ + paraId], localHessian[paraId]);
	//}
}

template class ITMLib::Engine::ITMRenTracker_CUDA<ITMVoxel, ITMVoxelIndex>;
