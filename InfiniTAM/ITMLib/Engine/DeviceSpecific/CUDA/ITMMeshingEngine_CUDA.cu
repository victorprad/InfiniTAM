// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM

#include "ITMMeshingEngine_CUDA.h"
#include "../../DeviceAgnostic/ITMMeshingEngine.h"
#include "ITMCUDAUtils.h"

#include "../../../../ORUtils/CUDADefines.h"

template<class TVoxel>
__global__ void meshScene_device(ITMMesh::Triangle *triangles, unsigned int *noTriangles_device, float factor, int noTotalEntries,
	int noMaxTriangles, const Vector4s *visibleBlockGlobalPos, const TVoxel *localVBA, const ITMHashEntry *hashTable);

__global__ void findAllocateBlocks(Vector4s *visibleBlockGlobalPos, const ITMHashEntry *hashTable, int noTotalEntries);

using namespace ITMLib::Engine;

template<class TVoxel>
ITMMeshingEngine_CUDA<TVoxel,ITMVoxelBlockHash>::ITMMeshingEngine_CUDA(void) 
{
	ITMSafeCall(cudaMalloc((void**)&visibleBlockGlobalPos_device, SDF_LOCAL_BLOCK_NUM * sizeof(Vector4s)));
	ITMSafeCall(cudaMalloc((void**)&noTriangles_device, sizeof(unsigned int)));
}

template<class TVoxel>
ITMMeshingEngine_CUDA<TVoxel,ITMVoxelBlockHash>::~ITMMeshingEngine_CUDA(void) 
{
	ITMSafeCall(cudaFree(visibleBlockGlobalPos_device));
	ITMSafeCall(cudaFree(noTriangles_device));
}

template<class TVoxel>
void ITMMeshingEngine_CUDA<TVoxel, ITMVoxelBlockHash>::MeshScene(ITMMesh *mesh, const ITMScene<TVoxel, ITMVoxelBlockHash> *scene)
{
	ITMMesh::Triangle *triangles = mesh->triangles->GetData(MEMORYDEVICE_CUDA);
	const TVoxel *localVBA = scene->localVBA.GetVoxelBlocks();
	const ITMHashEntry *hashTable = scene->index.GetEntries();

	int noMaxTriangles = mesh->noMaxTriangles, noTotalEntries = scene->index.noTotalEntries;
	float factor = scene->sceneParams->voxelSize;

	ITMSafeCall(cudaMemset(noTriangles_device, 0, sizeof(unsigned int)));
	ITMSafeCall(cudaMemset(visibleBlockGlobalPos_device, 0, sizeof(Vector4s) * SDF_LOCAL_BLOCK_NUM));

	{ // identify used voxel blocks
		dim3 cudaBlockSize(256); 
		dim3 gridSize((int)ceil((float)noTotalEntries / (float)cudaBlockSize.x));

		findAllocateBlocks << <gridSize, cudaBlockSize >> >(visibleBlockGlobalPos_device, hashTable, noTotalEntries);
	}

	{ // mesh used voxel blocks
		dim3 cudaBlockSize(SDF_BLOCK_SIZE, SDF_BLOCK_SIZE, SDF_BLOCK_SIZE);
		dim3 gridSize(SDF_LOCAL_BLOCK_NUM / 16, 16);

		meshScene_device<TVoxel> << <gridSize, cudaBlockSize >> >(triangles, noTriangles_device, factor, noTotalEntries, noMaxTriangles,
			visibleBlockGlobalPos_device, localVBA, hashTable);

		ITMSafeCall(cudaMemcpy(&mesh->noTotalTriangles, noTriangles_device, sizeof(unsigned int), cudaMemcpyDeviceToHost));
	}
}

template<class TVoxel>
ITMMeshingEngine_CUDA<TVoxel,ITMPlainVoxelArray>::ITMMeshingEngine_CUDA(void) 
{}

template<class TVoxel>
ITMMeshingEngine_CUDA<TVoxel,ITMPlainVoxelArray>::~ITMMeshingEngine_CUDA(void) 
{}

template<class TVoxel>
void ITMMeshingEngine_CUDA<TVoxel, ITMPlainVoxelArray>::MeshScene(ITMMesh *mesh, const ITMScene<TVoxel, ITMPlainVoxelArray> *scene)
{}

__global__ void findAllocateBlocks(Vector4s *visibleBlockGlobalPos, const ITMHashEntry *hashTable, int noTotalEntries)
{
	int entryId = threadIdx.x + blockIdx.x * blockDim.x;
	if (entryId > noTotalEntries - 1) return;

	const ITMHashEntry &currentHashEntry = hashTable[entryId];

	if (currentHashEntry.ptr >= 0) 
		visibleBlockGlobalPos[currentHashEntry.ptr] = Vector4s(currentHashEntry.pos.x, currentHashEntry.pos.y, currentHashEntry.pos.z, 1);
}

template<class TVoxel>
__global__ void meshScene_device(ITMMesh::Triangle *triangles, unsigned int *noTriangles_device, float factor, int noTotalEntries, 
	int noMaxTriangles, const Vector4s *visibleBlockGlobalPos, const TVoxel *localVBA, const ITMHashEntry *hashTable)
{
	const Vector4s globalPos_4s = visibleBlockGlobalPos[blockIdx.x + gridDim.x * blockIdx.y];

	if (globalPos_4s.w == 0) return;

	Vector3i globalPos = Vector3i(globalPos_4s.x, globalPos_4s.y, globalPos_4s.z) * SDF_BLOCK_SIZE;

	Vector3f vertList[12];
	int cubeIndex = buildVertList(vertList, globalPos, Vector3i(threadIdx.x, threadIdx.y, threadIdx.z), localVBA, hashTable);

	if (cubeIndex < 0) return;

	for (int i = 0; triangleTable[cubeIndex][i] != -1; i += 3)
	{
		int triangleId = atomicAdd(noTriangles_device, 1);

		if (triangleId < noMaxTriangles - 1)
		{
			triangles[triangleId].p0 = vertList[triangleTable[cubeIndex][i]] * factor;
			triangles[triangleId].p1 = vertList[triangleTable[cubeIndex][i + 1]] * factor;
			triangles[triangleId].p2 = vertList[triangleTable[cubeIndex][i + 2]] * factor;
		}
	}
}

template class ITMLib::Engine::ITMMeshingEngine_CUDA<ITMVoxel, ITMVoxelIndex>;
