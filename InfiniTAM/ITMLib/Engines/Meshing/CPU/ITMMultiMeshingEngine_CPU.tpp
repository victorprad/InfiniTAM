// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#include "ITMMultiMeshingEngine_CPU.h"

#include "../Shared/ITMMultiMeshingEngine_Shared.h"

using namespace ITMLib;

template<class TVoxel>
inline void ITMMultiMeshingEngine_CPU<TVoxel, ITMVoxelBlockHash>::MeshScene(ITMMesh * mesh, const MultiSceneManager & sceneManager)
{
	int numScenes = (int)sceneManager.numScenes();
	if (numScenes > MAX_NUM_SCENES) numScenes = MAX_NUM_SCENES;

	MultiIndexData hashTables;
	MultiVoxelData localVBAs;

	const ITMSceneParams & sceneParams = *(sceneManager.getScene(0)->scene->sceneParams);
	hashTables.numScenes = numScenes;
	for (int sceneId = 0; sceneId < numScenes; ++sceneId)
	{
		hashTables.poses_vs[sceneId] = sceneManager.getEstimatedGlobalPose(sceneId).GetM();
		hashTables.poses_vs[sceneId].m30 /= sceneParams.voxelSize;
		hashTables.poses_vs[sceneId].m31 /= sceneParams.voxelSize;
		hashTables.poses_vs[sceneId].m32 /= sceneParams.voxelSize;
		
		hashTables.posesInv[sceneId] = sceneManager.getEstimatedGlobalPose(sceneId).GetInvM();
		hashTables.posesInv[sceneId].m30 /= sceneParams.voxelSize;
		hashTables.posesInv[sceneId].m31 /= sceneParams.voxelSize;
		hashTables.posesInv[sceneId].m32 /= sceneParams.voxelSize;

		hashTables.index[sceneId] = sceneManager.getScene(sceneId)->scene->index.getIndexData();
		localVBAs.voxels[sceneId] = sceneManager.getScene(sceneId)->scene->localVBA.GetVoxelBlocks();
	}

	ITMMesh::Triangle *triangles = mesh->triangles->GetData(MEMORYDEVICE_CPU);
	mesh->triangles->Clear();

	int noTriangles = 0, noMaxTriangles = mesh->noMaxTriangles, noTotalEntriesPerLocalMap = ITMVoxelBlockHash::noTotalEntries;
	float factor = sceneParams.voxelSize;

	// very dumb rendering -- likely to generate lots of duplicates
	for (int sceneId = 0; sceneId < numScenes; ++sceneId)
	{
		ITMHashEntry *hashTable = hashTables.index[sceneId];

		for (int entryId = 0; entryId < noTotalEntriesPerLocalMap; entryId++)
		{
			Vector3i globalPos;
			const ITMHashEntry &currentHashEntry = hashTable[entryId];

			if (currentHashEntry.ptr < 0) continue;

			globalPos = currentHashEntry.pos.toInt() * SDF_BLOCK_SIZE;

			for (int z = 0; z < SDF_BLOCK_SIZE; z++) for (int y = 0; y < SDF_BLOCK_SIZE; y++) for (int x = 0; x < SDF_BLOCK_SIZE; x++)
			{
				Vector3f vertList[12];
				int cubeIndex = buildVertListMulti(vertList, globalPos, Vector3i(x, y, z), &localVBAs, &hashTables, sceneId);

				if (cubeIndex < 0) continue;

				for (int i = 0; triangleTable[cubeIndex][i] != -1; i += 3)
				{
					triangles[noTriangles].p0 = vertList[triangleTable[cubeIndex][i]] * factor;
					triangles[noTriangles].p1 = vertList[triangleTable[cubeIndex][i + 1]] * factor;
					triangles[noTriangles].p2 = vertList[triangleTable[cubeIndex][i + 2]] * factor;

					if (noTriangles < noMaxTriangles - 1) noTriangles++;
				}
			}
		}
	}

	mesh->noTotalTriangles = noTriangles;
}