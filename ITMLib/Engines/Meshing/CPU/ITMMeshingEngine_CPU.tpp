// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#include "ITMMeshingEngine_CPU.h"

#include <algorithm>

#include "../Shared/ITMMeshingEngine_Shared.h"

using namespace ITMLib;

template<class TVoxel>
void ITMMeshingEngine_CPU<TVoxel, ITMVoxelBlockHash>::MeshScene(ITMMesh *mesh, const ITMScene<TVoxel, ITMVoxelBlockHash> *scene)
{
	ITMMesh::Triangle *triangles = mesh->triangles->GetData(MEMORYDEVICE_CPU);
	const TVoxel *localVBA = scene->localVBA.GetVoxelBlocks();
	const ITMHashEntry *hashTable = scene->index.GetEntries();

	int noTotalEntries = scene->index.noTotalEntries;
	uint noTriangles = 0;
	float factor = scene->sceneParams->voxelSize;

	mesh->triangles->Clear();

	for (int entryId = 0; entryId < noTotalEntries; entryId++)
	{
		Vector3i globalPos;
		const ITMHashEntry &currentHashEntry = hashTable[entryId];

		if (currentHashEntry.ptr < 0) continue;

		globalPos = currentHashEntry.pos.toInt() * SDF_BLOCK_SIZE;

		for (int z = 0; z < SDF_BLOCK_SIZE; z++) for (int y = 0; y < SDF_BLOCK_SIZE; y++) for (int x = 0; x < SDF_BLOCK_SIZE; x++)
		{
			Vector3f vertList[12];
			int cubeIndex = buildVertList(vertList, globalPos, Vector3i(x, y, z), localVBA, hashTable);
			
			if (cubeIndex < 0) continue;

			outputTriangles(triangles, &noTriangles, factor, mesh->noMaxTriangles, localVBA, hashTable, vertList, cubeIndex);

			// If we cannot add more triangles to the output list, exit all loops at once.
			if(noTriangles >= mesh->noMaxTriangles) goto done;
		}
	}

done:
	mesh->noTotalTriangles = std::min<uint>(noTriangles, mesh->noMaxTriangles);
}
