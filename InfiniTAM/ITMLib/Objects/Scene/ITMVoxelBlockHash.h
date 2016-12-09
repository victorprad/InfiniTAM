// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#ifndef __METALC__
#include <stdlib.h>
#include <fstream>
#include <iostream>
#endif

#include "../../Utils/ITMMath.h"
#include "../../../ORUtils/MemoryBlock.h"
#include "../../../ORUtils/MemoryBlockPersister.h"

#define SDF_BLOCK_SIZE 8				// SDF block size
#define SDF_BLOCK_SIZE3 512				// SDF_BLOCK_SIZE3 = SDF_BLOCK_SIZE * SDF_BLOCK_SIZE * SDF_BLOCK_SIZE

#define SDF_LOCAL_BLOCK_NUM 0x40000		// Number of locally stored blocks, currently 2^17

#define SDF_BUCKET_NUM 0x100000			// Number of Hash Bucket, should be 2^n and bigger than SDF_LOCAL_BLOCK_NUM, SDF_HASH_MASK = SDF_BUCKET_NUM - 1
#define SDF_HASH_MASK 0xfffff			// Used for get hashing value of the bucket index,  SDF_HASH_MASK = SDF_BUCKET_NUM - 1
#define SDF_EXCESS_LIST_SIZE 0x20000	// 0x20000 Size of excess list, used to handle collisions. Also max offset (unsigned short) value.

//// for loop closure
//#define SDF_LOCAL_BLOCK_NUM 0x10000		// Number of locally stored blocks, currently 2^12
//
//#define SDF_BUCKET_NUM 0x40000			// Number of Hash Bucket, should be 2^n and bigger than SDF_LOCAL_BLOCK_NUM, SDF_HASH_MASK = SDF_BUCKET_NUM - 1
//#define SDF_HASH_MASK 0x3ffff			// Used for get hashing value of the bucket index,  SDF_HASH_MASK = SDF_BUCKET_NUM - 1
//#define SDF_EXCESS_LIST_SIZE 0x8000		// 0x8000 Size of excess list, used to handle collisions. Also max offset (unsigned short) value.

#define SDF_TRANSFER_BLOCK_NUM 0x1000	// Maximum number of blocks transfered in one swap operation

/** \brief
	A single entry in the hash table.
*/
struct ITMHashEntry
{
	/** Position of the corner of the 8x8x8 volume, that identifies the entry. */
	Vector3s pos;
	/** Offset in the excess list. */
	int offset;
	/** Pointer to the voxel block array.
		- >= 0 identifies an actual allocated entry in the voxel block array
		- -1 identifies an entry that has been removed (swapped out)
		- <-1 identifies an unallocated block
	*/
	int ptr;
};

namespace ITMLib
{
	/** \brief
	This is the central class for the voxel block hash
	implementation. It contains all the data needed on the CPU
	and a pointer to the data structure on the GPU.
	*/
	class ITMVoxelBlockHash
	{
	public:
		typedef ITMHashEntry IndexData;

		struct IndexCache {
			Vector3i blockPos;
			int blockPtr;
			_CPU_AND_GPU_CODE_ IndexCache(void) : blockPos(0x7fffffff), blockPtr(-1) {}
		};

		/** Maximum number of total entries. */
		static const CONSTPTR(int) noTotalEntries = SDF_BUCKET_NUM + SDF_EXCESS_LIST_SIZE;
		static const CONSTPTR(int) voxelBlockSize = SDF_BLOCK_SIZE * SDF_BLOCK_SIZE * SDF_BLOCK_SIZE;

#ifndef __METALC__
	private:
		int lastFreeExcessListId;

		/** The actual data in the hash table. */
		ORUtils::MemoryBlock<ITMHashEntry> *hashEntries;

		/** Identifies which entries of the overflow
		list are allocated. This is used if too
		many hash collisions caused the buckets to
		overflow.
		*/
		ORUtils::MemoryBlock<int> *excessAllocationList;

		MemoryDeviceType memoryType;

	public:
		ITMVoxelBlockHash(MemoryDeviceType memoryType)
		{
			this->memoryType = memoryType;
			hashEntries = new ORUtils::MemoryBlock<ITMHashEntry>(noTotalEntries, memoryType);
			excessAllocationList = new ORUtils::MemoryBlock<int>(SDF_EXCESS_LIST_SIZE, memoryType);
		}

		~ITMVoxelBlockHash(void)
		{
			delete hashEntries;
			delete excessAllocationList;
		}

		/** Get the list of actual entries in the hash table. */
		const ITMHashEntry *GetEntries(void) const { return hashEntries->GetData(memoryType); }
		ITMHashEntry *GetEntries(void) { return hashEntries->GetData(memoryType); }

		const IndexData *getIndexData(void) const { return hashEntries->GetData(memoryType); }
		IndexData *getIndexData(void) { return hashEntries->GetData(memoryType); }

		/** Get the list that identifies which entries of the
		overflow list are allocated. This is used if too
		many hash collisions caused the buckets to overflow.
		*/
		const int *GetExcessAllocationList(void) const { return excessAllocationList->GetData(memoryType); }
		int *GetExcessAllocationList(void) { return excessAllocationList->GetData(memoryType); }

		int GetLastFreeExcessListId(void) { return lastFreeExcessListId; }
		void SetLastFreeExcessListId(int lastFreeExcessListId) { this->lastFreeExcessListId = lastFreeExcessListId; }

#ifdef COMPILE_WITH_METAL
		const void* GetEntries_MB(void) { return hashEntries->GetMetalBuffer(); }
		const void* GetExcessAllocationList_MB(void) { return excessAllocationList->GetMetalBuffer(); }
		const void* getIndexData_MB(void) const { return hashEntries->GetMetalBuffer(); }
#endif

		/** Maximum number of total entries. */
		int getNumAllocatedVoxelBlocks(void) { return SDF_LOCAL_BLOCK_NUM; }
		int getVoxelBlockSize(void) { return SDF_BLOCK_SIZE3; }

		void SaveToDirectory(const std::string &outputDirectory) const
		{
			std::string hashEntriesFileName = outputDirectory + "hash.dat";
			std::string excessAllocationListFileName = outputDirectory + "excess.dat";
			std::string lastFreeExcessListIdFileName = outputDirectory + "last.txt";

			std::ofstream ofs(lastFreeExcessListIdFileName.c_str());
			if (!ofs) throw std::runtime_error("Could not open " + lastFreeExcessListIdFileName + " for writing");

			ofs << lastFreeExcessListId;
			ORUtils::MemoryBlockPersister::SaveMemoryBlock(hashEntriesFileName, *hashEntries, memoryType);
			ORUtils::MemoryBlockPersister::SaveMemoryBlock(excessAllocationListFileName, *excessAllocationList, memoryType);
		}

		void LoadFromDirectory(const std::string &inputDirectory)
		{
			std::string hashEntriesFileName = inputDirectory + "hash.dat";
			std::string excessAllocationListFileName = inputDirectory + "excess.dat";
			std::string lastFreeExcessListIdFileName = inputDirectory + "last.txt";

			std::ifstream ifs(lastFreeExcessListIdFileName.c_str());
			if (!ifs) throw std::runtime_error("Count not open " + lastFreeExcessListIdFileName + " for reading");

			ifs >> this->lastFreeExcessListId;
			ORUtils::MemoryBlockPersister::LoadMemoryBlock(hashEntriesFileName.c_str(), *hashEntries, memoryType);
			ORUtils::MemoryBlockPersister::LoadMemoryBlock(excessAllocationListFileName.c_str(), *excessAllocationList, memoryType);
		}

		// Suppress the default copy constructor and assignment operator
		ITMVoxelBlockHash(const ITMVoxelBlockHash&);
		ITMVoxelBlockHash& operator=(const ITMVoxelBlockHash&);
#endif
	};
}
