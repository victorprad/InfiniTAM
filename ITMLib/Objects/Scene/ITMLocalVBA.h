// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../../../ORUtils/MemoryBlock.h"
#include "../../../ORUtils/MemoryBlockPersister.h"

namespace ITMLib
{
	/** \brief
	Stores the actual voxel content that is referred to by a
	ITMLib::ITMHashTable.
	*/
	template<class TVoxel>
	class ITMLocalVBA
	{
	private:
		ORUtils::MemoryBlock<TVoxel> *voxelBlocks;
		ORUtils::MemoryBlock<int> *allocationList;

		MemoryDeviceType memoryType;

	public:
		inline TVoxel *GetVoxelBlocks(void) { return voxelBlocks->GetData(memoryType); }
		inline const TVoxel *GetVoxelBlocks(void) const { return voxelBlocks->GetData(memoryType); }
		int *GetAllocationList(void) { return allocationList->GetData(memoryType); }

#ifdef COMPILE_WITH_METAL
		const void* GetVoxelBlocks_MB() const { return voxelBlocks->GetMetalBuffer(); }
		const void* GetAllocationList_MB(void) const { return allocationList->GetMetalBuffer(); }
#endif
		int lastFreeBlockId;

		int allocatedSize;

		void SaveToDirectory(const std::string &outputDirectory) const
		{
			std::string VBFileName = outputDirectory + "voxel.dat";
			std::string ALFileName = outputDirectory + "alloc.dat";
			std::string AllocSizeFileName = outputDirectory + "vba.txt";

			ORUtils::MemoryBlockPersister::SaveMemoryBlock(VBFileName, *voxelBlocks, memoryType);
			ORUtils::MemoryBlockPersister::SaveMemoryBlock(ALFileName, *allocationList, memoryType);

			std::ofstream ofs(AllocSizeFileName.c_str());
			if (!ofs) throw std::runtime_error("Could not open " + AllocSizeFileName + " for writing");

			ofs << lastFreeBlockId << ' ' << allocatedSize;
		}

		void LoadFromDirectory(const std::string &inputDirectory)
		{
			std::string VBFileName = inputDirectory + "voxel.dat";
			std::string ALFileName = inputDirectory + "alloc.dat";
			std::string AllocSizeFileName = inputDirectory + "vba.txt";

			ORUtils::MemoryBlockPersister::LoadMemoryBlock(VBFileName, *voxelBlocks, memoryType);
			ORUtils::MemoryBlockPersister::LoadMemoryBlock(ALFileName, *allocationList, memoryType);

			std::ifstream ifs(AllocSizeFileName.c_str());
			if (!ifs) throw std::runtime_error("Could not open " + AllocSizeFileName + " for reading");

			ifs >> lastFreeBlockId >> allocatedSize;
		}

		ITMLocalVBA(MemoryDeviceType memoryType, int noBlocks, int blockSize)
		{
			this->memoryType = memoryType;

			allocatedSize = noBlocks * blockSize;

			voxelBlocks = new ORUtils::MemoryBlock<TVoxel>(allocatedSize, memoryType);
			allocationList = new ORUtils::MemoryBlock<int>(noBlocks, memoryType);
		}

		~ITMLocalVBA(void)
		{
			delete voxelBlocks;
			delete allocationList;
		}

		// Suppress the default copy constructor and assignment operator
		ITMLocalVBA(const ITMLocalVBA&);
		ITMLocalVBA& operator=(const ITMLocalVBA&);
	};
}
