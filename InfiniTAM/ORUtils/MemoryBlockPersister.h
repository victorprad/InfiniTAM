// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM

#pragma once

#include <fstream>
#include <string>

#include "MemoryBlock.h"

namespace ORUtils
{

/**
 * \brief This class provides functions for loading and saving memory blocks.
 */
class MemoryBlockPersister
{
  //#################### PUBLIC STATIC MEMBER FUNCTIONS ####################
public:
  /**
   * \brief Loads data from a file on disk into a memory block.
   *
   * \param filename          The name of the file.
   * \param block             The memory block into which to load the data.
   * \param memoryDeviceType  The type of memory device on which to load the data.
   */
  template <typename T>
  static void LoadMemoryBlock(const std::string& filename, ORUtils::MemoryBlock<T>& block, MemoryDeviceType memoryDeviceType)
  {
    int blockSize = ReadBlockSize(filename);
    if(memoryDeviceType == MEMORYDEVICE_CUDA)
    {
      // If we're loading into a block on the GPU, first try and read the data into a temporary block on the CPU.
      ORUtils::MemoryBlock<T> cpuBlock(block.dataSize, MEMORYDEVICE_CPU);
      ReadBlockData(filename, cpuBlock, blockSize);

      // Then copy the data across to the GPU.
      block.SetFrom(&cpuBlock, ORUtils::MemoryBlock<T>::CPU_TO_CUDA);
    }
    else
    {
      // If we're loading into a block on the CPU, read the data directly into the block.
      ReadBlockData(filename, block, blockSize);
    }
  }

  /**
   * \brief Loads data from a file on disk into a memory block newly-allocated on the CPU with the appropriate size.
   *
   * \param filename  The name of the file.
   * \param dummy     An optional dummy parameter that can be used for type inference.
   * \return          The loaded memory block.
   */
  template <typename T>
  static ORUtils::MemoryBlock<T> *LoadMemoryBlock(const std::string& filename, ORUtils::MemoryBlock<T> *dummy = NULL)
  {
    int blockSize = ReadBlockSize(filename);
    ORUtils::MemoryBlock<T> *block = new ORUtils::MemoryBlock<T>(blockSize, MEMORYDEVICE_CPU);
    ReadBlockData(filename, *block, blockSize);
    return block;
  }

  /**
   * \brief Attempts to read the size of a memory block from a file containing data for a single block.
   *
   * The size is stored as a single integer and precedes the data for the block.
   *
   * \param filename            The name of the file.
   * \return                    The size of the memory block in the file.
   * \throws std::runtime_error If the read is unsuccessful.
   */
  static int ReadBlockSize(const std::string& filename)
  {
    std::ifstream fs(filename.c_str(), std::ios::binary);
    if(!fs) throw std::runtime_error("Could not open " + filename + " for reading");
    return ReadBlockSize(fs);
  }

  /**
   * \brief Saves a memory block to a file on disk.
   *
   * \param filename          The name of the file.
   * \param block             The memory block to save.
   * \param memoryDeviceType  The type of memory device from which to save the data.
   */
  template <typename T>
  static void SaveMemoryBlock(const std::string& filename, const ORUtils::MemoryBlock<T>& block, MemoryDeviceType memoryDeviceType)
  {
    std::ofstream fs(filename.c_str(), std::ios::binary);
    if(!fs) throw std::runtime_error("Could not open " + filename + " for writing");

    if(memoryDeviceType == MEMORYDEVICE_CUDA)
    {
      // If we are saving the memory block from the GPU, first make a CPU copy of it.
      ORUtils::MemoryBlock<T> cpuBlock(block.dataSize, MEMORYDEVICE_CPU);
      cpuBlock.SetFrom(&block, ORUtils::MemoryBlock<T>::CUDA_TO_CPU);

      // Then write the CPU copy to disk.
      WriteBlock(fs, cpuBlock);
    }
    else
    {
      // If we are saving the memory block from the CPU, write it directly to disk.
      WriteBlock(fs, block);
    }
  }

  //#################### PRIVATE STATIC MEMBER FUNCTIONS ####################
private:
  /**
   * \brief Attempts to read data into a memory block allocated on the CPU from an input stream.
   *
   * The memory block must have the specified size (which should have been obtained by a call to ReadBlockSize).
   *
   * \param is                  The input stream.
   * \param block               The memory block into which to read.
   * \param blockSize           The required size for the memory block.
   * \throws std::runtime_error If the read is unsuccessful.
   */
  template <typename T>
  static void ReadBlockData(std::istream& is, ORUtils::MemoryBlock<T>& block, int blockSize)
  {
    // Try and read the block's size.
    if(block.dataSize != blockSize)
    {
      throw std::runtime_error("Could not read data into a memory block of the wrong size");
    }

    // Try and read the block's data.
    if(!is.read(reinterpret_cast<char*>(block.GetData(MEMORYDEVICE_CPU)), blockSize * sizeof(T)))
    {
      throw std::runtime_error("Could not read memory block data");
    }
  }

  /**
   * \brief Attempts to read data into a memory block allocated on the CPU from a file that contains data for a single block.
   *
   * The memory block must have the specified size (which should have been obtained by a call to ReadBlockSize).
   *
   * \param filename            The name of the file.
   * \param block               The memory block into which to read.
   * \param blockSize           The required size for the memory block.
   * \throws std::runtime_error If the read is unsuccessful.
   */
  template <typename T>
  static void ReadBlockData(const std::string& filename, ORUtils::MemoryBlock<T>& block, int blockSize)
  {
    std::ifstream fs(filename.c_str(), std::ios::binary);
    if(!fs) throw std::runtime_error("Could not open " + filename + " for reading");

    // Try and skip the block's size.
    if(!fs.seekg(sizeof(int))) throw std::runtime_error("Could not skip memory block size");

    // Try and read the block's data.
    ReadBlockData(fs, block, blockSize);
  }

  /**
   * \brief Attempts to read the size of a memory block from an input stream.
   *
   * The size is stored as a single integer and precedes the data for the block.
   *
   * \param is                  The input stream.
   * \return                    The size of the memory block.
   * \throws std::runtime_error If the read is unsuccesssful.
   */
  static int ReadBlockSize(std::istream& is)
  {
    int blockSize;
    if(is.read(reinterpret_cast<char*>(&blockSize), sizeof(int))) return blockSize;
    else throw std::runtime_error("Could not read memory block size");
  }

  /**
   * \brief Attempts to write a memory block allocated on the CPU to an output stream.
   *
   * A single integer containing the number of elements in the block is written prior to the block itself.
   *
   * \param os                  The output stream.
   * \param block               The memory block to write.
   * \throws std::runtime_error If the write is unsuccessful.
   */
  template <typename T>
  static void WriteBlock(std::ostream& os, const ORUtils::MemoryBlock<T>& block)
  {
    // Try and write the block's size.
    if(!os.write(reinterpret_cast<const char *>(&block.dataSize), sizeof(block.dataSize)))
    {
      throw std::runtime_error("Could not write memory block size");
    }

    // Try and write the block's data.
    if(!os.write(reinterpret_cast<const char *>(block.GetData(MEMORYDEVICE_CPU)), block.dataSize * sizeof(T)))
    {
      throw std::runtime_error("Could not write memory block data");
    }
  }
};

}
