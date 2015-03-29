// Copyright 2014 Isis Innovation Limited and the authors of InfiniTAM

#pragma once

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
  static void loadMemoryBlock(const std::string& filename, ORUtils::MemoryBlock<T>& block, MemoryDeviceType memoryDeviceType)
  {
    int blockSize = readBlockSize(filename);
    if(memoryDeviceType == MEMORYDEVICE_CUDA)
    {
      // If we're loading into a block on the GPU, first try and read the data into a temporary block on the CPU.
      ORUtils::MemoryBlock<T> cpuBlock(block.dataSize, MEMORYDEVICE_CPU);
      readBlockData(filename, cpuBlock, blockSize);

      // Then copy the data across to the GPU.
      block.SetFrom(&cpuBlock, ORUtils::MemoryBlock<T>::CPU_TO_CUDA);
    }
    else
    {
      // If we're loading into a block on the CPU, read the data directly into the block.
      readBlockData(filename, block, blockSize);
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
  static ORUtils::MemoryBlock<T> *loadMemoryBlock(const std::string& filename, ORUtils::MemoryBlock<T> *dummy = NULL)
  {
    int blockSize = readBlockSize(filename);
    ORUtils::MemoryBlock<T> *block = new ORUtils::MemoryBlock<T>(blockSize, MEMORYDEVICE_CPU);
    readBlockData(filename, *block, blockSize);
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
  static int readBlockSize(const std::string& filename)
  {
    FILE *file = fopen(filename.c_str(), "rb");
    if(!file) throw std::runtime_error("Could not open " + filename + " for reading");

    try
    {
      int blockSize = readBlockSize(file);
      fclose(file);
      return blockSize;
    }
    catch(std::runtime_error&)
    {
      fclose(file);
      throw;
    }
  }

  /**
   * \brief Saves a memory block to a file on disk.
   *
   * \param filename          The name of the file.
   * \param block             The memory block to save.
   * \param memoryDeviceType  The type of memory device from which to save the data.
   */
  template <typename T>
  static void saveMemoryBlock(const std::string& filename, const ORUtils::MemoryBlock<T>& block, MemoryDeviceType memoryDeviceType)
  {
    FILE *file = fopen(filename.c_str(), "wb");
    if(!file) throw std::runtime_error("Could not open " + filename + " for writing");

    try
    {
      if(memoryDeviceType == MEMORYDEVICE_CUDA)
      {
        // If we are saving the memory block from the GPU, first make a CPU copy of it.
        ORUtils::MemoryBlock<T> cpuBlock(block.dataSize, MEMORYDEVICE_CPU);
        cpuBlock.SetFrom(&block, ORUtils::MemoryBlock<T>::CUDA_TO_CPU);

        // Then write the CPU copy to disk.
        writeBlock(file, cpuBlock);
      }
      else
      {
        // If we are saving the memory block from the CPU, write it directly to disk.
        writeBlock(file, block);
      }

      fclose(file);
    }
    catch(std::runtime_error&)
    {
      fclose(file);
      throw;
    }
  }

  //#################### PRIVATE STATIC MEMBER FUNCTIONS ####################
private:
  /**
   * \brief Attempts to read data into a memory block allocated on the CPU from an already-opened file.
   *
   * The memory block must have the specified size (which should have been obtained by a call to readBlockSize).
   *
   * \param file                The file pointer.
   * \param block               The memory block into which to read.
   * \param blockSize           The required size for the memory block.
   * \throws std::runtime_error If the read is unsuccessful.
   */
  template <typename T>
  static void readBlockData(FILE *file, ORUtils::MemoryBlock<T>& block, int blockSize)
  {
    if(block.dataSize != blockSize)
    {
      throw std::runtime_error("Could not read data into a memory block of the wrong size");
    }

    if(fread(block.GetData(MEMORYDEVICE_CPU), sizeof(T), blockSize, file) != blockSize)
    {
      throw std::runtime_error("Could not read memory block data");
    }
  }

  /**
   * \brief Attempts to read data into a memory block allocated on the CPU from a file that contains data for a single block.
   *
   * The memory block must have the specified size (which should have been obtained by a call to readBlockSize).
   *
   * \param filename            The name of the file.
   * \param block               The memory block into which to read.
   * \param blockSize           The required size for the memory block.
   * \throws std::runtime_error If the read is unsuccessful.
   */
  template <typename T>
  static void readBlockData(const std::string& filename, ORUtils::MemoryBlock<T>& block, int blockSize)
  {
    FILE *file = fopen(filename.c_str(), "rb");
    if(!file) throw std::runtime_error("Could not open " + filename + " for reading");

    if(fseek(file, sizeof(int), SEEK_SET) != 0)
    {
      fclose(file);
      throw std::runtime_error("Could not skip memory block size");
    }

    try
    {
      readBlockData(file, block, blockSize);
      fclose(file);
    }
    catch(std::runtime_error&)
    {
      fclose(file);
      throw;
    }
  }

  /**
   * \brief Attempts to read the size of a memory block from an already-opened file.
   *
   * The size is stored as a single integer and precedes the data for the block.
   *
   * \param file                The file pointer.
   * \return                    The size of the memory block.
   * \throws std::runtime_error If the read is unsuccesssful.
   */
  static int readBlockSize(FILE *file)
  {
    int blockSize;
    if(fread(&blockSize, sizeof(int), 1, file) == 1) return blockSize;
    else throw std::runtime_error("Could not read memory block size");
  }

  /**
   * \brief Attempts to write a memory block allocated on the CPU to an already-opened file.
   *
   * A single integer containing the number of elements in the block is written prior to the block itself.
   *
   * \param file                The file pointer.
   * \param block               The memory block to write.
   * \throws std::runtime_error If the write is unsuccessful.
   */
  template <typename T>
  static void writeBlock(FILE *file, const ORUtils::MemoryBlock<T>& block)
  {
    // Try and write the block's size.
    if(fwrite(&block.dataSize, sizeof(block.dataSize), 1, file) != 1)
    {
      throw std::runtime_error("Could not write memory block size");
    }

    // Try and write the block's data.
    if(fwrite(block.GetData(MEMORYDEVICE_CPU), sizeof(T), block.dataSize, file) != block.dataSize)
    {
      throw std::runtime_error("Could not write memory block data");
    }
  }
};

}
