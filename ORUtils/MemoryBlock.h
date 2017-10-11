// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#include "MemoryDeviceType.h"
#include "PlatformIndependence.h"

#ifndef __METALC__

#ifndef COMPILE_WITHOUT_CUDA
#include "CUDADefines.h"
#endif

#ifdef COMPILE_WITH_METAL
#include "MetalContext.h"
#endif

#include <stdlib.h>
#include <string.h>

namespace ORUtils
{
	/** \brief
	Represents memory blocks, templated on the data type
	*/
	template <typename T>
	class MemoryBlock
	{
	protected:
		bool isAllocated_CPU, isAllocated_CUDA, isMetalCompatible;
		/** Pointer to memory on CPU host. */
		DEVICEPTR(T)* data_cpu;

		/** Pointer to memory on GPU, if available. */
		DEVICEPTR(T)* data_cuda;

#ifdef COMPILE_WITH_METAL
		void *data_metalBuffer;
#endif
	public:
		enum MemoryCopyDirection { CPU_TO_CPU, CPU_TO_CUDA, CUDA_TO_CPU, CUDA_TO_CUDA };

		/** Total number of allocated entries in the data array. */
		size_t dataSize;

		/** Get the data pointer on CPU or GPU. */
		inline DEVICEPTR(T)* GetData(MemoryDeviceType memoryType)
		{
			switch (memoryType)
			{
			case MEMORYDEVICE_CPU: return data_cpu;
			case MEMORYDEVICE_CUDA: return data_cuda;
			}

			return 0;
		}

		/** Get the data pointer on CPU or GPU. */
		inline const DEVICEPTR(T)* GetData(MemoryDeviceType memoryType) const
		{
			switch (memoryType)
			{
			case MEMORYDEVICE_CPU: return data_cpu;
			case MEMORYDEVICE_CUDA: return data_cuda;
			}

			return 0;
		}
#ifdef COMPILE_WITH_METAL
		inline const void *GetMetalBuffer() const { return data_metalBuffer; }
#endif

		/** Initialize an empty memory block of the given size,
		on CPU only or GPU only or on both. CPU might also use the
		Metal compatible allocator (i.e. with 16384 alignment).
		*/
		MemoryBlock(size_t dataSize, bool allocate_CPU, bool allocate_CUDA, bool metalCompatible = true)
		{
			this->isAllocated_CPU = false;
			this->isAllocated_CUDA = false;
			this->isMetalCompatible = false;

#ifndef NDEBUG // When building in debug mode always allocate both on the CPU and the GPU
			if (allocate_CUDA) allocate_CPU = true;
#endif

			Allocate(dataSize, allocate_CPU, allocate_CUDA, metalCompatible);
			Clear();
		}

		/** Initialize an empty memory block of the given size, either
		on CPU only or on GPU only. CPU will be Metal compatible if Metal
		is enabled.
		*/
		MemoryBlock(size_t dataSize, MemoryDeviceType memoryType)
		{
			this->isAllocated_CPU = false;
			this->isAllocated_CUDA = false;
			this->isMetalCompatible = false;

			switch (memoryType)
			{
				case MEMORYDEVICE_CPU: Allocate(dataSize, true, false, true); break;
				case MEMORYDEVICE_CUDA:
				{
#ifndef NDEBUG // When building in debug mode always allocate both on the CPU and the GPU
					Allocate(dataSize, true, true, true);
#else
					Allocate(dataSize, false, true, true);
#endif
					break;
				}
			}

			Clear();
		}

		/** Set all image data to the given @p defaultValue. */
		void Clear(unsigned char defaultValue = 0)
		{
			if (isAllocated_CPU) memset(data_cpu, defaultValue, dataSize * sizeof(T));
#ifndef COMPILE_WITHOUT_CUDA
			if (isAllocated_CUDA) ORcudaSafeCall(cudaMemset(data_cuda, defaultValue, dataSize * sizeof(T)));
#endif
		}

		/** Resize a memory block, losing all old data.
		Essentially any previously allocated data is
		released, new memory is allocated.
		*/
		void Resize(size_t newDataSize, bool forceReallocation = true)
		{
			if(newDataSize == dataSize) return;

			if(newDataSize > dataSize || forceReallocation)
			{
				bool allocate_CPU = this->isAllocated_CPU;
				bool allocate_CUDA = this->isAllocated_CUDA;
				bool metalCompatible = this->isMetalCompatible;

				this->Free();
				this->Allocate(newDataSize, allocate_CPU, allocate_CUDA, metalCompatible);
			}

			this->dataSize = newDataSize;
		}

		/** Transfer data from CPU to GPU, if possible. */
		void UpdateDeviceFromHost() const {
#ifndef COMPILE_WITHOUT_CUDA
			if (isAllocated_CUDA && isAllocated_CPU)
				ORcudaSafeCall(cudaMemcpy(data_cuda, data_cpu, dataSize * sizeof(T), cudaMemcpyHostToDevice));
#endif
		}
		/** Transfer data from GPU to CPU, if possible. */
		void UpdateHostFromDevice() const {
#ifndef COMPILE_WITHOUT_CUDA
			if (isAllocated_CUDA && isAllocated_CPU)
				ORcudaSafeCall(cudaMemcpy(data_cpu, data_cuda, dataSize * sizeof(T), cudaMemcpyDeviceToHost));
#endif
		}

		/** Copy data */
		void SetFrom(const MemoryBlock<T> *source, MemoryCopyDirection memoryCopyDirection)
		{
			Resize(source->dataSize);
			switch (memoryCopyDirection)
			{
			case CPU_TO_CPU:
				memcpy(this->data_cpu, source->data_cpu, source->dataSize * sizeof(T));
				break;
#ifndef COMPILE_WITHOUT_CUDA
			case CPU_TO_CUDA:
				ORcudaSafeCall(cudaMemcpyAsync(this->data_cuda, source->data_cpu, source->dataSize * sizeof(T), cudaMemcpyHostToDevice));
				break;
			case CUDA_TO_CPU:
				ORcudaSafeCall(cudaMemcpy(this->data_cpu, source->data_cuda, source->dataSize * sizeof(T), cudaMemcpyDeviceToHost));
				break;
			case CUDA_TO_CUDA:
				ORcudaSafeCall(cudaMemcpyAsync(this->data_cuda, source->data_cuda, source->dataSize * sizeof(T), cudaMemcpyDeviceToDevice));
				break;
#endif
			default: break;
			}
		}

		/** Get an individual element of the memory block from either the CPU or GPU. */
		T GetElement(int n, MemoryDeviceType memoryType) const
		{
			switch(memoryType)
			{
				case MEMORYDEVICE_CPU:
				{
					return this->data_cpu[n];
				}
#ifndef COMPILE_WITHOUT_CUDA
				case MEMORYDEVICE_CUDA:
				{
					T result;
					ORcudaSafeCall(cudaMemcpy(&result, this->data_cuda + n, sizeof(T), cudaMemcpyDeviceToHost));
					return result;
				}
#endif
				default: throw std::runtime_error("Invalid memory type");
			}
		}

		virtual ~MemoryBlock() { this->Free(); }

		/** Allocate image data of the specified size. If the
		data has been allocated before, the data is freed.
		*/
		void Allocate(size_t dataSize, bool allocate_CPU, bool allocate_CUDA, bool metalCompatible)
		{
			Free();

			this->dataSize = dataSize;

			if (allocate_CPU)
			{
				int allocType = 0;

#ifndef COMPILE_WITHOUT_CUDA
				if (allocate_CUDA) allocType = 1;
#endif
#ifdef COMPILE_WITH_METAL
				if (metalCompatible) allocType = 2;
#endif
				switch (allocType)
				{
				case 0:
					if (dataSize == 0) data_cpu = NULL;
					else data_cpu = new T[dataSize];
					break;
				case 1:
#ifndef COMPILE_WITHOUT_CUDA
					if (dataSize == 0) data_cpu = NULL;
					else ORcudaSafeCall(cudaMallocHost((void**)&data_cpu, dataSize * sizeof(T)));
#endif
					break;
				case 2:
#ifdef COMPILE_WITH_METAL
					if (dataSize == 0) data_cpu = NULL;
					else allocateMetalData((void**)&data_cpu, (void**)&data_metalBuffer, (int)(dataSize * sizeof(T)), true);
#endif
					break;
				}

				this->isAllocated_CPU = allocate_CPU;
				this->isMetalCompatible = metalCompatible;
			}

			if (allocate_CUDA)
			{
#ifndef COMPILE_WITHOUT_CUDA
				if (dataSize == 0) data_cuda = NULL;
				else ORcudaSafeCall(cudaMalloc((void**)&data_cuda, dataSize * sizeof(T)));
				this->isAllocated_CUDA = allocate_CUDA;
#endif
			}
		}

		void Free()
		{
			if (isAllocated_CPU)
			{
				int allocType = 0;

#ifndef COMPILE_WITHOUT_CUDA
				if (isAllocated_CUDA) allocType = 1;
#endif
#ifdef COMPILE_WITH_METAL
				if (isMetalCompatible) allocType = 2;
#endif
				switch (allocType)
				{
				case 0:
					if (data_cpu != NULL) delete[] data_cpu;
					break;
				case 1:
#ifndef COMPILE_WITHOUT_CUDA
					if (data_cpu != NULL) ORcudaSafeCall(cudaFreeHost(data_cpu));
#endif
					break;
				case 2:
#ifdef COMPILE_WITH_METAL
					if (data_cpu != NULL) freeMetalData((void**)&data_cpu, (void**)&data_metalBuffer, (int)(dataSize * sizeof(T)), true);
#endif
					break;
				}

				isMetalCompatible = false;
				isAllocated_CPU = false;
			}

			if (isAllocated_CUDA)
			{
#ifndef COMPILE_WITHOUT_CUDA
				if (data_cuda != NULL) ORcudaSafeCall(cudaFree(data_cuda));
#endif
				isAllocated_CUDA = false;
			}
		}

		void Swap(MemoryBlock<T>& rhs)
		{
			std::swap(this->dataSize, rhs.dataSize);
			std::swap(this->data_cpu, rhs.data_cpu);
			std::swap(this->data_cuda, rhs.data_cuda);
#ifdef COMPILE_WITH_METAL
			std::swap(this->data_metalBuffer, rhs.data_metalBuffer);
#endif
			std::swap(this->isAllocated_CPU, rhs.isAllocated_CPU);
			std::swap(this->isAllocated_CUDA, rhs.isAllocated_CUDA);
			std::swap(this->isMetalCompatible, rhs.isMetalCompatible);
		}

		// Suppress the default copy constructor and assignment operator
		MemoryBlock(const MemoryBlock&);
		MemoryBlock& operator=(const MemoryBlock&);
	};
}

#endif
