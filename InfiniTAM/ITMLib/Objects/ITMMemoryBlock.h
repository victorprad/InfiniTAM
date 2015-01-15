// Copyright 2014 Isis Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../Utils/ITMLibDefines.h"

#ifndef COMPILE_WITHOUT_CUDA
#include "../Engine/DeviceSpecific/CUDA/ITMCUDADefines.h"
#endif

#ifdef COMPILE_WITH_METAL
#include "../Engine/DeviceSpecific/Metal/ITMMetalContext.h"
#endif

#include <stdlib.h>
#include <string.h>

namespace ITMLib
{
	namespace Objects
	{
		/** \brief
		Represents memory blocks, templated on the data type
		*/
		template <typename T>
		class ITMMemoryBlock
		{
		protected:
			bool isAllocated_CPU, isAllocated_CUDA, isMetalCompatible;

			/** Pointer to memory on CPU host. */
			T* data_cpu;

			/** Pointer to memory on GPU, if available. */
			T* data_cuda;

#ifdef COMPILE_WITH_METAL
			void *data_metalBuffer;
#endif
		public:
			/** Total number of allocated entries in the data array. */
			int dataSize;

			/** Get the data pointer on CPU or GPU. */
			inline T* GetData(bool useGPU) { return useGPU ? data_cuda : data_cpu; }

			/** Get the data pointer on CPU or GPU. */
			inline const T* GetData(bool useGPU) const { return useGPU ? data_cuda : data_cpu; }

#ifdef COMPILE_WITH_METAL
			inline const void *GetMetalBuffer() const { return data_metalBuffer; }
#endif

			///** Initialize an empty 0x0 array of data, either on CPU only
			//or on both CPU and GPU.
			//*/
			//explicit ITMMemoryBlock(bool allocate_CPU, bool allocate_CUDA, bool metalCompatible = false)
			//{
			//	this->isAllocated_CPU = allocate_CPU;
			//	this->isAllocated_CUDA = allocate_CUDA;
			//	this->isMetalCompatible = metalCompatible;

			//	this->dataSize = 0;
			//}

			/** Initialize an empty memory block of the given size, either
			on CPU only or on both CPU and GPU. CPU might also use the
			Metal compatible allocator (i.e. with 16384 alignment).
			*/
			ITMMemoryBlock(int dataSize, bool allocate_CPU, bool allocate_CUDA, bool metalCompatible = false)
			{
				this->isAllocated_CPU = false;
				this->isAllocated_CUDA = false;
				this->isMetalCompatible = false;

				Allocate(dataSize, allocate_CPU, allocate_CUDA, metalCompatible);
				Clear();
			}

			/** Set all image data to the given @p defaultValue. */
			void Clear(uchar defaultValue = 0)
			{
				if (isAllocated_CPU) memset(data_cpu, defaultValue, dataSize * sizeof(T));
#ifndef COMPILE_WITHOUT_CUDA
				if (isAllocated_CUDA) ITMSafeCall(cudaMemset(data_cuda, defaultValue, dataSize * sizeof(T)));
#endif
			}

			/** Transfer data from CPU to GPU, if possible. */
			void UpdateDeviceFromHost() {
#ifndef COMPILE_WITHOUT_CUDA
				if (isAllocated_CUDA && isAllocated_CPU)
					ITMSafeCall(cudaMemcpy(data_cuda, data_cpu, dataSize * sizeof(T), cudaMemcpyHostToDevice));
#endif
			}
			/** Transfer data from GPU to CPU, if possible. */
			void UpdateHostFromDevice() {
#ifndef COMPILE_WITHOUT_CUDA
				if (isAllocated_CUDA && isAllocated_CPU)
					ITMSafeCall(cudaMemcpy(data_cpu, data_cuda, dataSize * sizeof(T), cudaMemcpyDeviceToHost));
#endif
			}

			/** Copy image content, does not resize image! */
			void SetFrom(const ITMMemoryBlock<T> *source, bool copyHost = true, bool copyDevice = false)
			{
				if (copyHost && isAllocated_CPU)
					memcpy(this->data_cpu, source->data_cpu, source->dataSize * sizeof(T));
#ifndef COMPILE_WITHOUT_CUDA
				if (copyDevice && isAllocated_CUDA)
					ITMSafeCall(cudaMemcpy(this->data_cuda, source->data_cuda, source->dataSize * sizeof(T), cudaMemcpyDeviceToDevice));
#endif
			}

			virtual ~ITMMemoryBlock() { this->Free(); }

			/** Allocate image data of the specified size. If the
			data has been allocated before, the data is freed.
			*/
			void Allocate(int dataSize, bool allocate_CPU, bool allocate_CUDA, bool metalCompatible)
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
						data_cpu = new T[dataSize];
						break;
					case 1:
#ifndef COMPILE_WITHOUT_CUDA
						ITMSafeCall(cudaMallocHost((void**)&data_cpu, dataSize * sizeof(T)));
#endif
						break;
					case 2:
#ifdef COMPILE_WITH_METAL
						allocateMetalData((void**)&data_cpu, (void**)&data_metalBuffer, dataSize * sizeof(T), true);
#endif
						break;
					}

					this->isAllocated_CPU = allocate_CPU;
					this->isMetalCompatible = metalCompatible;
				}

				if (allocate_CUDA)
				{
#ifndef COMPILE_WITHOUT_CUDA
					ITMSafeCall(cudaMalloc((void**)&data_cuda, dataSize * sizeof(T)));
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
					if (metalCompatible) allocType = 2;
#endif
					switch (allocType)
					{
					case 0: 
						delete[] data_cpu;
						break;
					case 1:
#ifndef COMPILE_WITHOUT_CUDA
						ITMSafeCall(cudaFreeHost(data_cpu));
#endif
						break;
					case 2:
#ifdef COMPILE_WITH_METAL
						freeMetalData((void**)&data_cpu, (void**)&data_metalBuffer, dataSize * sizeof(T), true);
#endif
						break;
					}

					isMetalCompatible = false;
					isAllocated_CPU = false;
				}

				if (isAllocated_CUDA)
				{
					if (!isAllocated_CUDA) return;
#ifndef COMPILE_WITHOUT_CUDA
					ITMSafeCall(cudaMalloc((void**)&data_cuda, dataSize * sizeof(T)));
#endif
					isAllocated_CUDA = false;
				}
			}

			// Suppress the default copy constructor and assignment operator
			ITMMemoryBlock(const ITMMemoryBlock&);
			ITMMemoryBlock& operator=(const ITMMemoryBlock&);
		};
	}
}