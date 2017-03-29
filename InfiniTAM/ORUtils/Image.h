// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#include "MemoryBlock.h"
#include "Vector.h"

#ifndef __METALC__

namespace ORUtils
{
	/** \brief
	Represents images, templated on the pixel type
	*/
	template <typename T>
	class Image : private MemoryBlock<T>
	{
	public:
		/** Expose public MemoryBlock<T> member variables. */
		using MemoryBlock<T>::dataSize;

		/** Expose public MemoryBlock<T> datatypes. */
		using typename MemoryBlock<T>::MemoryCopyDirection;
		using MemoryBlock<T>::CPU_TO_CPU;
		using MemoryBlock<T>::CPU_TO_CUDA;
		using MemoryBlock<T>::CUDA_TO_CPU;
		using MemoryBlock<T>::CUDA_TO_CUDA;

		/** Expose public MemoryBlock<T> member functions. */
		using MemoryBlock<T>::Clear;
		using MemoryBlock<T>::GetData;
		using MemoryBlock<T>::GetElement;
#ifdef COMPILE_WITH_METAL
		using MemoryBlock<T>::GetMetalBuffer();
#endif
		using MemoryBlock<T>::UpdateDeviceFromHost;
		using MemoryBlock<T>::UpdateHostFromDevice;

		/** Size of the image in pixels. */
		Vector2<int> noDims;

		/** Initialize an empty image of the given size, either
		on CPU only or on both CPU and GPU.
		*/
		Image(Vector2<int> noDims, bool allocate_CPU, bool allocate_CUDA, bool metalCompatible = true)
			: MemoryBlock<T>(noDims.x * noDims.y, allocate_CPU, allocate_CUDA, metalCompatible)
		{
			this->noDims = noDims;
		}

		Image(bool allocate_CPU, bool allocate_CUDA, bool metalCompatible = true)
			: MemoryBlock<T>(0, allocate_CPU, allocate_CUDA, metalCompatible)
		{
			this->noDims = Vector2<int>(0, 0);
		}

		Image(Vector2<int> noDims, MemoryDeviceType memoryType)
			: MemoryBlock<T>(noDims.x * noDims.y, memoryType)
		{
			this->noDims = noDims;
		}

		/** Resize an image, losing all old image data.
		Essentially any previously allocated data is
		released, new memory is allocated.
		*/
		void ChangeDims(Vector2<int> newDims, bool forceReallocation = true)
		{
			MemoryBlock<T>::Resize(newDims.x * newDims.y, forceReallocation);
			noDims = newDims;
		}

		void SetFrom(const Image<T> *source, MemoryCopyDirection memoryCopyDirection)
		{
			ChangeDims(source->noDims);
			MemoryBlock<T>::SetFrom(source, memoryCopyDirection);
		}

		void Swap(Image<T>& rhs)
		{
			MemoryBlock<T>::Swap(rhs);
			std::swap(this->noDims, rhs.noDims);
		}

		// Suppress the default copy constructor and assignment operator
		Image(const Image&);
		Image& operator=(const Image&);
	};
}

#endif
