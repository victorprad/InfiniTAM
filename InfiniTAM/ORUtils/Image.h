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
	class Image : public MemoryBlock < T >
	{
	public:
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

		/** Resize an image, loosing all old image data.
		Essentially any previously allocated data is
		released, new memory is allocated.
		*/
		void ChangeDims(Vector2<int> newDims, bool noResize = false)
		{
			MemoryBlock<T>::ChangeDims(newDims.x * newDims.y, noResize);
			noDims = newDims;
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
