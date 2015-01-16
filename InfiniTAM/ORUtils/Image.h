// Copyright 2014 Isis Innovation Limited and the authors of InfiniTAM

#pragma once

#include "MemoryBlock.h"

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
		Vector2i noDims;

		/** Initialize an empty image of the given size, either
		on CPU only or on both CPU and GPU.
		*/
		Image(Vector2i noDims, bool allocate_CPU, bool allocate_CUDA, bool metalCompatible = true)
			: MemoryBlock(noDims.x * noDims.y, allocate_CPU, allocate_CUDA, metalCompatible)
		{
			this->noDims = noDims;
		}

		Image(bool allocate_CPU, bool allocate_CUDA, bool metalCompatible = true)
			: MemoryBlock(1, allocate_CPU, allocate_CUDA, metalCompatible)
		{
			this->noDims = Vector2i(1, 1);  //TODO - make nicer
		}

		Image(Vector2i noDims, MemoryDeviceType memoryType)
			: MemoryBlock(noDims.x * noDims.y, memoryType)
		{
			this->noDims = noDims;
		}

		/** Resize an image, loosing all old image data.
		Essentially any previously allocated data is
		released, new memory is allocated.
		*/
		void ChangeDims(Vector2i newDims)
		{
			if (newDims != noDims)
			{
				this->noDims = newDims;

				bool allocate_CPU = isAllocated_CPU;
				bool allocate_CUDA = isAllocated_CUDA;
				bool metalCompatible = isMetalCompatible;

				Free();
				Allocate(newDims.x * newDims.y, allocate_CPU, allocate_CUDA, metalCompatible);
			}
		}

		// Suppress the default copy constructor and assignment operator
		Image(const Image&);
		Image& operator=(const Image&);
	};
}