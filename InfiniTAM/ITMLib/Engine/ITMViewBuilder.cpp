// Copyright 2014 Isis Innovation Limited and the authors of InfiniTAM

#include "ITMViewBuilder.h"

using namespace ITMLib::Engine;
using namespace ITMLib::Objects;
using namespace ORUtils;

ITMViewBuilder::ITMViewBuilder(const ITMRGBDCalib *calib, ITMLibSettings::DeviceType deviceType)
{
	this->calib = calib;
	this->deviceType = deviceType;
	this->shortImage = NULL;

	if (calib->disparityCalib.params.y == 0) inputImageType = InfiniTAM_SHORT_DEPTH_IMAGE;
	else inputImageType = InfiniTAM_DISPARITY_IMAGE;
}

ITMViewBuilder::~ITMViewBuilder()
{
	if (shortImage != NULL) delete shortImage;
}

void ITMViewBuilder::allocateView(ITMView *view, Vector2i imgSize_rgb, Vector2i imgSize_d, bool useGPU, bool allocateShort)
{
	view->calib = new ITMRGBDCalib(*calib);
	view->rgb = new ITMUChar4Image(imgSize_rgb, true, useGPU);
	view->depth = new ITMFloatImage(imgSize_d, true, useGPU);

	if (allocateShort) this->shortImage = new ITMShortImage(imgSize_d, true, useGPU);

	view->isAllocated = true;
}

void ITMViewBuilder::UpdateView(ITMView *view, ITMUChar4Image *rgbImage, ITMShortImage *rawDepthImage)
{
	bool useGPU = (deviceType == ITMLibSettings::DEVICE_CUDA);

	if (!view->isAllocated) this->allocateView(view, rgbImage->noDims, rawDepthImage->noDims, useGPU, true);

	if (useGPU)
	{
		view->rgb->SetFrom(rgbImage, MemoryBlock<Vector4u>::CPU_TO_CUDA);
		this->shortImage->SetFrom(rawDepthImage, MemoryBlock<short>::CPU_TO_CUDA);
	}
	else
	{
		view->rgb->SetFrom(rgbImage, MemoryBlock<Vector4u>::CPU_TO_CPU);
		this->shortImage->SetFrom(rawDepthImage, MemoryBlock<short>::CPU_TO_CPU);
	}

	if (inputImageType == InfiniTAM_DISPARITY_IMAGE)
		this->ConvertDisparityToDepth(view->depth, this->shortImage, &(view->calib->intrinsics_d), &(view->calib->disparityCalib));
	else if (inputImageType == InfiniTAM_SHORT_DEPTH_IMAGE)
		this->ConvertDepthMMToFloat(view->depth, this->shortImage);
}

void ITMViewBuilder::UpdateView(ITMView *view, ITMUChar4Image *rgbImage, ITMFloatImage *depthImage)
{
	bool useGPU = (deviceType == ITMLibSettings::DEVICE_CUDA);

	if (!view->isAllocated) this->allocateView(view, rgbImage->noDims, depthImage->noDims, useGPU, false);

	if (useGPU)
	{
		view->rgb->UpdateDeviceFromHost();
		view->depth->UpdateDeviceFromHost();
	}
}