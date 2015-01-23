// Copyright 2014 Isis Innovation Limited and the authors of InfiniTAM

#include "ITMViewBuilder_CPU.h"

#include "../../DeviceAgnostic/ITMViewBuilder.h"
#include "../../../../ORUtils/MetalContext.h"

using namespace ITMLib::Engine;
using namespace ORUtils;

ITMViewBuilder_CPU::ITMViewBuilder_CPU(const ITMRGBDCalib *calib):ITMViewBuilder(calib) { }
ITMViewBuilder_CPU::~ITMViewBuilder_CPU(void) { }

void ITMViewBuilder_CPU::UpdateView(ITMView *view, ITMUChar4Image *rgbImage, ITMShortImage *rawDepthImage)
{
	view->rgb->SetFrom(rgbImage, MemoryBlock<Vector4u>::MemoryCopyDirection::CPU_TO_CPU);
	this->shortImage->SetFrom(rawDepthImage, MemoryBlock<short>::MemoryCopyDirection::CPU_TO_CPU);

	switch (inputImageType)
	{
	case InfiniTAM_DISPARITY_IMAGE:
		this->ConvertDisparityToDepth(view->depth, this->shortImage, &(view->calib->intrinsics_d), &(view->calib->disparityCalib));
		break;
	case InfiniTAM_SHORT_DEPTH_IMAGE:
		this->ConvertDepthMMToFloat(view->depth, this->shortImage);
		break;
	default:
		break;
	}		
}

void ITMViewBuilder_CPU::UpdateView(ITMView *view, ITMUChar4Image *rgbImage, ITMFloatImage *depthImage)
{
	view->rgb->UpdateDeviceFromHost();
	view->depth->UpdateDeviceFromHost();
}

void ITMViewBuilder_CPU::ConvertDisparityToDepth(ITMFloatImage *depth_out, const ITMShortImage *depth_in, const ITMIntrinsics *depthIntrinsics,
	const ITMDisparityCalib *disparityCalib)
{
	Vector2i imgSize = depth_in->noDims;

	const short *d_in = depth_in->GetData(MEMORYDEVICE_CPU);
	float *d_out = depth_out->GetData(MEMORYDEVICE_CPU);

	Vector2f disparityCalibParams; float fx_depth;
	disparityCalibParams.x = disparityCalib->params.x;
	disparityCalibParams.y = disparityCalib->params.y;
	fx_depth = depthIntrinsics->projectionParamsSimple.fx;

	for (int y = 0; y < imgSize.y; y++) for (int x = 0; x < imgSize.x; x++)
		convertDisparityToDepth(d_out, x, y, d_in, disparityCalibParams, fx_depth, imgSize);
}

void ITMViewBuilder_CPU::AllocateView(ITMView *view, Vector2i imgSize_rgb, Vector2i imgSize_d)
{
	view->calib = new ITMRGBDCalib(*calib);
	view->rgb = new ITMUChar4Image(imgSize_rgb, true, false);
	view->depth = new ITMFloatImage(imgSize_d, true, false);

	if (this->shortImage != NULL) delete this->shortImage;
	this->shortImage = new ITMShortImage(imgSize_d, true, false);

	view->isAllocated = true;
}

void ITMViewBuilder_CPU::ConvertDepthMMToFloat(ITMFloatImage *depth_out, const ITMShortImage *depth_in)
{
	Vector2i imgSize = depth_in->noDims;

	const short *d_in = depth_in->GetData(MEMORYDEVICE_CPU);
	float *d_out = depth_out->GetData(MEMORYDEVICE_CPU);

	for (int y = 0; y < imgSize.y; y++) for (int x = 0; x < imgSize.x; x++)
		convertDepthMMToFloat(d_out, x, y, d_in, imgSize);
}
