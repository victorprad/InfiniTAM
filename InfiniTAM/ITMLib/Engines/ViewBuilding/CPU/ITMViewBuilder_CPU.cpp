// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM

#include "ITMViewBuilder_CPU.h"

#include "../Shared/ITMViewBuilder_Shared.h"
#include "../../../../ORUtils/MetalContext.h"

#include "../../../../ORUtils/FileUtils.h"

using namespace ITMLib;
using namespace ORUtils;

ITMViewBuilder_CPU::ITMViewBuilder_CPU(const ITMRGBDCalib *calib, const Vector2i &paddingSize) : ITMViewBuilder(calib, paddingSize) { }
ITMViewBuilder_CPU::~ITMViewBuilder_CPU(void) { }

void ITMViewBuilder_CPU::UpdateView(ITMView **view_ptr, ITMUChar4Image *in_rgbImage, ITMShortImage *in_rawDepthImage, bool useBilateralFilter, bool modelSensorNoise)
{ 
	if (*view_ptr == NULL)
	{
		Vector2i newDims_depth, newDims_rgb;

		newDims_depth = 2 * paddingSize + in_rawDepthImage->noDims;
		newDims_rgb = 2 * paddingSize + in_rgbImage->noDims;

		if (this->rawDepthImage != NULL) delete this->rawDepthImage;
		this->rawDepthImage = new ITMShortImage(newDims_depth, true, false);

		if (this->rgbImage != NULL) delete this->rgbImage;
		this->rgbImage = new ITMUChar4Image(newDims_rgb, true, false);

		*view_ptr = new ITMView(calib, rgbImage->noDims, rawDepthImage->noDims, false);
		
		if (this->shortImage != NULL) delete this->shortImage;
		this->shortImage = new ITMShortImage(rawDepthImage->noDims, true, false);
		if (this->floatImage != NULL) delete this->floatImage;
		this->floatImage = new ITMFloatImage(rawDepthImage->noDims, true, false);

		if (modelSensorNoise)
		{
			(*view_ptr)->depthNormals = new ITMFloat4Image(rawDepthImage->noDims, true, false);
			(*view_ptr)->depthUncertainty = new ITMFloatImage(rawDepthImage->noDims, true, false);
		}
	}
	ITMView *view = *view_ptr;

	this->PadImage(this->rawDepthImage, in_rawDepthImage, paddingSize);
	this->PadImage(this->rgbImage, in_rgbImage, paddingSize);

	view->rgb->SetFrom(rgbImage, MemoryBlock<Vector4u>::CPU_TO_CPU);
	this->shortImage->SetFrom(rawDepthImage, MemoryBlock<short>::CPU_TO_CPU);

	switch (view->calib->disparityCalib.type)
	{
	case ITMDisparityCalib::TRAFO_KINECT:
		this->ConvertDisparityToDepth(view->depth, this->shortImage, &(view->calib->intrinsics_d), view->calib->disparityCalib.params);
		break;
	case ITMDisparityCalib::TRAFO_AFFINE:
		this->ConvertDepthAffineToFloat(view->depth, this->shortImage, view->calib->disparityCalib.params);
		break;
	default:
		break;
	}

	float *confidence = view->depthConfidence->GetData(MEMORYDEVICE_CPU);
	for (int y = 0; y < rawDepthImage->noDims.y; y++) for (int x = 0; x < rawDepthImage->noDims.x; x++)
	{
		confidence[x + y * rawDepthImage->noDims.x] = 1.0f;
	}

	if (useBilateralFilter)
	{
		//5 steps of bilateral filtering
		this->DepthFiltering(this->floatImage, view->depth);
		this->DepthFiltering(view->depth, this->floatImage);
		this->DepthFiltering(this->floatImage, view->depth);
		this->DepthFiltering(view->depth, this->floatImage);
		this->DepthFiltering(this->floatImage, view->depth);
		view->depth->SetFrom(this->floatImage, MemoryBlock<float>::CPU_TO_CPU);
	}

	if (modelSensorNoise)
	{
		this->ComputeNormalAndWeights(view->depthNormals, view->depthUncertainty, view->depth, view->calib->intrinsics_d.projectionParamsSimple.all);
	}

	//WriteToBIN(view->depth->GetData(MEMORYDEVICE_CPU), view->depth->dataSize, "c:/temp/data.bin");
}

//void ITMViewBuilder_CPU::UpdateView(ITMView **view_ptr, ITMUChar4Image *rgbImage, ITMFloatImage *depthImage)
//{
//	if (*view_ptr == NULL)
//		*view_ptr = new ITMView(calib, rgbImage->noDims, depthImage->noDims, false);
//
//	ITMView *view = *view_ptr;
//
//	view->rgb->UpdateDeviceFromHost();
//	view->depth->UpdateDeviceFromHost();
//}

void ITMViewBuilder_CPU::UpdateView(ITMView **view_ptr, ITMUChar4Image *in_rgbImage, ITMShortImage *in_rawDepthImage, bool useBilateralFilter, ITMIMUMeasurement *imuMeasurement, bool modelSensorNoise)
{
	if (*view_ptr == NULL)
	{
		Vector2i newDims_depth, newDims_rgb;

		newDims_depth = 2 * paddingSize + in_rawDepthImage->noDims;
		newDims_rgb = 2 * paddingSize + in_rgbImage->noDims;

		if (this->rawDepthImage != NULL) delete this->rawDepthImage;
		this->rawDepthImage = new ITMShortImage(newDims_depth, true, false);

		if (this->rgbImage != NULL) delete this->rgbImage;
		this->rgbImage = new ITMUChar4Image(newDims_rgb, true, false);

		*view_ptr = new ITMViewIMU(calib, rgbImage->noDims, depthImage->noDims, false);
		if (this->shortImage != NULL) delete this->shortImage;
		this->shortImage = new ITMShortImage(depthImage->noDims, true, false);
		if (this->floatImage != NULL) delete this->floatImage;
		this->floatImage = new ITMFloatImage(depthImage->noDims, true, false);

		if (modelSensorNoise)
		{
			(*view_ptr)->depthNormals = new ITMFloat4Image(depthImage->noDims, true, false);
			(*view_ptr)->depthUncertainty = new ITMFloatImage(depthImage->noDims, true, false);
		}
	}

	ITMViewIMU* imuView = (ITMViewIMU*)(*view_ptr);
	imuView->imu->SetFrom(imuMeasurement);

	this->UpdateView(view_ptr, rgbImage, rawDepthImage, useBilateralFilter, modelSensorNoise);
}

void ITMViewBuilder_CPU::ConvertDisparityToDepth(ITMFloatImage *depth_out, const ITMShortImage *depth_in, const ITMIntrinsics *depthIntrinsics,
	Vector2f disparityCalibParams)
{
	Vector2i imgSize = depth_in->noDims;

	const short *d_in = depth_in->GetData(MEMORYDEVICE_CPU);
	float *d_out = depth_out->GetData(MEMORYDEVICE_CPU);

	float fx_depth = depthIntrinsics->projectionParamsSimple.fx;

	for (int y = 0; y < imgSize.y; y++) for (int x = 0; x < imgSize.x; x++)
		convertDisparityToDepth(d_out, x, y, d_in, disparityCalibParams, fx_depth, imgSize);
}

void ITMViewBuilder_CPU::ConvertDepthAffineToFloat(ITMFloatImage *depth_out, const ITMShortImage *depth_in, const Vector2f depthCalibParams)
{
	Vector2i imgSize = depth_in->noDims;

	const short *d_in = depth_in->GetData(MEMORYDEVICE_CPU);
	float *d_out = depth_out->GetData(MEMORYDEVICE_CPU);

	for (int y = 0; y < imgSize.y; y++) for (int x = 0; x < imgSize.x; x++)
		convertDepthAffineToFloat(d_out, x, y, d_in, imgSize, depthCalibParams);
}

void ITMViewBuilder_CPU::DepthFiltering(ITMFloatImage *image_out, const ITMFloatImage *image_in)
{
	Vector2i imgSize = image_in->noDims;

	image_out->Clear();

	float *imout = image_out->GetData(MEMORYDEVICE_CPU);
	const float *imin = image_in->GetData(MEMORYDEVICE_CPU);

	for (int y = 2; y < imgSize.y - 2; y++) for (int x = 2; x < imgSize.x - 2; x++)
	{
		imout[x + y * imgSize.x] = imin[x + y * imgSize.x];
		filterDepth(imout, imin, x, y, imgSize);
	}
}

void ITMViewBuilder_CPU::ComputeNormalAndWeights(ITMFloat4Image *normal_out, ITMFloatImage *sigmaZ_out, const ITMFloatImage *depth_in, Vector4f intrinsic)
{
	Vector2i imgDims = depth_in->noDims;

	const float *depthData_in = depth_in->GetData(MEMORYDEVICE_CPU);

	float *sigmaZData_out = sigmaZ_out->GetData(MEMORYDEVICE_CPU);
	Vector4f *normalData_out = normal_out->GetData(MEMORYDEVICE_CPU);

	for (int y = 2; y < imgDims.y - 2; y++) for (int x = 2; x < imgDims.x - 2; x++)
		computeNormalAndWeight(depthData_in, normalData_out, sigmaZData_out, x, y, imgDims, intrinsic);
}

