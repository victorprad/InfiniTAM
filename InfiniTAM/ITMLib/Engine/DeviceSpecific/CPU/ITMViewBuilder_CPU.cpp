// Copyright 2014 Isis Innovation Limited and the authors of InfiniTAM

#include "ITMViewBuilder_CPU.h"

#include "../../DeviceAgnostic/ITMViewBuilder.h"

using namespace ITMLib::Engine;

ITMViewBuilder_CPU::ITMViewBuilder_CPU(const ITMRGBDCalib *calib, ITMLibSettings::DeviceType deviceType)
	:ITMViewBuilder(calib, deviceType) { }
ITMViewBuilder_CPU::~ITMViewBuilder_CPU(void) { }

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

void ITMViewBuilder_CPU::ConvertDepthMMToFloat(ITMFloatImage *depth_out, const ITMShortImage *depth_in)
{
	Vector2i imgSize = depth_in->noDims;

	const short *d_in = depth_in->GetData(MEMORYDEVICE_CPU);
	float *d_out = depth_out->GetData(MEMORYDEVICE_CPU);

	for (int y = 0; y < imgSize.y; y++) for (int x = 0; x < imgSize.x; x++)
		convertDepthMMToFloat(d_out, x, y, d_in, imgSize);
}
