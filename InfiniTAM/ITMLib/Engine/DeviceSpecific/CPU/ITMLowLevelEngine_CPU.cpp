// Copyright 2014 Isis Innovation Limited and the authors of InfiniTAM

#include "ITMLowLevelEngine_CPU.h"

#include "../../DeviceAgnostic/ITMLowLevelEngine.h"

using namespace ITMLib::Engine;

ITMLowLevelEngine_CPU::ITMLowLevelEngine_CPU(void) { }
ITMLowLevelEngine_CPU::~ITMLowLevelEngine_CPU(void) { }

void ITMLowLevelEngine_CPU::CopyImage(ITMUChar4Image *image_out, const ITMUChar4Image *image_in)
{
	Vector4u *dest = image_out->GetData(false);
	const Vector4u *src = image_in->GetData(false);

	memcpy(dest, src, image_in->dataSize * sizeof(Vector4u));
}

void ITMLowLevelEngine_CPU::CopyImage(ITMFloatImage *image_out, const ITMFloatImage *image_in)
{
	float *dest = image_out->GetData(false);
	const float *src = image_in->GetData(false);

	memcpy(dest, src, image_in->dataSize * sizeof(float));
}

void ITMLowLevelEngine_CPU::CopyImage(ITMFloat4Image *image_out, const ITMFloat4Image *image_in)
{
	Vector4f *dest = image_out->GetData(false);
	const Vector4f *src = image_in->GetData(false);

	memcpy(dest, src, image_in->dataSize * sizeof(Vector4f));
}

void ITMLowLevelEngine_CPU::FilterSubsample(ITMUChar4Image *image_out, const ITMUChar4Image *image_in)
{
	Vector2i oldDims = image_in->noDims;
	Vector2i newDims; newDims.x = image_in->noDims.x / 2; newDims.y = image_in->noDims.y / 2;

	image_out->ChangeDims(newDims);

	const Vector4u *imageData_in = image_in->GetData(false);
	Vector4u *imageData_out = image_out->GetData(false);

	for (int y = 0; y < newDims.y; y++) for (int x = 0; x < newDims.x; x++)
		filterSubsample(imageData_out, x, y, newDims, imageData_in, oldDims);
}

void ITMLowLevelEngine_CPU::FilterSubsampleWithHoles(ITMFloatImage *image_out, const ITMFloatImage *image_in)
{
	Vector2i oldDims = image_in->noDims;
	Vector2i newDims; newDims.x = image_in->noDims.x / 2; newDims.y = image_in->noDims.y / 2;

	image_out->ChangeDims(newDims);

	const float *imageData_in = image_in->GetData(false);
	float *imageData_out = image_out->GetData(false);

	for (int y = 0; y < newDims.y; y++) for (int x = 0; x < newDims.x; x++)
		filterSubsampleWithHoles(imageData_out, x, y, newDims, imageData_in, oldDims);
}

void ITMLowLevelEngine_CPU::FilterSubsampleWithHoles(ITMFloat4Image *image_out, const ITMFloat4Image *image_in)
{
	Vector2i oldDims = image_in->noDims;
	Vector2i newDims; newDims.x = image_in->noDims.x / 2; newDims.y = image_in->noDims.y / 2;

	image_out->ChangeDims(newDims);

	const Vector4f *imageData_in = image_in->GetData(false);
	Vector4f *imageData_out = image_out->GetData(false);

	for (int y = 0; y < newDims.y; y++) for (int x = 0; x < newDims.x; x++)
		filterSubsampleWithHoles(imageData_out, x, y, newDims, imageData_in, oldDims);
}

void ITMLowLevelEngine_CPU::GradientX(ITMShort4Image *grad_out, const ITMUChar4Image *image_in)
{
	grad_out->ChangeDims(image_in->noDims);
	Vector2i imgSize = image_in->noDims;

	Vector4s *grad = grad_out->GetData(false); 
	const Vector4u *image = image_in->GetData(false);

	memset(grad, 0, imgSize.x * imgSize.y * sizeof(Vector3s));

	for (int y = 1; y < imgSize.y - 1; y++) for (int x = 1; x < imgSize.x - 1; x++)
		gradientX(grad, x, y, image, imgSize);
}

void ITMLowLevelEngine_CPU::GradientY(ITMShort4Image *grad_out, const ITMUChar4Image *image_in)
{
	grad_out->ChangeDims(image_in->noDims);
	Vector2i imgSize = image_in->noDims;

	Vector4s *grad = grad_out->GetData(false);
	const Vector4u *image = image_in->GetData(false);

	memset(grad, 0, imgSize.x * imgSize.y * sizeof(Vector3s));

	for (int y = 1; y < imgSize.y - 1; y++) for (int x = 1; x < imgSize.x - 1; x++)
		gradientY(grad, x, y, image, imgSize);
}

void ITMLowLevelEngine_CPU::ConvertDisparityToDepth(ITMFloatImage *depth_out, const ITMShortImage *depth_in, const ITMIntrinsics *depthIntrinsics,
	const ITMDisparityCalib *disparityCalib)
{
	Vector2i imgSize = depth_in->noDims;

	const short *d_in = depth_in->GetData(false);
	float *d_out = depth_out->GetData(false);

	Vector2f disparityCalibParams; float fx_depth;
	disparityCalibParams.x = disparityCalib->params.x;
	disparityCalibParams.y = disparityCalib->params.y;
	fx_depth = depthIntrinsics->projectionParamsSimple.fx;

	for (int y = 0; y < imgSize.y; y++) for (int x = 0; x < imgSize.x; x++)
		convertDisparityToDepth(d_out, x, y, d_in, disparityCalibParams, fx_depth, imgSize);
}

void ITMLowLevelEngine_CPU::ConvertDepthMMToFloat(ITMFloatImage *depth_out, const ITMShortImage *depth_in)
{
	Vector2i imgSize = depth_in->noDims;

	const short *d_in = depth_in->GetData(false);
	float *d_out = depth_out->GetData(false);

	for (int y = 0; y < imgSize.y; y++) for (int x = 0; x < imgSize.x; x++)
		convertDepthMMToFloat(d_out, x, y, d_in, imgSize);
}
