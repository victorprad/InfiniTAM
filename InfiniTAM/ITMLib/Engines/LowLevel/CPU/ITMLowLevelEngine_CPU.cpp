// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#include "ITMLowLevelEngine_CPU.h"

#include "../Shared/ITMLowLevelEngine_Shared.h"

using namespace ITMLib;

ITMLowLevelEngine_CPU::ITMLowLevelEngine_CPU(void) { }
ITMLowLevelEngine_CPU::~ITMLowLevelEngine_CPU(void) { }

void ITMLowLevelEngine_CPU::CopyImage(ITMUChar4Image *image_out, const ITMUChar4Image *image_in) const
{
	Vector4u *dest = image_out->GetData(MEMORYDEVICE_CPU);
	const Vector4u *src = image_in->GetData(MEMORYDEVICE_CPU);

	memcpy(dest, src, image_in->dataSize * sizeof(Vector4u));
}

void ITMLowLevelEngine_CPU::CopyImage(ITMFloatImage *image_out, const ITMFloatImage *image_in) const
{
	float *dest = image_out->GetData(MEMORYDEVICE_CPU);
	const float *src = image_in->GetData(MEMORYDEVICE_CPU);

	memcpy(dest, src, image_in->dataSize * sizeof(float));
}

void ITMLowLevelEngine_CPU::CopyImage(ITMFloat4Image *image_out, const ITMFloat4Image *image_in) const
{
	Vector4f *dest = image_out->GetData(MEMORYDEVICE_CPU);
	const Vector4f *src = image_in->GetData(MEMORYDEVICE_CPU);

	memcpy(dest, src, image_in->dataSize * sizeof(Vector4f));
}

void ITMLowLevelEngine_CPU::ConvertColourToIntensity(ITMFloatImage *image_out, const ITMUChar4Image *image_in) const
{
	const Vector2i dims = image_in->noDims;
	image_out->ChangeDims(dims);

	float *dest = image_out->GetData(MEMORYDEVICE_CPU);
	const Vector4u *src = image_in->GetData(MEMORYDEVICE_CPU);

	for (int y = 0; y < dims.y; y++) for (int x = 0; x < dims.x; x++)
		convertColourToIntensity(dest, x, y, dims, src);
}

void ITMLowLevelEngine_CPU::FilterIntensity(ITMFloatImage *image_out, const ITMFloatImage *image_in) const
{
	Vector2i dims = image_in->noDims;

	image_out->ChangeDims(dims);
	image_out->Clear(0);

	const float *imageData_in = image_in->GetData(MEMORYDEVICE_CPU);
	float *imageData_out = image_out->GetData(MEMORYDEVICE_CPU);

	for (int y = 2; y < dims.y - 2; y++) for (int x = 2; x < dims.x - 2; x++)
		boxFilter2x2(imageData_out, x, y, dims, imageData_in, x, y, dims);
}

void ITMLowLevelEngine_CPU::FilterSubsample(ITMUChar4Image *image_out, const ITMUChar4Image *image_in) const
{
	Vector2i oldDims = image_in->noDims;
	Vector2i newDims; newDims.x = image_in->noDims.x / 2; newDims.y = image_in->noDims.y / 2;

	image_out->ChangeDims(newDims);

	const Vector4u *imageData_in = image_in->GetData(MEMORYDEVICE_CPU);
	Vector4u *imageData_out = image_out->GetData(MEMORYDEVICE_CPU);

	for (int y = 0; y < newDims.y; y++) for (int x = 0; x < newDims.x; x++)
		filterSubsample(imageData_out, x, y, newDims, imageData_in, oldDims);
}

void ITMLowLevelEngine_CPU::FilterSubsample(ITMFloatImage *image_out, const ITMFloatImage *image_in) const
{
	Vector2i oldDims = image_in->noDims;
	Vector2i newDims(image_in->noDims.x / 2, image_in->noDims.y / 2);

	image_out->ChangeDims(newDims);
	image_out->Clear();

	const float *imageData_in = image_in->GetData(MEMORYDEVICE_CPU);
	float *imageData_out = image_out->GetData(MEMORYDEVICE_CPU);

	for (int y = 1; y < newDims.y - 1; y++) for (int x = 1; x < newDims.x - 1; x++)
		boxFilter2x2(imageData_out, x, y, newDims, imageData_in, x * 2, y * 2, oldDims);
}

void ITMLowLevelEngine_CPU::FilterSubsampleWithHoles(ITMFloatImage *image_out, const ITMFloatImage *image_in) const
{
	Vector2i oldDims = image_in->noDims;
	Vector2i newDims; newDims.x = image_in->noDims.x / 2; newDims.y = image_in->noDims.y / 2;

	image_out->ChangeDims(newDims);

	const float *imageData_in = image_in->GetData(MEMORYDEVICE_CPU);
	float *imageData_out = image_out->GetData(MEMORYDEVICE_CPU);

	for (int y = 0; y < newDims.y; y++) for (int x = 0; x < newDims.x; x++)
		filterSubsampleWithHoles(imageData_out, x, y, newDims, imageData_in, oldDims);
}

void ITMLowLevelEngine_CPU::FilterSubsampleWithHoles(ITMFloat4Image *image_out, const ITMFloat4Image *image_in) const
{
	Vector2i oldDims = image_in->noDims;
	Vector2i newDims; newDims.x = image_in->noDims.x / 2; newDims.y = image_in->noDims.y / 2;

	image_out->ChangeDims(newDims);

	const Vector4f *imageData_in = image_in->GetData(MEMORYDEVICE_CPU);
	Vector4f *imageData_out = image_out->GetData(MEMORYDEVICE_CPU);

	for (int y = 0; y < newDims.y; y++) for (int x = 0; x < newDims.x; x++)
		filterSubsampleWithHoles(imageData_out, x, y, newDims, imageData_in, oldDims);
}

void ITMLowLevelEngine_CPU::GradientX(ITMShort4Image *grad_out, const ITMUChar4Image *image_in) const
{
	grad_out->ChangeDims(image_in->noDims);
	Vector2i imgSize = image_in->noDims;

	Vector4s *grad = grad_out->GetData(MEMORYDEVICE_CPU);
	const Vector4u *image = image_in->GetData(MEMORYDEVICE_CPU);

	memset(grad, 0, imgSize.x * imgSize.y * sizeof(Vector4s));

	for (int y = 1; y < imgSize.y - 1; y++) for (int x = 1; x < imgSize.x - 1; x++)
		gradientX(grad, x, y, image, imgSize);
}

void ITMLowLevelEngine_CPU::GradientY(ITMShort4Image *grad_out, const ITMUChar4Image *image_in) const
{
	grad_out->ChangeDims(image_in->noDims);
	Vector2i imgSize = image_in->noDims;

	Vector4s *grad = grad_out->GetData(MEMORYDEVICE_CPU);
	const Vector4u *image = image_in->GetData(MEMORYDEVICE_CPU);

	memset(grad, 0, imgSize.x * imgSize.y * sizeof(Vector4s));

	for (int y = 1; y < imgSize.y - 1; y++) for (int x = 1; x < imgSize.x - 1; x++)
		gradientY(grad, x, y, image, imgSize);
}

void ITMLowLevelEngine_CPU::GradientXY(ITMFloat2Image *grad_out, const ITMFloatImage *image_in) const
{
	Vector2i imgSize = image_in->noDims;
	grad_out->ChangeDims(imgSize);
	grad_out->Clear();

	Vector2f *grad = grad_out->GetData(MEMORYDEVICE_CPU);
	const float *image = image_in->GetData(MEMORYDEVICE_CPU);

	for (int y = 1; y < imgSize.y - 1; y++) for (int x = 1; x < imgSize.x - 1; x++)
		gradientXY(grad, x, y, image, imgSize);
}

int ITMLowLevelEngine_CPU::CountValidDepths(const ITMFloatImage *image_in) const
{
	int noValidPoints = 0;
	const float *imageData_in = image_in->GetData(MEMORYDEVICE_CPU);

	for (int i = 0; i < image_in->noDims.x * image_in->noDims.y; ++i) if (imageData_in[i] > 0.0) noValidPoints++;

	return noValidPoints;
}
