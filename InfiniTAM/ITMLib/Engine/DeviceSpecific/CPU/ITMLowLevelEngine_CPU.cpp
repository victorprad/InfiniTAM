// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM

#include "ITMLowLevelEngine_CPU.h"

#include "../../DeviceAgnostic/ITMLowLevelEngine.h"

using namespace ITMLib::Engine;

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

	memset(grad, 0, imgSize.x * imgSize.y * sizeof(Vector3s));

	for (int y = 1; y < imgSize.y - 1; y++) for (int x = 1; x < imgSize.x - 1; x++)
		gradientX(grad, x, y, image, imgSize);
}

void ITMLowLevelEngine_CPU::GradientY(ITMShort4Image *grad_out, const ITMUChar4Image *image_in) const
{
	grad_out->ChangeDims(image_in->noDims);
	Vector2i imgSize = image_in->noDims;

	Vector4s *grad = grad_out->GetData(MEMORYDEVICE_CPU);
	const Vector4u *image = image_in->GetData(MEMORYDEVICE_CPU);

	memset(grad, 0, imgSize.x * imgSize.y * sizeof(Vector3s));

	for (int y = 1; y < imgSize.y - 1; y++) for (int x = 1; x < imgSize.x - 1; x++)
		gradientY(grad, x, y, image, imgSize);
}