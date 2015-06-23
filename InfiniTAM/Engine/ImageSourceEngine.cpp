// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM

#include "ImageSourceEngine.h"

#include "../Utils/FileUtils.h"

#include <stdio.h>

using namespace InfiniTAM::Engine;

ImageSourceEngine::ImageSourceEngine(const char *calibFilename)
{
	readRGBDCalib(calibFilename, calib);
}

ImageFileReader::ImageFileReader(const char *calibFilename, const char *rgbImageMask, const char *depthImageMask)
	: ImageSourceEngine(calibFilename)
{
	strncpy(this->rgbImageMask, rgbImageMask, BUF_SIZE);
	strncpy(this->depthImageMask, depthImageMask, BUF_SIZE);

	currentFrameNo = 0;
	cachedFrameNo = -1;

	cached_rgb = NULL;
	cached_depth = NULL;
}

ImageFileReader::~ImageFileReader()
{
	delete cached_rgb;
	delete cached_depth;
}

void ImageFileReader::loadIntoCache(void)
{
	if (currentFrameNo == cachedFrameNo) return;
	cachedFrameNo = currentFrameNo;

	//TODO> make nicer
	cached_rgb = new ITMUChar4Image(true, false); 
	cached_depth = new ITMShortImage(true, false);

	char str[2048];

	sprintf(str, rgbImageMask, currentFrameNo);
	if (!ReadImageFromFile(cached_rgb, str)) 
	{
		delete cached_rgb; cached_rgb = NULL;
		printf("error reading file '%s'\n", str);
	}

	sprintf(str, depthImageMask, currentFrameNo);
	if (!ReadImageFromFile(cached_depth, str)) 
	{
		delete cached_depth; cached_depth = NULL;
		printf("error reading file '%s'\n", str);
	}
}

bool ImageFileReader::hasMoreImages(void)
{
	loadIntoCache();
	return ((cached_rgb!=NULL)&&(cached_depth!=NULL));
}

void ImageFileReader::getImages(ITMUChar4Image *rgb, ITMShortImage *rawDepth)
{
	bool bUsedCache = false;
	if (cached_rgb != NULL) {
		rgb->SetFrom(cached_rgb, ORUtils::MemoryBlock<Vector4u>::CPU_TO_CPU);
		delete cached_rgb;
		cached_rgb = NULL;
		bUsedCache = true;
	}
	if (cached_depth != NULL) {
		rawDepth->SetFrom(cached_depth, ORUtils::MemoryBlock<short>::CPU_TO_CPU);
		delete cached_depth;
		cached_depth = NULL;
		bUsedCache = true;
	}

	if (!bUsedCache) {
		char str[2048];

		sprintf(str, rgbImageMask, currentFrameNo);
		if (!ReadImageFromFile(rgb, str)) printf("error reading file '%s'\n", str);

		sprintf(str, depthImageMask, currentFrameNo);
		if (!ReadImageFromFile(rawDepth, str)) printf("error reading file '%s'\n", str);
	}

	++currentFrameNo;
}

Vector2i ImageFileReader::getDepthImageSize(void)
{
	loadIntoCache();
	return cached_depth->noDims;
}

Vector2i ImageFileReader::getRGBImageSize(void)
{
	loadIntoCache();
	if (cached_rgb != NULL) return cached_rgb->noDims;
	return cached_depth->noDims;
}

CalibSource::CalibSource(const char *calibFilename, Vector2i setImageSize, float ratio)
	: ImageSourceEngine(calibFilename)
{
	this->imgSize = setImageSize;
	this->ResizeIntrinsics(calib.intrinsics_d, ratio);
	this->ResizeIntrinsics(calib.intrinsics_rgb, ratio);
}

void CalibSource::ResizeIntrinsics(ITMIntrinsics &intrinsics, float ratio)
{
	intrinsics.projectionParamsSimple.fx *= ratio;
	intrinsics.projectionParamsSimple.fy *= ratio;
	intrinsics.projectionParamsSimple.px *= ratio;
	intrinsics.projectionParamsSimple.py *= ratio;
	intrinsics.projectionParamsSimple.all *= ratio;
}

RawFileReader::RawFileReader(const char *calibFilename, const char *rgbImageMask, const char *depthImageMask, Vector2i setImageSize, float ratio) 
	: ImageSourceEngine(calibFilename)
{
	this->imgSize = setImageSize;
	this->ResizeIntrinsics(calib.intrinsics_d, ratio);
	this->ResizeIntrinsics(calib.intrinsics_rgb, ratio);
	
	strncpy(this->rgbImageMask, rgbImageMask, BUF_SIZE);
	strncpy(this->depthImageMask, depthImageMask, BUF_SIZE);

	currentFrameNo = 0;
	cachedFrameNo = -1;

	cached_rgb = NULL;
	cached_depth = NULL;
}

void RawFileReader::ResizeIntrinsics(ITMIntrinsics &intrinsics, float ratio)
{
	intrinsics.projectionParamsSimple.fx *= ratio;
	intrinsics.projectionParamsSimple.fy *= ratio;
	intrinsics.projectionParamsSimple.px *= ratio;
	intrinsics.projectionParamsSimple.py *= ratio;
	intrinsics.projectionParamsSimple.all *= ratio;
}

void RawFileReader::loadIntoCache(void)
{
	if (currentFrameNo == cachedFrameNo) return;
	cachedFrameNo = currentFrameNo;

	//TODO> make nicer
	cached_rgb = new ITMUChar4Image(imgSize, MEMORYDEVICE_CPU);
	cached_depth = new ITMShortImage(imgSize, MEMORYDEVICE_CPU);

	char str[2048]; FILE *f; bool success = false;

	sprintf(str, rgbImageMask, currentFrameNo);

	f = fopen(str, "rb");
	if (f)
	{
		size_t tmp = fread(cached_rgb->GetData(MEMORYDEVICE_CPU), sizeof(Vector4u), imgSize.x * imgSize.y, f);
		fclose(f);
		if (tmp == (size_t)imgSize.x * imgSize.y) success = true;
	}
	if (!success)
	{
		delete cached_rgb; cached_rgb = NULL;
		printf("error reading file '%s'\n", str);
	}

	sprintf(str, depthImageMask, currentFrameNo); success = false;
	f = fopen(str, "rb");
	if (f)
	{
		size_t tmp = fread(cached_depth->GetData(MEMORYDEVICE_CPU), sizeof(short), imgSize.x * imgSize.y, f);
		fclose(f);
		if (tmp == (size_t)imgSize.x * imgSize.y) success = true;
	}
	if (!success)
	{
		delete cached_depth; cached_depth = NULL;
		printf("error reading file '%s'\n", str);
	}
}


bool RawFileReader::hasMoreImages(void)
{
	loadIntoCache(); 

	return ((cached_rgb != NULL) || (cached_depth != NULL));
}

void RawFileReader::getImages(ITMUChar4Image *rgb, ITMShortImage *rawDepth)
{
	bool bUsedCache = false;

	if (cached_rgb != NULL)
	{
		rgb->SetFrom(cached_rgb, ORUtils::MemoryBlock<Vector4u>::CPU_TO_CPU);
		delete cached_rgb;
		cached_rgb = NULL;
		bUsedCache = true;
	}

	if (cached_depth != NULL)
	{
		rawDepth->SetFrom(cached_depth, ORUtils::MemoryBlock<short>::CPU_TO_CPU);
		delete cached_depth;
		cached_depth = NULL;
		bUsedCache = true;
	}

	if (!bUsedCache) this->loadIntoCache();

	++currentFrameNo;
}
