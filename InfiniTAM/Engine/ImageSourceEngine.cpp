// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM

#include "ImageSourceEngine.h"

#include "../ITMLib/Objects/Camera/ITMCalibIO.h"
#include "../Utils/FileUtils.h"

#include <stdexcept>
#include <stdio.h>

using namespace InfiniTAM::Engine;
using namespace ITMLib;

ImageSourceEngine::ImageSourceEngine(const char *calibFilename)
{
	readRGBDCalib(calibFilename, calib);
}

ImageMaskPathGenerator::ImageMaskPathGenerator(const char *rgbImageMask_, const char *depthImageMask_)
{
	strncpy(rgbImageMask, rgbImageMask_, BUF_SIZE);
	strncpy(depthImageMask, depthImageMask_, BUF_SIZE);
}

std::string ImageMaskPathGenerator::getRgbImagePath(size_t currentFrameNo) const
{
	char str[BUF_SIZE];
	sprintf(str, rgbImageMask, currentFrameNo);
	return std::string(str);
}

std::string ImageMaskPathGenerator::getDepthImagePath(size_t currentFrameNo) const
{
	char str[BUF_SIZE];
	sprintf(str, depthImageMask, currentFrameNo);
	return std::string(str);
}

ImageListPathGenerator::ImageListPathGenerator(const std::vector<std::string>& rgbImagePaths_, const std::vector<std::string>& depthImagePaths_)
	: depthImagePaths(depthImagePaths_),
	  rgbImagePaths(rgbImagePaths_)
{
	if(rgbImagePaths.size() != depthImagePaths.size()) throw std::runtime_error("error: the rgb and depth image path lists do not have the same size");
}

std::string ImageListPathGenerator::getRgbImagePath(size_t currentFrameNo) const
{
	return currentFrameNo < imageCount() ? rgbImagePaths[currentFrameNo] : "";
}

std::string ImageListPathGenerator::getDepthImagePath(size_t currentFrameNo) const
{
	return currentFrameNo < imageCount() ? depthImagePaths[currentFrameNo] : "";
}

size_t ImageListPathGenerator::imageCount() const
{
	return rgbImagePaths.size();
}

template <typename PathGenerator>
ImageFileReader<PathGenerator>::ImageFileReader(const char *calibFilename, const PathGenerator& pathGenerator_)
	: ImageSourceEngine(calibFilename),
	  pathGenerator(pathGenerator_)
{
	currentFrameNo = 0;
	cachedFrameNo = -1;

	cached_rgb = NULL;
	cached_depth = NULL;
}

template <typename PathGenerator>
ImageFileReader<PathGenerator>::~ImageFileReader()
{
	delete cached_rgb;
	delete cached_depth;
}

template <typename PathGenerator>
void ImageFileReader<PathGenerator>::loadIntoCache(void)
{
	if (currentFrameNo == cachedFrameNo) return;
	cachedFrameNo = currentFrameNo;

	//TODO> make nicer
	cached_rgb = new ITMUChar4Image(true, false);
	cached_depth = new ITMShortImage(true, false);

	std::string rgbPath = pathGenerator.getRgbImagePath(currentFrameNo);
	if (!ReadImageFromFile(cached_rgb, rgbPath.c_str()))
	{
		delete cached_rgb; cached_rgb = NULL;
		printf("error reading file '%s'\n", rgbPath.c_str());
	}

	std::string depthPath = pathGenerator.getDepthImagePath(currentFrameNo);
	if (!ReadImageFromFile(cached_depth, depthPath.c_str()))
	{
		delete cached_depth; cached_depth = NULL;
		printf("error reading file '%s'\n", depthPath.c_str());
	}
}

template <typename PathGenerator>
bool ImageFileReader<PathGenerator>::hasMoreImages(void)
{
	loadIntoCache();
	return ((cached_rgb!=NULL)&&(cached_depth!=NULL));
}

template <typename PathGenerator>
void ImageFileReader<PathGenerator>::getImages(ITMUChar4Image *rgb, ITMShortImage *rawDepth)
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

		std::string rgbPath = pathGenerator.getRgbImagePath(currentFrameNo);
		if (!ReadImageFromFile(rgb, rgbPath.c_str())) printf("error reading file '%s'\n", rgbPath.c_str());

		std::string depthPath = pathGenerator.getDepthImagePath(currentFrameNo);
		if (!ReadImageFromFile(rawDepth, depthPath.c_str())) printf("error reading file '%s'\n", depthPath.c_str());
	}

	++currentFrameNo;
}

template <typename PathGenerator>
Vector2i ImageFileReader<PathGenerator>::getDepthImageSize(void)
{
	loadIntoCache();
	return cached_depth->noDims;
}

template <typename PathGenerator>
Vector2i ImageFileReader<PathGenerator>::getRGBImageSize(void)
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

template class ImageFileReader<ImageMaskPathGenerator>;
template class ImageFileReader<ImageListPathGenerator>;
