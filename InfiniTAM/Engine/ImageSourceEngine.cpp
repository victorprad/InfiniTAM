// Copyright 2014 Isis Innovation Limited and the authors of InfiniTAM

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
	if ((cached_rgb != NULL) || (cached_depth != NULL)) return;

	cached_rgb = new ITMUChar4Image();
	cached_depth = new ITMShortImage();

	char str[2048];
	sprintf(str, rgbImageMask, currentFrameNo);
	if (!ReadImageFromFile(cached_rgb, str)) {
		delete cached_rgb; cached_rgb = NULL;
		printf("error reading file '%s'\n", str);
	}
	sprintf(str, depthImageMask, currentFrameNo);
	if (!ReadImageFromFile(cached_depth, str)) {
		delete cached_depth; cached_depth = NULL;
		printf("error reading file '%s'\n", str);
	}
}

bool ImageFileReader::hasMoreImages(void)
{
	loadIntoCache();
	return ((cached_rgb!=NULL)&&(cached_depth!=NULL));
}

void ImageFileReader::getImages(ITMView *out)
{
	bool bUsedCache = false;
	if (cached_rgb != NULL) {
		out->rgb->SetFrom(cached_rgb);
		delete cached_rgb;
		cached_rgb = NULL;
		bUsedCache = true;
	}
	if (cached_depth != NULL) {
		out->rawDepth->SetFrom(cached_depth);
		delete cached_depth;
		cached_depth = NULL;
		bUsedCache = true;
	}

	if (!bUsedCache) {
		char str[2048];
		sprintf(str, rgbImageMask, currentFrameNo);
		if (!ReadImageFromFile(out->rgb, str)) {
			printf("error reading file '%s'\n", str);
		}

		sprintf(str, depthImageMask, currentFrameNo);
		if (!ReadImageFromFile(out->rawDepth, str)) {
			printf("error reading file '%s'\n", str);
		}
	}

	if (calib.disparityCalib.params.y == 0) out->inputImageType = ITMView::InfiniTAM_SHORT_DEPTH_IMAGE;
	else out->inputImageType = ITMView::InfiniTAM_DISPARITY_IMAGE;

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
	return cached_rgb->noDims;
}

