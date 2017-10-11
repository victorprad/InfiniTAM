// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#include "ImageSourceEngine.h"

namespace InputSource {

class FFMPEGReader : public BaseImageSourceEngine
{
	public:
	class PrivateData;

	FFMPEGReader(const char *calibFilename, const char *filename1, const char *filename2 = NULL);
	~FFMPEGReader(void);

	bool hasMoreImages(void) const;
	void getImages(ITMUChar4Image *rgb, ITMShortImage *rawDepth);

	Vector2i getDepthImageSize(void) const;
	Vector2i getRGBImageSize(void) const;

	private:
	PrivateData *mData1;
	PrivateData *mData2;
	bool isValid;
};

}
