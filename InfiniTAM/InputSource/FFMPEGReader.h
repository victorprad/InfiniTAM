// Copyright 2016 Isis Innovation Limited and the authors of InfiniTAM

#pragma once

#include "ImageSourceEngine.h"

namespace InputSource {

class FFMPEGReader : public ImageSourceEngine
{
	public:
	class PrivateData;

	FFMPEGReader(const char *calibFilename, const char *filename1, const char *filename2 = NULL);
	~FFMPEGReader(void);

	bool hasMoreImages(void);
	void getImages(ITMUChar4Image *rgb, ITMShortImage *rawDepth);

	Vector2i getDepthImageSize(void);
	Vector2i getRGBImageSize(void);

	private:
	PrivateData *mData1;
	PrivateData *mData2;
	bool isValid;
};

}
