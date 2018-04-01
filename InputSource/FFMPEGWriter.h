// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../ORUtils/ImageTypes.h"

namespace InputSource {

class FFMPEGWriter
{
	public:
	class PrivateData;

	FFMPEGWriter(void);
	~FFMPEGWriter(void);

	bool open(const char *filename, int size_x, int size_y, bool isDepth, int fps);
	bool writeFrame(ORUChar4Image *rgbImage);
	bool writeFrame(ORShortImage *depthImage);
	bool close(void);

	bool isOpen(void) const;

	private:
	PrivateData *mData;
	int counter;
};

}
