// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../ITMLib/Utils/ITMImageTypes.h"

namespace InputSource {

class FFMPEGWriter
{
	public:
	class PrivateData;

	FFMPEGWriter(void);
	~FFMPEGWriter(void);

	bool open(const char *filename, int size_x, int size_y, bool isDepth, int fps);
	bool writeFrame(ITMUChar4Image *rgbImage);
	bool writeFrame(ITMShortImage *depthImage);
	bool close(void);

	bool isOpen(void) const;

	private:
	PrivateData *mData;
	int counter;
};

}
