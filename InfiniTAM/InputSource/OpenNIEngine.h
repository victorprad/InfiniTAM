// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM

#pragma once

#include "ImageSourceEngine.h"

#if (!defined USING_CMAKE) && (defined _MSC_VER)
#pragma comment(lib, "OpenNI2")
#endif

namespace InfiniTAM
{
	namespace Engine
	{
		class OpenNIEngine : public ImageSourceEngine
		{
		private:
			class PrivateData;
			PrivateData *data;
			Vector2i imageSize_rgb, imageSize_d;
			bool colorAvailable, depthAvailable;
		public:
			OpenNIEngine(const char *calibFilename, const char *deviceURI = NULL, const bool useInternalCalibration = false,
				Vector2i imageSize_rgb = Vector2i(640, 480), Vector2i imageSize_d = Vector2i(640, 480));
			~OpenNIEngine();

			bool hasMoreImages(void);
			void getImages(ITMUChar4Image *rgb, ITMShortImage *rawDepth);
			Vector2i getDepthImageSize(void);
			Vector2i getRGBImageSize(void);
		};
	}
}

