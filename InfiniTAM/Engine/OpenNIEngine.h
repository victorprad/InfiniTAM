// Copyright 2014 Isis Innovation Limited and the authors of InfiniTAM

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
			bool colorAvailable, depthAvailable;
		public:
			OpenNIEngine(const char *calibFilename, const char *deviceURI = NULL, const bool useInternalCalibration = false);
			~OpenNIEngine();

			bool hasMoreImages(void);
			void getImages(ITMView *out);
			Vector2i getDepthImageSize(void);
			Vector2i getRGBImageSize(void);
		};
	}
}

