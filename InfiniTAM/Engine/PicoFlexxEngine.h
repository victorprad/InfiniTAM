// Copyright 2017 Akos Maroy

#pragma once

#include "ImageSourceEngine.h"

#ifdef COMPILE_WITH_LibRoyale

#if (!defined USING_CMAKE) && (defined _MSC_VER)
#pragma comment(lib, "royale")
#endif

#endif

namespace InfiniTAM
{
	namespace Engine
	{
		class PicoFlexxEngine : public ImageSourceEngine
		{
		private:
			class PrivateData;
			PrivateData *data;
			Vector2i imageSize_rgb, imageSize_d;
			bool colorAvailable, depthAvailable;

		public:
			PicoFlexxEngine(const char *calibFilename, const char *deviceURI = NULL, const bool useInternalCalibration = false,
				Vector2i imageSize_rgb = Vector2i(224, 171), Vector2i imageSize_d = Vector2i(224, 171));
			~PicoFlexxEngine();

			bool hasMoreImages(void);
			void getImages(ITMUChar4Image *rgb, ITMShortImage *rawDepth);
			Vector2i getDepthImageSize(void);
			Vector2i getRGBImageSize(void);
		};
	}
}

