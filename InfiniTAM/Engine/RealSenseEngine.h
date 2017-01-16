// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM

#pragma once

#include "ImageSourceEngine.h"

#ifdef COMPILE_WITH_RealSense

#if (!defined USING_CMAKE) && (defined _MSC_VER)
#ifdef _DEBUG
#pragma comment(lib, "libpxcmd_d")
#else
#pragma comment(lib, "libpxcmd")
#endif 
#endif

#endif

namespace InfiniTAM
{
	namespace Engine
	{
		class RealSenseEngine : public ImageSourceEngine
		{
		private:
			class PrivateData;
			PrivateData *data; bool dataAvailable;

			Vector2i imageSize_rgb, imageSize_d;
		public:
			RealSenseEngine(const char *calibFilename, Vector2i imageSize_rgb = Vector2i(640, 480), Vector2i imageSize_d = Vector2i(640, 480));
			~RealSenseEngine();

			bool hasMoreImages(void);
			void getImages(ITMUChar4Image *rgb, ITMShortImage *rawDepth);
			Vector2i getDepthImageSize(void);
			Vector2i getRGBImageSize(void);
		};
	}
}

