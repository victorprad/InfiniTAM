// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM

#pragma once

#include "ImageSourceEngine.h"

namespace InfiniTAM
{
	namespace Engine
	{
		/** This class provides an interface for reading from the
		    Microsoft Kinect 2 via the provided API. There is currently
		    no appropriate postprocessing to compensate for the
		    artefacts in the depth images of the Kinect 2, so the
		    results look a bit weird on objects with reflections.
		*/
		class Kinect2Engine : public ImageSourceEngine
		{
		private:
			class PrivateData;
			PrivateData *data;

			Vector2i imageSize_d, imageSize_rgb;
			bool colorAvailable, depthAvailable;
		public:
			Kinect2Engine(const char *calibFilename);
			~Kinect2Engine();

			bool hasMoreImages(void);
			void getImages(ITMUChar4Image *rgbImage, ITMShortImage *rawDepthImage);
			Vector2i getDepthImageSize(void);
			Vector2i getRGBImageSize(void);
		};
	}
}

