// Copyright 2014 Isis Innovation Limited and the authors of InfiniTAM

#include <cstdlib>

#include "Engine/UIEngine.h"
#include "Engine/ImageSourceEngine.h"
#include "Engine/OpenNIEngine.h"
#include "Engine/Kinect2Engine.h"

using namespace InfiniTAM::Engine;

int main(int argc, char** argv)
try
{
	const char *calibFile = "./Files/Teddy/calib.txt";
	const char *imagesource_part1 = NULL;
	const char *imagesource_part2 = NULL;

	int arg = 1;
	do {
		if (argv[arg] != NULL) calibFile = argv[arg]; else break;
		++arg;
		if (argv[arg] != NULL) imagesource_part1 = argv[arg]; else break;
		++arg;
		if (argv[arg] != NULL) imagesource_part2 = argv[arg]; else break;
	} while (false);

	if (arg == 1) {
		printf("usage: %s [<calibfile> [<imagesource>] ]\n"
		       "  <calibfile>   : path to a file containing intrinsic calibration parameters\n"
		       "  <imagesource> : either one argument to specify OpenNI device ID\n"
		       "                  or two arguments specifying rgb and depth file masks\n"
		       "\n"
		       "examples:\n"
		       "  %s ./Files/Teddy/calib.txt ./Files/Teddy/Frames/%%04i.ppm ./Files/Teddy/Frames/%%04i.pgm\n"
		       "  %s ./Files/Teddy/calib.txt\n\n", argv[0], argv[0], argv[0]);
	}

	printf("initialising ...\n");
	ITMLibSettings *internalSettings = new ITMLibSettings();

	ImageSourceEngine *imageSource;
	printf("using calibration file: %s\n", calibFile);
	if (imagesource_part2 == NULL) {
		printf("using OpenNI device: %s\n", (imagesource_part1==NULL)?"<OpenNI default device>":imagesource_part1);
		imageSource = new OpenNIEngine(calibFile, imagesource_part1);
		if (imageSource->getDepthImageSize().x == 0) {
			delete imageSource;
			printf("trying MS Kinect device\n");
			imageSource = new Kinect2Engine(calibFile);
		}
	} else {
		printf("using rgb images: %s\nusing depth images: %s\n", imagesource_part1, imagesource_part2);
		imageSource = new ImageFileReader(calibFile, imagesource_part1, imagesource_part2);
	}

	ITMMainEngine *mainEngine = new ITMMainEngine(internalSettings, &imageSource->calib, imageSource->getRGBImageSize(), imageSource->getDepthImageSize());

	UIEngine::Instance()->Initialise(argc, argv, imageSource, mainEngine, "./Files/Out");
	UIEngine::Instance()->Run();
	UIEngine::Instance()->Shutdown();

	delete mainEngine;
	delete internalSettings;
	delete imageSource;
	return 0;
}
catch(std::exception& e)
{
	std::cerr << e.what() << '\n';
	return EXIT_FAILURE;
}

