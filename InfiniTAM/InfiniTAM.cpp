// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM

#include <cstdlib>

#include "Engine/UIEngine.h"
#include "Engine/ImageSourceEngine.h"

#include "Engine/OpenNIEngine.h"
#include "Engine/Kinect2Engine.h"
#include "Engine/LibUVCEngine.h"
#include "Engine/RealSenseEngine.h"
#include "Engine/PicoFlexxEngine.h"

using namespace InfiniTAM::Engine;

/** Create a default source of depth images from a list of command line
    arguments. Typically, @para arg1 would identify the calibration file to
    use, @para arg2 the colour images, @para arg3 the depth images and
    @para arg4 the IMU images. If images are omitted, some live sources will
    be tried.
*/
static void CreateDefaultImageSource(ImageSourceEngine* & imageSource, IMUSourceEngine* & imuSource, const char *arg1, const char *arg2, const char *arg3, const char *arg4)
{
	const char *calibFile = arg1;
	const char *filename1 = arg2;
	const char *filename2 = arg3;
	const char *filename_imu = arg4;

	printf("using calibration file: %s\n", calibFile);

	if (filename2 != NULL)
	{
		printf("using rgb images: %s\nusing depth images: %s\n", filename1, filename2);
		if (filename_imu == NULL)
		{
			imageSource = new ImageFileReader(calibFile, filename1, filename2);
		}
		else
		{
			printf("using imu data: %s\n", filename_imu);
			imageSource = new RawFileReader(calibFile, filename1, filename2, Vector2i(320, 240), 0.5f);
			imuSource = new IMUSourceEngine(filename_imu);
		}
	}

	if (imageSource == NULL)
	{
		printf("trying OpenNI device: %s\n", (filename1==NULL)?"<OpenNI default device>":filename1);
		imageSource = new OpenNIEngine(calibFile, filename1);
		if (imageSource->getDepthImageSize().x == 0)
		{
			delete imageSource;
			imageSource = NULL;
		}
	}
	if (imageSource == NULL)
	{
		printf("trying UVC device\n");
		imageSource = new LibUVCEngine(calibFile);
		if (imageSource->getDepthImageSize().x == 0)
		{
			delete imageSource;
			imageSource = NULL;
		}
	}
	if (imageSource == NULL)
	{
		printf("trying RealSense device\n");
		imageSource = new RealSenseEngine(calibFile);
		if (imageSource->getDepthImageSize().x == 0)
		{
			delete imageSource;
			imageSource = NULL;
		}
	}
	if (imageSource == NULL)
	{
		printf("trying MS Kinect 2 device\n");
		imageSource = new Kinect2Engine(calibFile);
		if (imageSource->getDepthImageSize().x == 0)
		{
			delete imageSource;
			imageSource = NULL;
		}
	}
	if (imageSource == NULL)
	{
		printf("trying PMD PicoFlexx device\n");
		imageSource = new PicoFlexxEngine(calibFile);
		if (imageSource->getDepthImageSize().x == 0)
		{
			delete imageSource;
			imageSource = NULL;
		}
	}

	// this is a hack to ensure backwards compatibility in certain configurations
	if (imageSource == NULL) return;
	if (imageSource->calib.disparityCalib.params == Vector2f(0.0f, 0.0f))
	{
		imageSource->calib.disparityCalib.type = ITMDisparityCalib::TRAFO_AFFINE;
		imageSource->calib.disparityCalib.params = Vector2f(1.0f/1000.0f, 0.0f);
	}
}

int main(int argc, char** argv)
try
{
	const char *arg1 = "";
	const char *arg2 = NULL;
	const char *arg3 = NULL;
	const char *arg4 = NULL;

	int arg = 1;
	do {
		if (argv[arg] != NULL) arg1 = argv[arg]; else break;
		++arg;
		if (argv[arg] != NULL) arg2 = argv[arg]; else break;
		++arg;
		if (argv[arg] != NULL) arg3 = argv[arg]; else break;
		++arg;
		if (argv[arg] != NULL) arg4 = argv[arg]; else break;
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
	ImageSourceEngine *imageSource = NULL;
	IMUSourceEngine *imuSource = NULL;

	CreateDefaultImageSource(imageSource, imuSource, arg1, arg2, arg3, arg4);
	if (imageSource==NULL)
	{
		std::cout << "failed to open any image stream" << std::endl;
		return -1;
	}

	ITMLibSettings *internalSettings = new ITMLibSettings();
	ITMMainEngine *mainEngine = new ITMMainEngine(internalSettings, &imageSource->calib, imageSource->getRGBImageSize(), imageSource->getDepthImageSize());

	UIEngine::Instance()->Initialise(argc, argv, imageSource, imuSource, mainEngine, "./Files/Out", internalSettings->deviceType);
	UIEngine::Instance()->Run();
	UIEngine::Instance()->Shutdown();

	delete mainEngine;
	delete internalSettings;
	delete imageSource;
	if (imuSource != NULL) delete imuSource;
	return 0;
}
catch(std::exception& e)
{
	std::cerr << e.what() << '\n';
	return EXIT_FAILURE;
}

