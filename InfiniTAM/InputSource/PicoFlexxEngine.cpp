// Copyright 2017 Akos Maroy and the authors of InfiniTAM

#include "PicoFlexxEngine.h"

#ifdef COMPILE_WITH_LibRoyale

#include <cstdio>
#include <stdexcept>
#include <algorithm>
#include <mutex>

#ifdef WIN32
#include <windows.h>
#define ROYALE_SLEEP_MS(x) Sleep(x)
#else
#include <unistd.h>
#define ROYALE_SLEEP_MS(x) usleep(x * 1000)
#endif

#include <royale.hpp>

using namespace InputSource;
using namespace royale;
using namespace std;

class PicoFlexxEngine::PrivateData : public IDepthDataListener
{
public:
	PicoFlexxEngine *engine;
	std::unique_ptr<ICameraDevice> cameraDevice;
	vector<Vector4u> rgbImage;
	vector<short> depthImage;
	mutex mtx;

	explicit PrivateData(PicoFlexxEngine *pfe) : engine(pfe), depthImage(0) {}
	~PrivateData() {}

	void onNewData(const DepthData *data);
};


void PicoFlexxEngine::PrivateData::onNewData(const DepthData *data)
{
	lock_guard<mutex> lock(mtx);

	// handle the grayscale image
	engine->imageSize_rgb = Vector2i(data->width, data->height);

	rgbImage.clear();
	rgbImage.reserve(data->points.size());

	// copy grayscale image data into an RGB data set
	for (size_t pointId = 0; pointId < data->points.count(); pointId++)
	{
		// PicoFlexx seems to return 0 when bright, and up to FF0 when totally dark
		// convert this into a value into an 8 bit value
		unsigned char charValue = data->points[pointId].grayValue >> 4;
		Vector4u pixel;
		pixel.x = pixel.y = pixel.z = pixel.w = charValue;
		this->rgbImage.push_back(pixel);
	}

	// handle the depth image
	engine->imageSize_d = Vector2i(data->width, data->height);

	depthImage.clear();
	depthImage.reserve(data->points.size());

	// copy depth image data, converting meters in float to millimeters in short
	for (size_t pointId = 0; pointId < data->points.count(); pointId++)
	{
		// do not copy if confidence is low. confidence is 0 when bad, 255 when good
		// it seems there are no intermediate values, still let's cut at 128
		const DepthPoint &dd = data->points[pointId];
		this->depthImage.push_back(dd.depthConfidence > 128 ? (short)(dd.z * 1000.0) : 0);
	}
}

PicoFlexxEngine::PicoFlexxEngine(const char *calibFilename, const char *deviceURI, const bool useInternalCalibration,
	Vector2i requested_imageSize_rgb, Vector2i requested_imageSize_d)
	: BaseImageSourceEngine(calibFilename)
{
	this->imageSize_d = Vector2i(0, 0);
	this->imageSize_rgb = Vector2i(0, 0);

	data = new PrivateData(this);

	// the camera manager will query for a connected camera
	{
		CameraManager manager;

		royale::Vector<royale::String> camlist = manager.getConnectedCameraList();
		cout << "Detected " << camlist.size() << " camera(s)." << endl;
		if (!camlist.empty())
		{
			cout << "CamID for first device: " << camlist.at(0).c_str() << " with a length of (" << camlist.at(0).length() << ")" << endl;
			data->cameraDevice = manager.createCamera(camlist[0]);
		}
	}
	// the camera device is now available and CameraManager can be deallocated here

	if (data->cameraDevice == nullptr)
	{
		cerr << "Cannot create the camera device" << endl;
		return;
	}

	// IMPORTANT: call the initialize method before working with the camera device
	if (data->cameraDevice->initialize() != CameraStatus::SUCCESS)
	{
		cerr << "Cannot initialize the camera device" << endl;
		return;
	}

	// set camera parameters
	LensParameters lensParams;
	if (data->cameraDevice->getLensParameters(lensParams) != CameraStatus::SUCCESS) {
		cerr << "Cannot determine lens parameters" << endl;
		return;
	}

	this->calib.intrinsics_d.SetFrom(lensParams.focalLength.first, lensParams.focalLength.second,
		lensParams.principalPoint.first, lensParams.principalPoint.second);
	this->calib.intrinsics_rgb.SetFrom(lensParams.focalLength.first, lensParams.focalLength.second,
		lensParams.principalPoint.first, lensParams.principalPoint.second);

	royale::Vector<royale::String> useCases;
	royale::CameraStatus status = data->cameraDevice->getUseCases(useCases);

	if (status != CameraStatus::SUCCESS || useCases.empty())
	{
		cerr << "No use cases are available" << endl;
		cerr << "getUseCases() returned: " << getErrorString(status) << endl;
		return;
	}

	// list the available use cases
	cout << "Available Pico Flexx use cases:" << endl;
	for (size_t caseId = 0; caseId < useCases.count(); caseId++)
		cout << useCases[caseId] << endl;

	// register a data listener
	if (data->cameraDevice->registerDataListener(data) != CameraStatus::SUCCESS)
	{
		cerr << "Error registering data listener" << endl;
		return;
	}

	// set an operation mode
	// TODO allow setting this from the command line
	if (data->cameraDevice->setUseCase("MODE_9_10FPS_1000") != CameraStatus::SUCCESS)
	{
		cerr << "Error setting use case" << endl;
		return;
	}

	// start capture mode
	if (data->cameraDevice->startCapture() != CameraStatus::SUCCESS)
	{
		cerr << "Error starting the capturing" << endl;
		return;
	}

	// waiting for the capture to start and 'data' to be populated
	ROYALE_SLEEP_MS(1000);
}

PicoFlexxEngine::~PicoFlexxEngine()
{
	if (data)
	{
		// stop capture mode
		if (data->cameraDevice)
			if (data->cameraDevice->stopCapture() != CameraStatus::SUCCESS) cerr << "Error stopping the capturing" << endl;

		delete data;
	}
}

void PicoFlexxEngine::getImages(ITMUChar4Image *rgbImage, ITMShortImage *rawDepthImage)
{
	lock_guard<mutex> lock(data->mtx);

	// copy the color info
#ifdef PROVIDE_RGB
	Vector4u *rgb = rgbImage->GetData(MEMORYDEVICE_CPU);
	if (data->rgbImage.size()) memcpy(rgb, data->rgbImage.data(), rgbImage->dataSize * sizeof(Vector4u));
	else memset(rgb, 0, rgbImage->dataSize * sizeof(Vector4u));
#else
	imageSize_rgb = Vector2i(0, 0);
#endif

	// copy the depth info
	short *depth = rawDepthImage->GetData(MEMORYDEVICE_CPU);
	if (data->depthImage.size()) memcpy(depth, data->depthImage.data(), rawDepthImage->dataSize * sizeof(short));
	else memset(depth, 0, rawDepthImage->dataSize * sizeof(short));
}

bool PicoFlexxEngine::hasMoreImages(void) const { return data != NULL; }
Vector2i PicoFlexxEngine::getDepthImageSize(void) const { return data != NULL ? imageSize_d : Vector2i(0, 0); }
Vector2i PicoFlexxEngine::getRGBImageSize(void) const { return data != NULL ? imageSize_rgb : Vector2i(0, 0); }

#else

using namespace InputSource;

PicoFlexxEngine::PicoFlexxEngine(const char *calibFilename, const char *deviceURI, const bool useInternalCalibration, Vector2i requested_imageSize_rgb, Vector2i requested_imageSize_d)
	: BaseImageSourceEngine(calibFilename)
{
	printf("compiled without LibRoyale support\n");
}
PicoFlexxEngine::~PicoFlexxEngine() {}
void PicoFlexxEngine::getImages(ITMUChar4Image *rgbImage, ITMShortImage *rawDepthImage) { }
bool PicoFlexxEngine::hasMoreImages(void) const { return false; }
Vector2i PicoFlexxEngine::getDepthImageSize(void) const { return Vector2i(0, 0); }
Vector2i PicoFlexxEngine::getRGBImageSize(void) const { return Vector2i(0, 0); }

#endif