// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM

#include "LibUVCEngine.h"

using namespace InfiniTAM::Engine;

#ifdef COMPILE_WITH_LibUVC

#include <libuvc/libuvc.h>

class LibUVCEngine::PrivateData {
	public:
	PrivateData(void)
	{
		ctx = NULL;
		dev = NULL; devh_d = NULL; devh_rgb = NULL;
		//ctrl_rgb = NULL; ctrl_d = NULL;
		framebuffer_rgb = NULL; framebuffer_depth = NULL;
		got_depth = got_color = 0;
	}

	uvc_context_t *ctx;
	uvc_device_t *dev;
	uvc_device_handle_t *devh_d;
	uvc_device_handle_t *devh_rgb;
	uvc_stream_ctrl_t ctrl_rgb;
	uvc_stream_ctrl_t ctrl_d;

	uvc_frame_t *framebuffer_rgb;
	uvc_frame_t *framebuffer_depth;

	int got_depth, got_color;
};

/*#include <stdio.h>
#include <stdint.h>*/

void callback_rgb(uvc_frame_t *frame, void *_data)
{
	LibUVCEngine::PrivateData *data = (LibUVCEngine::PrivateData*)_data;

	if (data->framebuffer_rgb == NULL) {
		data->framebuffer_rgb = uvc_allocate_frame(frame->width * frame->height * 3);
	}

	uvc_any2rgb(frame, data->framebuffer_rgb);
	data->got_color = 1;
}

void callback_depth(uvc_frame_t *frame, void *_data)
{
	LibUVCEngine::PrivateData *data = (LibUVCEngine::PrivateData*)_data;

	if (data->framebuffer_depth == NULL) {
		data->framebuffer_depth = uvc_allocate_frame(frame->width * frame->height * 2);
	}

	uvc_duplicate_frame(frame, data->framebuffer_depth);
	data->framebuffer_depth->frame_format = UVC_FRAME_FORMAT_GRAY16;
	data->got_depth = 1;
}

struct FormatSpec {
	int size_x, size_y;
	int fps;
};

struct FormatSpec findSuitableFormat(uvc_device_handle_t *dev, int requiredResolutionX = -1, int requiredResolutionY = -1)
{
	FormatSpec ret;
	ret.size_x = ret.size_y = ret.fps = 0;

	const uvc_format_desc_t *formats = uvc_get_format_descs(dev);

	//fprintf(stderr, "%c%c%c%c\n", formats->guidFormat[0], formats->guidFormat[1], formats->guidFormat[2], formats->guidFormat[3]);
	for (struct uvc_frame_desc *frame_desc = formats->frame_descs; frame_desc != NULL; frame_desc = frame_desc->next) {
		//fprintf(stderr, "%i %i\n", frame_desc->wWidth, frame_desc->wHeight);
		int maxframerate = 0;
		for (int j = 0; frame_desc->intervals[j] != 0; ++j) {
			int fr = (int)(10000000.0f*(1.0f/(float)frame_desc->intervals[j]));
			if (fr > maxframerate) maxframerate = fr;
		}
		bool acceptAsBest = false;
		if ((frame_desc->wWidth == ret.size_x) &&
		    (maxframerate > ret.fps)) {
			acceptAsBest = true;
		} else if ((requiredResolutionX <= 0)&&(requiredResolutionY <= 0)) {
			if (frame_desc->wWidth > ret.size_x) {
				acceptAsBest = true;
			}
		} else {
			int diffX_cur = abs(frame_desc->wWidth - requiredResolutionX);
			int diffX_best = abs(ret.size_x - requiredResolutionX);
			int diffY_cur = abs(frame_desc->wHeight - requiredResolutionY);
			int diffY_best = abs(ret.size_y - requiredResolutionY);
			if (requiredResolutionX > 0) {
				if (diffX_cur < diffX_best) {
					acceptAsBest = true;
				}
				if ((requiredResolutionY > 0)&&(diffX_cur == diffX_best)&&(diffY_cur < diffY_best)) {
					acceptAsBest = true;
				}
			} else if (requiredResolutionY > 0) {
				if (diffY_cur < diffY_best) {
					acceptAsBest = true;
				}
			}
		}
		if (acceptAsBest) {
			ret.size_x = frame_desc->wWidth;
			ret.size_y = frame_desc->wHeight;
			ret.fps = maxframerate;
		}
	}

	//fprintf(stderr, "bestMode: %i %i %i\n", ret.size_x, ret.size_y, ret.fps);
	return ret;
}


LibUVCEngine::LibUVCEngine(const char *calibFilename, Vector2i requested_imageSize_rgb, Vector2i requested_imageSize_d)
	: ImageSourceEngine(calibFilename)
{
	// images from libuvc always seem come in 1/10th-millimeters...
	this->calib.disparityCalib.type = ITMDisparityCalib::TRAFO_AFFINE;
	this->calib.disparityCalib.params = Vector2f(1.0f/10000.0f, 0.0f);

	this->imageSize_rgb = Vector2i(0,0);
	this->imageSize_d = Vector2i(0,0);

	data = new PrivateData();

	do {
		uvc_error_t res;
		res = uvc_init(&(data->ctx), NULL);
		if (res < 0) {
			uvc_perror(res, "LibUVCEngine");
			break;
		}

		res = uvc_find_device(data->ctx, &(data->dev),
			0x8086, 0x0a66, NULL); /* filter devices: vendor_id, product_id, "serial_num" */
			//0x041e, 0x4099, NULL); /* filter devices: vendor_id, product_id, "serial_num" */
		if (res < 0) {
			/* no devices found */
			uvc_perror(res, "uvc_find_device");
			break;
		}

		/* Try to open the device: requires exclusive access */
		res = uvc_open2(data->dev, &(data->devh_rgb), 0); //Try to open camera 0 (RGB)
		if (res < 0) {
			uvc_perror(res, "uvc_open rgb");
			break;
		}

		/* Print out a message containing all the information that libuvc
		 * knows about the device */
		//uvc_print_diag(data->devh_rgb, stderr);

		res = uvc_open2(data->dev, &(data->devh_d), 1); //Try to open camera 1  (depth)
		if (res < 0) {
			uvc_perror(res, "uvc_open depth");
			break;
		}
		//uvc_print_diag(data->devh_d, stderr);
		FormatSpec format_rgb = findSuitableFormat(data->devh_rgb, requested_imageSize_rgb.x, requested_imageSize_rgb.y);
		FormatSpec format_d = findSuitableFormat(data->devh_d, requested_imageSize_rgb.x, requested_imageSize_rgb.y);

		/* Try to negotiate a 640x480 30 fps YUYV stream profile */
		res = uvc_get_stream_ctrl_format_size(
			data->devh_rgb, &(data->ctrl_rgb),
			UVC_FRAME_FORMAT_YUYV, /* YUV 422, aka YUV 4:2:2. try _COMPRESSED */
			format_rgb.size_x, format_rgb.size_y, format_rgb.fps);

		/* Print out the result */
		//uvc_print_stream_ctrl(&(data->ctrl_rgb), stderr);

		if (res < 0) {
			uvc_perror(res, "get_mode rgb");
			break;
		}
		/* Try to negotiate a 640x480 30 fps YUYV stream profile */
		res = uvc_get_stream_ctrl_format_size(
			data->devh_d, &(data->ctrl_d),
			UVC_FRAME_FORMAT_YUYV/*INVI*/,
			format_d.size_x, format_d.size_y, format_d.fps);

		/* Print out the result */
//		uvc_print_stream_ctrl(&(data->ctrl_d), stderr);
//		uvc_print_diag(data->devh_d, NULL);    

		if (res < 0) {
			uvc_perror(res, "get_mode depth");
			break;
		}

		res = uvc_start_streaming(data->devh_rgb, &(data->ctrl_rgb), callback_rgb, data, 0);
		if (res < 0) {
			uvc_perror(res, "start_streaming rgb");
		}
		res = uvc_start_streaming(data->devh_d, &(data->ctrl_d), callback_depth, data, 0);
		if (res < 0) {
			uvc_perror(res, "start_streaming depth");
			break;
		}

		this->imageSize_rgb = Vector2i(format_rgb.size_x, format_rgb.size_y);
		this->imageSize_d = Vector2i(format_d.size_x, format_d.size_y);
	} while (false);
}

LibUVCEngine::~LibUVCEngine(void)
{
	/* End the stream. Blocks until last callback is serviced */
	if (data->devh_rgb != NULL) {
		uvc_stop_streaming(data->devh_rgb);
	}
	if (data->devh_d != NULL) {
		uvc_stop_streaming(data->devh_d);
	}
	if (data->devh_rgb != NULL) {
		uvc_close(data->devh_rgb);
	}
	if (data->devh_d != NULL) {
		uvc_close(data->devh_d);
	}

	/* Release the device descriptor */
	if (data->dev != NULL) uvc_unref_device(data->dev);
	if (data->ctx != NULL) uvc_exit(data->ctx);
}

void LibUVCEngine::getImages(ITMUChar4Image *rgbImage, ITMShortImage *rawDepthImage)
{
	int counter = 0;
	while ((data->got_color == 0) || (data->got_depth == 0)) {
		struct timespec sleeptime;
		sleeptime.tv_sec = 0;
		sleeptime.tv_nsec = 10000000;
		nanosleep(&sleeptime, NULL);
		if (counter++ > 100) {
			fprintf(stderr, "timeout waiting for images\n");
			return;
		}
	}

	Vector4u *rgb = rgbImage->GetData(MEMORYDEVICE_CPU);
	for (int i = 0; i < rgbImage->noDims.x * rgbImage->noDims.y; i++) {
		Vector4u newPix; char *oldPix = &(((char*)(data->framebuffer_rgb->data))[i*3]);
		newPix.x = oldPix[0]; newPix.y = oldPix[1]; newPix.z = oldPix[2]; newPix.w = 255;
		rgb[i] = newPix;
	}

	short *depth = rawDepthImage->GetData(MEMORYDEVICE_CPU);
	memcpy(depth, data->framebuffer_depth->data, rawDepthImage->dataSize * sizeof(short));

	data->got_color = data->got_depth = 0;
}

bool LibUVCEngine::hasMoreImages(void) { return true; }
Vector2i LibUVCEngine::getDepthImageSize(void) { return imageSize_d; }
Vector2i LibUVCEngine::getRGBImageSize(void) { return imageSize_rgb; }

#else

LibUVCEngine::LibUVCEngine(const char *calibFilename, Vector2i requested_imageSize_rgb, Vector2i requested_imageSize_d)
	: ImageSourceEngine(calibFilename)
{
	printf("compiled without LibUVC support\n");
}
LibUVCEngine::~LibUVCEngine()
{}
void LibUVCEngine::getImages(ITMUChar4Image *rgbImage, ITMShortImage *rawDepthImage)
{ return; }
bool LibUVCEngine::hasMoreImages(void)
{ return false; }
Vector2i LibUVCEngine::getDepthImageSize(void)
{ return Vector2i(0,0); }
Vector2i LibUVCEngine::getRGBImageSize(void)
{ return Vector2i(0,0); }

#endif

