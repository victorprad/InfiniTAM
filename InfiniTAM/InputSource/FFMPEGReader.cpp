// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

// this hack is required on android
#define __STDC_CONSTANT_MACROS
#define __STDC_LIMIT_MACROS
#include <stdint.h>

#include "FFMPEGReader.h"

// If we're using a version of Visual Studio prior to 2015, snprintf isn't supported, so fall back to the non-standard _snprintf instead.
#if defined(_MSC_VER) && _MSC_VER < 1900
	#define snprintf _snprintf
#endif

#ifdef COMPILE_WITH_FFMPEG

extern "C" {
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
#include <libavfilter/avfiltergraph.h>
#include <libavfilter/buffersink.h>
#include <libavfilter/buffersrc.h>
#include <libavutil/opt.h>
#include <libavutil/pixdesc.h>
}

#include <deque>
#include <iostream>

using namespace InputSource;

class FFMPEGReader::PrivateData {
	private:
	typedef struct FilteringContext {
		AVFilterContext *buffersink_ctx;
		AVFilterContext *buffersrc_ctx;
		AVFilterGraph *filter_graph;
	} FilteringContext;

	public:
	PrivateData(void)
	{
		depthStreamIdx = colorStreamIdx = -1;
		av_init_packet(&packet);
		packet.data = NULL;
		packet.size = 0;
		frame = NULL;
		ifmt_ctx = NULL;
		filter_ctx = NULL;
	}

	~PrivateData(void)
	{
		av_packet_unref(&packet);
		av_frame_free(&frame);
	}

	bool open(const char *filename);
	bool readFrames(void);
	bool close(void);

	Vector2i getDepthImageSize(void) const
	{
		if (!providesDepth()) return Vector2i(0,0);
		AVCodecContext *dec_ctx = ifmt_ctx->streams[depthStreamIdx]->codec;
		return Vector2i(dec_ctx->width, dec_ctx->height);
		//return Vector2i(dec_ctx->width + 128, dec_ctx->height + 96);
	}

	Vector2i getColorImageSize(void) const
	{
		if (!providesColor()) return Vector2i(0,0);
		AVCodecContext *dec_ctx = ifmt_ctx->streams[colorStreamIdx]->codec;
		return Vector2i(dec_ctx->width, dec_ctx->height);
	}

	bool providesDepth(void) const
	{ return (depthStreamIdx >= 0); }
	bool hasQueuedDepth(void) const
	{ return (depthFrames.size()>0); }
	AVFrame* getFromDepthQueue(void)
	{
		if (depthFrames.size() == 0) return NULL;
		AVFrame *ret = depthFrames.front();
		depthFrames.pop_front();
		return ret;
	}

	bool providesColor(void) const
	{ return (colorStreamIdx >= 0); }
	bool hasQueuedColor(void) const
	{ return (colorFrames.size()>0); }
	AVFrame* getFromColorQueue(void)
	{
		if (colorFrames.size() == 0) return NULL;
		AVFrame *ret = colorFrames.front();
		colorFrames.pop_front();
		return ret;
	}

	bool hasMoreImages(void)
	{
		//fprintf(stderr, "check: %i %i %i %i\n", providesColor(), hasQueuedColor(), providesDepth(), hasQueuedDepth());
		if (providesColor()) {
			if (!hasQueuedColor()) readFrames();
			if (!hasQueuedColor()) return false;
		}
		if (providesDepth()) {
			if (!hasQueuedDepth()) readFrames();
			if (!hasQueuedDepth()) return false;
		}
		return true;
	}
	void flushQueue(bool depth)
	{
		while (true) {
			AVFrame *tmp;
			if (depth) tmp = getFromDepthQueue();
			else tmp = getFromColorQueue();
			if (tmp == NULL) break;
			av_frame_free(&tmp);
		}
	}

	private:
	int open_input_file(const char *filename);
	static int init_filter(FilteringContext* fctx, AVCodecContext *dec_ctx, const char *filter_spec, bool isDepth);
	int init_filters(void);
	int filter_decode_frame(AVFrame *frame, int stream_index);
	void flush_decoder_and_filter(void);

	AVFormatContext *ifmt_ctx;
	FilteringContext *filter_ctx;

	AVPacket packet;
	AVFrame *frame;

	int depthStreamIdx;
	int colorStreamIdx;
	std::deque<AVFrame*> depthFrames;
	std::deque<AVFrame*> colorFrames;
};

int FFMPEGReader::PrivateData::open_input_file(const char *filename)
{
	int ret;
	unsigned int i;
	if ((ret = avformat_open_input(&ifmt_ctx, filename, NULL, NULL)) < 0) {
		std::cerr << "Cannot open input file" << std::endl;
		return ret;
	}
	if ((ret = avformat_find_stream_info(ifmt_ctx, NULL)) < 0) {
		std::cerr << "Cannot find stream information" << std::endl;
		return ret;
	}
	for (i = 0; i < ifmt_ctx->nb_streams; i++) {
		AVStream *stream;
		AVCodecContext *codec_ctx;
		stream = ifmt_ctx->streams[i];
		codec_ctx = stream->codec;
		/* Reencode video & audio and remux subtitles etc. */
		if (codec_ctx->codec_type == AVMEDIA_TYPE_VIDEO) {
			/* Open decoder */
			ret = avcodec_open2(codec_ctx, avcodec_find_decoder(codec_ctx->codec_id), NULL);
			if (ret < 0) {
				std::cerr << "Failed to open decoder for stream #" << i << std::endl;
				continue;
				//return ret;
			}
			if (codec_ctx->pix_fmt == AV_PIX_FMT_GRAY16LE) depthStreamIdx = i;
			if ((codec_ctx->pix_fmt == AV_PIX_FMT_YUV422P)||
			    (codec_ctx->pix_fmt == AV_PIX_FMT_RGB24)||
			    (codec_ctx->pix_fmt == AV_PIX_FMT_RGBA)) colorStreamIdx = i;
		}
	}
	//av_dump_format(ifmt_ctx, 0, filename, 0);
	return 0;
}

int FFMPEGReader::PrivateData::init_filter(FilteringContext* fctx, AVCodecContext *dec_ctx, const char *filter_spec, bool isDepth)
{
	char args[512];
	int ret = 0;
	AVFilter *buffersrc = NULL;
	AVFilter *buffersink = NULL;
	AVFilterContext *buffersrc_ctx = NULL;
	AVFilterContext *buffersink_ctx = NULL;
	AVFilterInOut *outputs = avfilter_inout_alloc();
	AVFilterInOut *inputs  = avfilter_inout_alloc();
	AVFilterGraph *filter_graph = avfilter_graph_alloc();

	// TODO: depending on endianness, requiredOutput should maybe be set to
	//       AV_PIX_FMT_GRAY16BE
	AVPixelFormat requiredOutput = isDepth?AV_PIX_FMT_GRAY16LE:AV_PIX_FMT_RGBA;

	if (!outputs || !inputs || !filter_graph) {
		ret = AVERROR(ENOMEM);
		goto end;
	}
	buffersrc = avfilter_get_by_name("buffer");
	buffersink = avfilter_get_by_name("buffersink");
	if (!buffersrc || !buffersink) {
		std::cerr << "filtering source or sink element not found" << std::endl;
		ret = AVERROR_UNKNOWN;
		goto end;
	}
	sprintf(args,
	         "video_size=%dx%d:pix_fmt=%d:time_base=%d/%d:pixel_aspect=%d/%d",
	         dec_ctx->width, dec_ctx->height, dec_ctx->pix_fmt,
	         dec_ctx->time_base.num, dec_ctx->time_base.den,
	         dec_ctx->sample_aspect_ratio.num,
	         dec_ctx->sample_aspect_ratio.den);
	ret = avfilter_graph_create_filter(&buffersrc_ctx, buffersrc, "in", args, NULL, filter_graph);
	if (ret < 0) {
		std::cerr << "Cannot create buffer source" << std::endl;
		goto end;
	}
	ret = avfilter_graph_create_filter(&buffersink_ctx, buffersink, "out", NULL, NULL, filter_graph);
	if (ret < 0) {
		std::cerr << "Cannot create buffer sink" << std::endl;
		goto end;
	}

	ret = av_opt_set_bin(buffersink_ctx, "pix_fmts", (uint8_t*)&requiredOutput, sizeof(requiredOutput), AV_OPT_SEARCH_CHILDREN);
	if (ret < 0) {
		std::cerr << "Cannot set output pixel format" << std::endl;
		goto end;
	}
	/* Endpoints for the filter graph. */
	outputs->name       = av_strdup("in");
	outputs->filter_ctx = buffersrc_ctx;
	outputs->pad_idx    = 0;
	outputs->next       = NULL;
	inputs->name       = av_strdup("out");
	inputs->filter_ctx = buffersink_ctx;
	inputs->pad_idx    = 0;
	inputs->next       = NULL;
	if (!outputs->name || !inputs->name) {
		ret = AVERROR(ENOMEM);
		goto end;
	}
	if ((ret = avfilter_graph_parse_ptr(filter_graph, filter_spec, &inputs, &outputs, NULL)) < 0) goto end;
	if ((ret = avfilter_graph_config(filter_graph, NULL)) < 0) goto end;

	/* Fill FilteringContext */
	fctx->buffersrc_ctx = buffersrc_ctx;
	fctx->buffersink_ctx = buffersink_ctx;
	fctx->filter_graph = filter_graph;
end:
	avfilter_inout_free(&inputs);
	avfilter_inout_free(&outputs);
	return ret;
}

int FFMPEGReader::PrivateData::init_filters(void)
{
	const char *filter_spec;
	int i;
	int ret;
	filter_ctx = (FilteringContext*)av_malloc_array(ifmt_ctx->nb_streams, sizeof(*filter_ctx));
	if (!filter_ctx) return AVERROR(ENOMEM);
	for (i = 0; (unsigned int)i < ifmt_ctx->nb_streams; i++) {
		filter_ctx[i].buffersrc_ctx  = NULL;
		filter_ctx[i].buffersink_ctx = NULL;
		filter_ctx[i].filter_graph   = NULL;
		if ((i != depthStreamIdx)&&(i != colorStreamIdx)) continue;
		filter_spec = "null"; /* passthrough (dummy) filter for video */
		ret = init_filter(&filter_ctx[i], ifmt_ctx->streams[i]->codec, filter_spec, (i == depthStreamIdx));
		if (ret) return ret;
	}
	return 0;
}

int FFMPEGReader::PrivateData::filter_decode_frame(AVFrame *frame, int stream_index)
{
	int ret;

	/* push the decoded frame into the filtergraph */
	ret = av_buffersrc_add_frame_flags(filter_ctx[stream_index].buffersrc_ctx, frame, 0);
	if (ret < 0) {
		std::cerr << "Error while feeding the filtergraph" << std::endl;
		return ret;
	}

	/* pull filtered frames from the filtergraph */
	while (1) {
		AVFrame *filt_frame = av_frame_alloc();
		if (!filt_frame) {
			ret = AVERROR(ENOMEM);
			break;
		}
		ret = av_buffersink_get_frame(filter_ctx[stream_index].buffersink_ctx, filt_frame);
		if (ret < 0) {
		/* if no more frames for output - returns AVERROR(EAGAIN)
		 * if flushed and no more frames for output - returns AVERROR_EOF
		 * rewrite retcode to 0 to show it as normal procedure completion
		 */
			if (ret == AVERROR(EAGAIN) || ret == AVERROR_EOF) ret = 0;
			av_frame_free(&filt_frame);
			break;
		}
		filt_frame->pict_type = AV_PICTURE_TYPE_NONE;

		if (stream_index == depthStreamIdx) depthFrames.push_back(filt_frame);
		else if (stream_index == colorStreamIdx) colorFrames.push_back(filt_frame);
		else av_frame_free(&filt_frame);
	}
	return ret;
}

bool FFMPEGReader::PrivateData::open(const char *filename)
{
	av_register_all();
	avfilter_register_all();
	if (open_input_file(filename) < 0) return false;
	if (init_filters() < 0) return false;
	return true;
}

void FFMPEGReader::PrivateData::flush_decoder_and_filter(void)
{
	AVPacket flushPacket;
	av_init_packet(&flushPacket);
	flushPacket.data = NULL;
	flushPacket.size = 0;
	int ret = 0;
	int got_frame;

	/* flush filters and decoders */
	for (int i = 0; (unsigned int)i < ifmt_ctx->nb_streams; i++) {
		if ((i != colorStreamIdx)&&(i != depthStreamIdx)) continue;
		if (filter_ctx[i].filter_graph == NULL) continue;

		/* flush decoder */
		while (true) {
			AVFrame *frame = av_frame_alloc();
			if (!frame) {
				ret = AVERROR(ENOMEM);
				goto end;
			}
			ret = avcodec_decode_video2(ifmt_ctx->streams[i]->codec, frame, &got_frame, &flushPacket);
			if ((ret < 0)||(!got_frame)) {
				av_frame_free(&frame);
				if (ret < 0) std::cerr << "Decoding failed" << std::endl;
				break;
			}
			frame->pts = av_frame_get_best_effort_timestamp(frame);
			ret = filter_decode_frame(frame, i);
			av_frame_free(&frame);
			if (ret < 0) goto end;
		}

		/* flush filter */
		ret = filter_decode_frame(NULL, i);
		if (ret < 0) {
			std::cerr << "Flushing filter failed" << std::endl;
			goto end;
		}
	}
end:
	return;
}

bool FFMPEGReader::PrivateData::readFrames(void)
{
	int ret = 0;
	int stream_index;
	int got_frame;

	while (1) {
		// do we have to read more images?
		bool waitForColor = providesColor()&&(!hasQueuedColor());
		bool waitForDepth = providesDepth()&&(!hasQueuedDepth());
		if ((!waitForColor)&&(!waitForDepth)) break;

		// read packets
		if ((ret = av_read_frame(ifmt_ctx, &packet)) < 0) {
			flush_decoder_and_filter();
			break;
		}
		stream_index = packet.stream_index;
		if ((stream_index != colorStreamIdx)&&(stream_index != depthStreamIdx)) continue;
		if (filter_ctx[stream_index].filter_graph) {
			frame = av_frame_alloc();
			if (!frame) {
				ret = AVERROR(ENOMEM);
				break;
			}
			av_packet_rescale_ts(&packet,
			     ifmt_ctx->streams[stream_index]->time_base,
			     ifmt_ctx->streams[stream_index]->codec->time_base);
			ret = avcodec_decode_video2(ifmt_ctx->streams[stream_index]->codec, frame, &got_frame, &packet);
			if (ret < 0) {
				av_frame_free(&frame);
				std::cerr << "Decoding failed" << std::endl;
				break;
			}
			if (got_frame) {
				frame->pts = av_frame_get_best_effort_timestamp(frame);
				ret = filter_decode_frame(frame, stream_index);
				av_frame_free(&frame);
				//if (ret < 0) goto end;
				break;
			} else {
				av_frame_free(&frame);
			}
		}
		av_packet_unref(&packet);
	}

	return (ret == 0);
}

bool FFMPEGReader::PrivateData::close(void)
{
	if (ifmt_ctx != NULL) {
		for (unsigned int i = 0; i < ifmt_ctx->nb_streams; i++) {
			avcodec_close(ifmt_ctx->streams[i]->codec);
			if (filter_ctx && filter_ctx[i].filter_graph) avfilter_graph_free(&filter_ctx[i].filter_graph);
		}
		avformat_close_input(&ifmt_ctx);
	}
	if (filter_ctx != NULL) av_free(filter_ctx);
	ifmt_ctx = NULL;
	filter_ctx = NULL;
	return true;
}

FFMPEGReader::FFMPEGReader(const char *calibFilename, const char *filename1, const char *filename2)
	: BaseImageSourceEngine(calibFilename)
{
	mData1 = new PrivateData();
	isValid = mData1->open(filename1);

	if (isValid && (filename2 != NULL)) {
		mData2 = new PrivateData();
		mData2->open(filename2);
	} else {
		mData2 = NULL;
	}
}

FFMPEGReader::~FFMPEGReader(void)
{
	if (isValid) mData1->close();
	delete mData1;
	if (mData2 != NULL) {
		if (isValid) mData2->close();
		delete mData2;
	}
}

bool FFMPEGReader::hasMoreImages(void) const
{
	if (!isValid) return false;
	if (!mData1->hasMoreImages()) return false;
	if (mData2 != NULL) if (!mData2->hasMoreImages()) return false;
	return true;
}

static void copyRgba(const AVFrame *frame, Vector4u *rgb)
{
	for (int y = 0; y < frame->height; ++y) for (int x = 0; x < frame->width; ++x) {
		Vector4u tmp;
		tmp.x = frame->data[0][x*4+frame->linesize[0]*y + 0];
		tmp.y = frame->data[0][x*4+frame->linesize[0]*y + 1];
		tmp.z = frame->data[0][x*4+frame->linesize[0]*y + 2];
		tmp.w = frame->data[0][x*4+frame->linesize[0]*y + 3];
		rgb[x+y*frame->width] = tmp;
	}
}

static void copyDepth(const AVFrame *frame, short *depth)
{
	memcpy(depth, frame->data[0], frame->height*frame->width*2);
}

void FFMPEGReader::getImages(ITMUChar4Image *rgbImage, ITMShortImage *depthImage)
{
	Vector4u *rgb = rgbImage->GetData(MEMORYDEVICE_CPU);
	short *depth = depthImage->GetData(MEMORYDEVICE_CPU);

	bool gotColour = false;
	bool gotDepth = false;
	if (isValid) {
		if (mData1->providesColor()) {
			if (mData2 != NULL) mData2->flushQueue(false);

			if (!mData1->hasQueuedColor()) mData1->readFrames();
			if (mData1->hasQueuedColor()) {
				AVFrame *frame = mData1->getFromColorQueue();
				copyRgba(frame, rgb);
				av_frame_free(&frame);

				gotColour = true;
			}
		} else if (mData2 != NULL) if (mData2->providesColor()) {
			if (!mData2->hasQueuedColor()) mData2->readFrames();
			if (mData2->hasQueuedColor()) {
				AVFrame *frame = mData2->getFromColorQueue();
				copyRgba(frame, rgb);
				av_frame_free(&frame);

				gotColour = true;
			}
		}

		if (mData1->providesDepth()) {
			if (mData2 != NULL) mData2->flushQueue(true);

			if (!mData1->hasQueuedDepth()) mData1->readFrames();
			if (mData1->hasQueuedDepth()) {
				AVFrame *frame = mData1->getFromDepthQueue();
				copyDepth(frame, depth);
				av_frame_free(&frame);

				gotDepth = true;
			}
		} else if (mData2 != NULL) if (mData2->providesDepth()) {
			if (!mData2->hasQueuedDepth()) mData2->readFrames();
			if (mData2->hasQueuedDepth()) {
				AVFrame *frame = mData2->getFromDepthQueue();
				copyDepth(frame, depth);
				av_frame_free(&frame);

				gotDepth = true;
			}
		}
	}
	if (!gotColour) memset(rgb, 0, rgbImage->dataSize * sizeof(Vector4u));
	if (!gotDepth) memset(depth, 0, depthImage->dataSize * sizeof(short));
}

Vector2i FFMPEGReader::getDepthImageSize(void) const
{
	if (mData1->providesDepth()) return mData1->getDepthImageSize();
	if (mData2 != NULL) if (mData2->providesDepth()) return mData2->getDepthImageSize();
	return Vector2i(0,0);
}

Vector2i FFMPEGReader::getRGBImageSize(void) const
{
	if (mData1->providesColor()) return mData1->getColorImageSize();
	if (mData2 != NULL) if (mData2->providesColor()) return mData2->getColorImageSize();
	return Vector2i(0,0);
}

#else

using namespace InputSource;

FFMPEGReader::FFMPEGReader(const char *calibFilename, const char *filename1, const char *filename2)
	: BaseImageSourceEngine(calibFilename)
{
	printf("compiled without FFMPEG\n");
}
FFMPEGReader::~FFMPEGReader(void)
{}
void FFMPEGReader::getImages(ITMUChar4Image *rgbImage, ITMShortImage *rawDepthImage)
{ return; }
bool FFMPEGReader::hasMoreImages(void) const
{ return false; }
Vector2i FFMPEGReader::getDepthImageSize(void) const
{ return Vector2i(0,0); }
Vector2i FFMPEGReader::getRGBImageSize(void) const
{ return Vector2i(0,0); }

#endif

