// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

// this hack is required on android
#define __STDC_CONSTANT_MACROS
#define __STDC_LIMIT_MACROS
#include <stdint.h>

#include "FFMPEGWriter.h"

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
#include <libavutil/imgutils.h>
}

#include <iostream>

using namespace InputSource;

class FFMPEGWriter::PrivateData {
	public:
	int open(const char *filename, int size_x, int size_y, bool isDepth, int fps);
	int init_filters(void);
	int encode_write_frame(AVFrame *filt_frame, unsigned int stream_index, int *got_frame);
	int filter_encode_write_frame(AVFrame *frame, unsigned int stream_index);
	int flush_encoder(unsigned int stream_index);
	int close(void);

	AVFrame* getFrame(void) { return frame; }

	private:
	typedef struct FilteringContext {
		AVFilterContext *buffersink_ctx;
		AVFilterContext *buffersrc_ctx;
		AVFilterGraph *filter_graph;
	} FilteringContext;

	void allocFrame(bool isDepth);
	void freeFrame(void);
	static int init_filter(FilteringContext* fctx, AVCodecContext *enc_ctx, const char *filter_spec);


	AVFormatContext *ofmt_ctx;
	AVFrame *frame;

	FilteringContext filter_ctx;
};

int FFMPEGWriter::PrivateData::open(const char *filename, int size_x, int size_y, bool isDepth, int fps)
{
	printf("saving to video file: %s\n", filename);

	AVStream *out_stream;
	AVCodecContext *enc_ctx;
	AVCodec *encoder;
	int ret;

	ofmt_ctx = NULL;
	avformat_alloc_output_context2(&ofmt_ctx, NULL, NULL, filename);
	if (!ofmt_ctx) {
		std::cerr << "Could not create ffmpeg output context" << std::endl;
		return -1;
	}

	out_stream = avformat_new_stream(ofmt_ctx, NULL);
	out_stream->time_base.num = 1;
	out_stream->time_base.den = fps;
	if (!out_stream) {
		std::cerr << "Failed allocating ffmpeg output stream" << std::endl;
		return -1;
	}
	enc_ctx = out_stream->codec;
	encoder = avcodec_find_encoder(AV_CODEC_ID_FFV1);
	if (!encoder) {
		std::cerr << "Necessary encoder not found in ffmpeg" << std::endl;
		return -1;
	}
	enc_ctx->width = size_x;
	enc_ctx->height = size_y;
	enc_ctx->sample_aspect_ratio.num = 1;
	enc_ctx->sample_aspect_ratio.den = 1;
	enc_ctx->pix_fmt = isDepth?AV_PIX_FMT_GRAY16LE:AV_PIX_FMT_YUV422P;
	enc_ctx->time_base.num = 1;
	enc_ctx->time_base.den = fps;

	/* Third parameter can be used to pass settings to encoder */
	AVDictionary *dict = NULL;
	av_dict_set(&dict, "coder", "1", 0);
	av_dict_set(&dict, "context", "0", 0);
	av_dict_set(&dict, "level", "3", 0);
	av_dict_set(&dict, "threads", "8", 0);
	av_dict_set(&dict, "slices", "16", 0);
	ret = avcodec_open2(enc_ctx, encoder, &dict);
	av_dict_free(&dict);
	if (ret < 0) {
		std::cerr << "Cannot open video encoder for stream" << std::endl;
		return ret;
	}
	if (ofmt_ctx->oformat->flags & AVFMT_GLOBALHEADER) {
		enc_ctx->flags |= /*AV_*/CODEC_FLAG_GLOBAL_HEADER;
	}
//	av_dump_format(ofmt_ctx, 0, filename, 1);
	if (!(ofmt_ctx->oformat->flags & AVFMT_NOFILE)) {
		ret = avio_open(&ofmt_ctx->pb, filename, AVIO_FLAG_WRITE);
		if (ret < 0) {
			std::cerr << "Could not open output file '" << filename << "'" << std::endl;
			return ret;
		}
	}

	ret = avformat_write_header(ofmt_ctx, NULL);
	if (ret < 0) {
		std::cerr << "Error occurred when opening output file" << std::endl;
		return ret;
	}

	allocFrame(isDepth);

	return 0;
}

int FFMPEGWriter::PrivateData::init_filter(FilteringContext* fctx, AVCodecContext *enc_ctx, const char *filter_spec)
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
		// TODO: depending on host endianness, the desired format should
		//       maybe be set to AV_PIX_FMT_GRAY16BE
		enc_ctx->width, enc_ctx->height, (enc_ctx->pix_fmt==AV_PIX_FMT_GRAY16LE)?AV_PIX_FMT_GRAY16LE:AV_PIX_FMT_RGBA /*RGB24*/ /*AV_PIX_FMT_RGBA */,
		enc_ctx->time_base.num, enc_ctx->time_base.den,
		enc_ctx->sample_aspect_ratio.num,
		enc_ctx->sample_aspect_ratio.den);
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
	ret = av_opt_set_bin(buffersink_ctx, "pix_fmts",
		(uint8_t*)&enc_ctx->pix_fmt, sizeof(enc_ctx->pix_fmt),
		AV_OPT_SEARCH_CHILDREN);
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

int FFMPEGWriter::PrivateData::init_filters(void)
{
	const char *filter_spec;
	int ret;
	filter_ctx.buffersrc_ctx  = NULL;
	filter_ctx.buffersink_ctx = NULL;
	filter_ctx.filter_graph   = NULL;
	filter_spec = "null";
        ret = init_filter(&filter_ctx, ofmt_ctx->streams[0]->codec, filter_spec);
	return ret;
}

int FFMPEGWriter::PrivateData::encode_write_frame(AVFrame *filt_frame, unsigned int stream_index, int *got_frame)
{
	int ret;
	int got_frame_local;
	AVPacket enc_pkt;
	if (!got_frame) got_frame = &got_frame_local;

	/* encode filtered frame */
	enc_pkt.data = NULL;
	enc_pkt.size = 0;
	av_init_packet(&enc_pkt);
	ret = avcodec_encode_video2(ofmt_ctx->streams[stream_index]->codec, &enc_pkt, filt_frame, got_frame);
	av_frame_free(&filt_frame);
	if (ret < 0) return ret;
	if (!(*got_frame)) return 0;

	/* prepare packet for muxing */
	enc_pkt.stream_index = stream_index;
	av_packet_rescale_ts(&enc_pkt,
	                     ofmt_ctx->streams[stream_index]->codec->time_base,
	                     ofmt_ctx->streams[stream_index]->time_base);

	/* mux encoded frame */
	ret = av_interleaved_write_frame(ofmt_ctx, &enc_pkt);
	return ret;
}

int FFMPEGWriter::PrivateData::filter_encode_write_frame(AVFrame *frame, unsigned int stream_index)
{
	int ret;
	AVFrame *filt_frame;

	// filter
	ret = av_buffersrc_add_frame_flags(filter_ctx.buffersrc_ctx, frame, 0);
	if (ret < 0) {
		std::cerr << "Error while feeding the filtergraph" << std::endl;
		return ret;
	}

	// write frames
	while (1) {
		filt_frame = av_frame_alloc();
		if (!filt_frame) {
			ret = AVERROR(ENOMEM);
			break;
		}
		ret = av_buffersink_get_frame(filter_ctx.buffersink_ctx, filt_frame);
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
		ret = encode_write_frame(filt_frame, stream_index, NULL);
		if (ret < 0) break;
	}
	return ret;
}

int FFMPEGWriter::PrivateData::flush_encoder(unsigned int stream_index)
{
	int ret;
	int got_frame;
	if (!(ofmt_ctx->streams[stream_index]->codec->codec->capabilities & /*AV_*/CODEC_CAP_DELAY)) return 0;

	while (1) {
		ret = encode_write_frame(NULL, stream_index, &got_frame);
		if (ret < 0) break;
		if (!got_frame) return 0;
	}
	return ret;
}

int FFMPEGWriter::PrivateData::close(void)
{
	int ret = 0;
        /* flush filter */
	do {
	        if (!filter_ctx.filter_graph) continue;
	        ret = filter_encode_write_frame(NULL, 0);
		if (ret < 0) {
			std::cerr << "Flushing filter failed" << std::endl;
			goto end;
		}
		/* flush encoder */
		ret = flush_encoder(0);
		if (ret < 0) {
			std::cerr << "Flushing encoder failed" << std::endl;
			goto end;
		}
	} while (false);
	av_write_trailer(ofmt_ctx);
end:
	avcodec_close(ofmt_ctx->streams[0]->codec);
	if (/*mData->filter_ctx && */filter_ctx.filter_graph) {
		avfilter_graph_free(&filter_ctx.filter_graph);
	}
//    av_free(filter_ctx);
	if (ofmt_ctx && !(ofmt_ctx->oformat->flags & AVFMT_NOFILE))
		avio_closep(&ofmt_ctx->pb);
	avformat_free_context(ofmt_ctx);
	if (ret < 0)
		std::cerr << "Error occurred: " /* << std::string(av_err2str(ret)) */<< std::endl;

	freeFrame();
	return ret;
}

void FFMPEGWriter::PrivateData::allocFrame(bool isDepth)
{
	AVCodecContext *enc_ctx = ofmt_ctx->streams[0]->codec; 
	int ret = 0;

	frame = av_frame_alloc();
	if (!frame) {
		std::cerr << "Could not allocate video frame" << std::endl;
		return;
	}
	frame->format = isDepth?AV_PIX_FMT_GRAY16LE:AV_PIX_FMT_RGBA;
	frame->width  = enc_ctx->width;
	frame->height = enc_ctx->height;

	ret = av_image_alloc(frame->data, frame->linesize, frame->width, frame->height,
		(AVPixelFormat)frame->format, 32);

	if (ret < 0) {
		fprintf(stderr, "Could not allocate raw picture buffer\n");
		return;
	}
}

void FFMPEGWriter::PrivateData::freeFrame(void)
{
	av_freep(&frame->data[0]);
	av_frame_free(&frame);
}


FFMPEGWriter::FFMPEGWriter(void)
{
	mData = new PrivateData();
	counter = -1;
}

FFMPEGWriter::~FFMPEGWriter(void)
{
	close();

	delete mData;
}

bool FFMPEGWriter::open(const char *filename, int size_x, int size_y, bool isDepth, int fps)
{
	if (isOpen()) return false;

	int ret;
	av_register_all();
	avfilter_register_all();
	if ((ret = mData->open(filename, size_x, size_y, isDepth, fps)) < 0) return false;
	if ((ret = mData->init_filters()) < 0) return false;

	counter = 0;
	return true;
}


bool FFMPEGWriter::writeFrame(ITMUChar4Image *rgbImage)
{
	if (!isOpen()) return false;

	AVFrame *frame = mData->getFrame();

	if ((frame->format != AV_PIX_FMT_RGBA)||(frame->width != rgbImage->noDims.x)||(frame->height != rgbImage->noDims.y)) {
		std::cerr << "FFMPEGWriter: wrong image format for rgb stream" << std::endl;
	}

	Vector4u *rgb = rgbImage->GetData(MEMORYDEVICE_CPU);
	for (int y = 0; y < frame->height; ++y) for (int x = 0; x < frame->width; ++x) {
		frame->data[0][x*4+y*frame->linesize[0] + 0] = rgb[x+y*rgbImage->noDims.x].x;
		frame->data[0][x*4+y*frame->linesize[0] + 1] = rgb[x+y*rgbImage->noDims.x].y;
		frame->data[0][x*4+y*frame->linesize[0] + 2] = rgb[x+y*rgbImage->noDims.x].z;
		frame->data[0][x*4+y*frame->linesize[0] + 3] = rgb[x+y*rgbImage->noDims.x].w;
	}

	frame->pts = counter++;
	int ret = mData->filter_encode_write_frame(frame, /*stream_index*/0);
	return (ret>=0);
}

bool FFMPEGWriter::writeFrame(ITMShortImage *depthImage)
{
	if (!isOpen()) return false;

	AVFrame *frame = mData->getFrame();

	if ((frame->format != AV_PIX_FMT_GRAY16LE)||(frame->width != depthImage->noDims.x)||(frame->height != depthImage->noDims.y)) {
		std::cerr << "FFMPEGWriter: wrong image format for depth stream" << std::endl;
	}

	short *depth = depthImage->GetData(MEMORYDEVICE_CPU);
	unsigned char *tmp = frame->data[0];
	frame->data[0] = (unsigned char*)depth;

	frame->pts = counter++;
	int ret = mData->filter_encode_write_frame(frame, /*stream_index*/0);
	frame->data[0] = tmp;
	return (ret>=0);
}

bool FFMPEGWriter::close(void)
{
	if (!isOpen()) return false;

	counter = -1;
	int ret = mData->close();
	return (ret>=0);
}

bool FFMPEGWriter::isOpen(void) const
{
	return (counter>=0);
}

#else

using namespace InputSource;

FFMPEGWriter::FFMPEGWriter(void)
{}
FFMPEGWriter::~FFMPEGWriter(void)
{}
bool FFMPEGWriter::open(const char *filename, int size_x, int size_y, bool isDepth, int fps)
{ printf("compiled without FFMPEG\n"); return false; }
bool FFMPEGWriter::writeFrame(ITMUChar4Image *rgbImage)
{ return false; }
bool FFMPEGWriter::writeFrame(ITMShortImage *depthImage)
{ return false; }
bool FFMPEGWriter::close(void)
{ return false; }
bool FFMPEGWriter::isOpen(void) const
{ return false; }

#endif

