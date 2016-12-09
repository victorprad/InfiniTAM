// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#include "FileUtils.h"

#include <stdio.h>
#include <fstream>

#if defined _MSC_VER
#include <direct.h>
#else 
#include <sys/types.h>
#include <sys/stat.h>
#endif

#ifdef USE_LIBPNG
#include <png.h>
#endif

using namespace std;

static const char *pgm_ascii_id = "P2";
static const char *ppm_ascii_id = "P3";
static const char *pgm_id = "P5";
static const char *ppm_id = "P6";

typedef enum { MONO_8u, RGB_8u, MONO_16u, MONO_16s, RGBA_8u, FORMAT_UNKNOWN = -1 } FormatType;

struct PNGReaderData {
#ifdef USE_LIBPNG
	png_structp png_ptr;
	png_infop info_ptr;

	PNGReaderData(void)
	{ png_ptr = NULL; info_ptr = NULL; }
	~PNGReaderData(void)
	{ 
		if (info_ptr != NULL) png_destroy_info_struct(png_ptr, &info_ptr);
		if (png_ptr != NULL) png_destroy_read_struct(&png_ptr, (png_infopp)NULL, (png_infopp)NULL);
	}
#endif
};

static FormatType png_readheader(FILE *fp, int & width, int & height, PNGReaderData & internal)
{
	FormatType type = FORMAT_UNKNOWN;

#ifdef USE_LIBPNG
	png_byte color_type;
	png_byte bit_depth;

	unsigned char header[8];    // 8 is the maximum size that can be checked

	fread(header, 1, 8, fp);
	if (png_sig_cmp(header, 0, 8)) {
		//"not a PNG file"
		return type;
	}

	/* initialize stuff */
	internal.png_ptr = png_create_read_struct(PNG_LIBPNG_VER_STRING, NULL, NULL, NULL);

	if (!internal.png_ptr) {
		//"png_create_read_struct failed"
		return type;
	}

	internal.info_ptr = png_create_info_struct(internal.png_ptr);
	if (!internal.info_ptr) {
		//"png_create_info_struct failed"
		return type;
	}

	if (setjmp(png_jmpbuf(internal.png_ptr))) {
		//"setjmp failed"
		return type;
	}

	png_init_io(internal.png_ptr, fp);
	png_set_sig_bytes(internal.png_ptr, 8);

	png_read_info(internal.png_ptr, internal.info_ptr);

	width = png_get_image_width(internal.png_ptr, internal.info_ptr);
	height = png_get_image_height(internal.png_ptr, internal.info_ptr);
	color_type = png_get_color_type(internal.png_ptr, internal.info_ptr);
	bit_depth = png_get_bit_depth(internal.png_ptr, internal.info_ptr);

	if (color_type == PNG_COLOR_TYPE_GRAY) {
		if (bit_depth == 8) type = MONO_8u;
		else if (bit_depth == 16) type = MONO_16u;
		// bit depths 1, 2 and 4 are not accepted
	} else if (color_type == PNG_COLOR_TYPE_RGB) {
		if (bit_depth == 8) type = RGB_8u;
		// bit depth 16 is not accepted
	} else if (color_type == PNG_COLOR_TYPE_RGBA) {
		if (bit_depth == 8) type = RGBA_8u;
		// bit depth 16 is not accepted
	}
	// other color types are not accepted
#endif

	return type;
}

static bool png_readdata(FILE *f, int xsize, int ysize, PNGReaderData & internal, void *data_ext)
{
#ifdef USE_LIBPNG
	if (setjmp(png_jmpbuf(internal.png_ptr))) return false;

	png_read_update_info(internal.png_ptr, internal.info_ptr);

	/* read file */
	if (setjmp(png_jmpbuf(internal.png_ptr))) return false;

	int bytesPerRow = (int)png_get_rowbytes(internal.png_ptr, internal.info_ptr);

	png_byte *data = (png_byte*)data_ext;
	png_bytep *row_pointers = new png_bytep[ysize];
	for (int y=0; y<ysize; y++) row_pointers[y] = &(data[bytesPerRow*y]);

	png_read_image(internal.png_ptr, row_pointers);
	png_read_end(internal.png_ptr, NULL);

	delete[] row_pointers;

	return true;
#else
	return false;
#endif
}

static FormatType pnm_readheader(FILE *f, int *xsize, int *ysize, bool *binary)
{
	char tmp[1024];
	FormatType type = FORMAT_UNKNOWN;
	int xs = 0, ys = 0, max_i = 0;
	bool isBinary = true;

	/* read identifier */
	if (fscanf(f, "%[^ \n\t]", tmp) != 1) return type;
	if (!strcmp(tmp, pgm_id)) type = MONO_8u;
	else if (!strcmp(tmp, pgm_ascii_id)) { type = MONO_8u; isBinary = false; }
	else if (!strcmp(tmp, ppm_id)) type = RGB_8u;
	else if (!strcmp(tmp, ppm_ascii_id)) { type = RGB_8u; isBinary = false; }
	else return type;

	/* read size */
	if (!fscanf(f, "%i", &xs)) return FORMAT_UNKNOWN;
	if (!fscanf(f, "%i", &ys)) return FORMAT_UNKNOWN;

	if (!fscanf(f, "%i", &max_i)) return FORMAT_UNKNOWN;
	if (max_i < 0) return FORMAT_UNKNOWN;
	else if (max_i <= (1 << 8)) {}
	else if ((max_i <= (1 << 15)) && (type == MONO_8u)) type = MONO_16s;
	else if ((max_i <= (1 << 16)) && (type == MONO_8u)) type = MONO_16u;
	else return FORMAT_UNKNOWN;
	fgetc(f);

	if (xsize) *xsize = xs;
	if (ysize) *ysize = ys;
	if (binary) *binary = isBinary;

	return type;
}

template<class T>
static bool pnm_readdata_ascii_helper(FILE *f, int xsize, int ysize, int channels, T *data)
{
	for (int y = 0; y < ysize; ++y) for (int x = 0; x < xsize; ++x) for (int c = 0; c < channels; ++c) {
		int v;
		if (!fscanf(f, "%i", &v)) return false;
		*data++ = v;
	}
	return true;
}

static bool pnm_readdata_ascii(FILE *f, int xsize, int ysize, FormatType type, void *data)
{
	int channels = 0;
	switch (type)
	{
	case MONO_8u:
		channels = 1;
		return pnm_readdata_ascii_helper(f, xsize, ysize, channels, (unsigned char*)data);
	case RGB_8u:
		channels = 3;
		return pnm_readdata_ascii_helper(f, xsize, ysize, channels, (unsigned char*)data);
	case MONO_16s:
		channels = 1;
		return pnm_readdata_ascii_helper(f, xsize, ysize, channels, (short*)data);
	case MONO_16u:
		channels = 1;
		return pnm_readdata_ascii_helper(f, xsize, ysize, channels, (unsigned short*)data);
	case FORMAT_UNKNOWN:
	default: break;
	}
	return false;
}

static bool pnm_readdata_binary(FILE *f, int xsize, int ysize, FormatType type, void *data)
{
	int channels = 0;
	int bytesPerSample = 0;
	switch (type)
	{
	case MONO_8u: bytesPerSample = sizeof(unsigned char); channels = 1; break;
	case RGB_8u: bytesPerSample = sizeof(unsigned char); channels = 3; break;
	case MONO_16s: bytesPerSample = sizeof(short); channels = 1; break;
	case MONO_16u: bytesPerSample = sizeof(unsigned short); channels = 1; break;
	case FORMAT_UNKNOWN:
	default: break;
	}
	if (bytesPerSample == 0) return false;

	size_t tmp = fread(data, bytesPerSample, xsize*ysize*channels, f);
	if (tmp != (size_t)xsize*ysize*channels) return false;
	return (data != NULL);
}

static bool pnm_writeheader(FILE *f, int xsize, int ysize, FormatType type)
{
	const char *pnmid = NULL;
	int max = 0;
	switch (type) {
	case MONO_8u: pnmid = pgm_id; max = 256; break;
	case RGB_8u: pnmid = ppm_id; max = 255; break;
	case MONO_16s: pnmid = pgm_id; max = 32767; break;
	case MONO_16u: pnmid = pgm_id; max = 65535; break;
	case FORMAT_UNKNOWN:
	default: return false;
	}
	if (pnmid == NULL) return false;

	fprintf(f, "%s\n", pnmid);
	fprintf(f, "%i %i\n", xsize, ysize);
	fprintf(f, "%i\n", max);

	return true;
}

static bool pnm_writedata(FILE *f, int xsize, int ysize, FormatType type, const void *data)
{
	int channels = 0;
	int bytesPerSample = 0;
	switch (type)
	{
	case MONO_8u: bytesPerSample = sizeof(unsigned char); channels = 1; break;
	case RGB_8u: bytesPerSample = sizeof(unsigned char); channels = 3; break;
	case MONO_16s: bytesPerSample = sizeof(short); channels = 1; break;
	case MONO_16u: bytesPerSample = sizeof(unsigned short); channels = 1; break;
	case FORMAT_UNKNOWN:
	default: break;
	}
	fwrite(data, bytesPerSample, channels*xsize*ysize, f);
	return true;
}

void SaveImageToFile(const ORUtils::Image<ORUtils::Vector4<unsigned char> > * image, const char* fileName, bool flipVertical)
{
	FILE *f = fopen(fileName, "wb");
	if (!pnm_writeheader(f, image->noDims.x, image->noDims.y, RGB_8u)) {
		fclose(f); return;
	}

	unsigned char *data = new unsigned char[image->noDims.x*image->noDims.y * 3];

	ORUtils::Vector2<int> noDims = image->noDims;

	if (flipVertical)
	{
		for (int y = 0; y < noDims.y; y++) for (int x = 0; x < noDims.x; x++)
		{
			int locId_src, locId_dst;
			locId_src = x + y * noDims.x;
			locId_dst = x + (noDims.y - y - 1) * noDims.x;

			data[locId_dst * 3 + 0] = image->GetData(MEMORYDEVICE_CPU)[locId_src].x;
			data[locId_dst * 3 + 1] = image->GetData(MEMORYDEVICE_CPU)[locId_src].y;
			data[locId_dst * 3 + 2] = image->GetData(MEMORYDEVICE_CPU)[locId_src].z;
		}
	}
	else
	{
		for (int i = 0; i < noDims.x * noDims.y; ++i) {
			data[i * 3 + 0] = image->GetData(MEMORYDEVICE_CPU)[i].x;
			data[i * 3 + 1] = image->GetData(MEMORYDEVICE_CPU)[i].y;
			data[i * 3 + 2] = image->GetData(MEMORYDEVICE_CPU)[i].z;
		}
	}

	pnm_writedata(f, image->noDims.x, image->noDims.y, RGB_8u, data);
	delete[] data;
	fclose(f);
}

void SaveImageToFile(const ORUtils::Image<short>* image, const char* fileName)
{
	short *data = (short*)malloc(sizeof(short) * image->dataSize);
	const short *dataSource = image->GetData(MEMORYDEVICE_CPU);
	for (size_t i = 0; i < image->dataSize; i++) data[i] = (dataSource[i] << 8) | ((dataSource[i] >> 8) & 255);

	FILE *f = fopen(fileName, "wb");
	if (!pnm_writeheader(f, image->noDims.x, image->noDims.y, MONO_16u)) {
		fclose(f); return;
	}
	pnm_writedata(f, image->noDims.x, image->noDims.y, MONO_16u, data);
	fclose(f);

	delete data;
}

void SaveImageToFile(const ORUtils::Image<float>* image, const char* fileName)
{
	unsigned short *data = new unsigned short[image->dataSize];
	for (size_t i = 0; i < image->dataSize; i++)
	{
		float localData = image->GetData(MEMORYDEVICE_CPU)[i];
		data[i] = localData >= 0 ? (unsigned short)(localData * 1000.0f) : 0;
	}

	FILE *f = fopen(fileName, "wb");
	if (!pnm_writeheader(f, image->noDims.x, image->noDims.y, MONO_16u)) {
		fclose(f); return;
	}
	pnm_writedata(f, image->noDims.x, image->noDims.y, MONO_16u, data);
	fclose(f);

	delete[] data;
}

bool ReadImageFromFile(ORUtils::Image<ORUtils::Vector4<unsigned char> > * image, const char* fileName)
{
	PNGReaderData pngData;
	bool usepng = false;

	int xsize, ysize;
	FormatType type;
	bool binary;
	FILE *f = fopen(fileName, "rb");
	if (f == NULL) return false;
	type = pnm_readheader(f, &xsize, &ysize, &binary);
	if ((type != RGB_8u)&&(type != RGBA_8u)) {
		fclose(f);
		f = fopen(fileName, "rb");
		type = png_readheader(f, xsize, ysize, pngData);
		if ((type != RGB_8u)&&(type != RGBA_8u)) {
			fclose(f);
			return false;
		}
		usepng = true;
	}

	ORUtils::Vector2<int> newSize(xsize, ysize);
	image->ChangeDims(newSize);
	ORUtils::Vector4<unsigned char> *dataPtr = image->GetData(MEMORYDEVICE_CPU);

	unsigned char *data;
	if (type != RGBA_8u) data = new unsigned char[xsize*ysize * 3];
	else data = (unsigned char*)image->GetData(MEMORYDEVICE_CPU);

	if (usepng) {
		if (!png_readdata(f, xsize, ysize, pngData, data)) { fclose(f); delete[] data; return false; }
	} else if (binary) {
		if (!pnm_readdata_binary(f, xsize, ysize, RGB_8u, data)) { fclose(f); delete[] data; return false; }
	} else {
		if (!pnm_readdata_ascii(f, xsize, ysize, RGB_8u, data)) { fclose(f); delete[] data; return false; }
	}
	fclose(f);

	if (type != RGBA_8u)
	{
		for (int i = 0; i < image->noDims.x*image->noDims.y; ++i)
		{
			dataPtr[i].x = data[i * 3 + 0]; dataPtr[i].y = data[i * 3 + 1];
			dataPtr[i].z = data[i * 3 + 2]; dataPtr[i].w = 255;
		}

		delete[] data;
	}

	return true;
}

bool ReadImageFromFile(ORUtils::Image<short> *image, const char *fileName)
{
	PNGReaderData pngData;
	bool usepng = false;

	int xsize, ysize;
	bool binary;
	FILE *f = fopen(fileName, "rb");
	if (f == NULL) return false;
	FormatType type = pnm_readheader(f, &xsize, &ysize, &binary);
	if ((type != MONO_16s) && (type != MONO_16u)) {
		fclose(f);
		f = fopen(fileName, "rb");
		type = png_readheader(f, xsize, ysize, pngData);
		if ((type != MONO_16s) && (type != MONO_16u)) {
			fclose(f);
			return false;
		}
		usepng = true;
		binary = true;
	}

	short *data = new short[xsize*ysize];
	if (usepng) {
		if (!png_readdata(f, xsize, ysize, pngData, data)) { fclose(f); delete[] data; return false; }
	} else if (binary) {
		if (!pnm_readdata_binary(f, xsize, ysize, type, data)) { fclose(f); delete[] data; return false; }
	} else {
		if (!pnm_readdata_ascii(f, xsize, ysize, type, data)) { fclose(f); delete[] data; return false; }
	}
	fclose(f);

	ORUtils::Vector2<int> newSize(xsize, ysize);
	image->ChangeDims(newSize);
	if (binary) {
		for (int i = 0; i < image->noDims.x*image->noDims.y; ++i) {
			image->GetData(MEMORYDEVICE_CPU)[i] = (data[i] << 8) | ((data[i] >> 8) & 255);
		}
	} else {
		for (int i = 0; i < image->noDims.x*image->noDims.y; ++i) {
			image->GetData(MEMORYDEVICE_CPU)[i] = data[i];
		}
	}
	delete[] data;

	return true;
}

void MakeDir(const char *dirName)
{
#if defined _MSC_VER
		_mkdir(dirName);
#else
		mkdir(dirName, 0777);
#endif
}
