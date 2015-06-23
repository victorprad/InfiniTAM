// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM

#pragma once

#include <stdio.h>

#include "../ITMLib/Utils/ITMLibDefines.h"

void SaveImageToFile(const ITMUChar4Image* image, const char* fileName, bool flipVertical = false);
void SaveImageToFile(const ITMShortImage* image, const char* fileName);
void SaveImageToFile(const ITMFloatImage* image, const char* fileName);
bool ReadImageFromFile(ITMUChar4Image* image, const char* fileName);
bool ReadImageFromFile(ITMShortImage *image, const char *fileName);

template <typename T> void ReadFromBIN(T *data, int dataSize, const char *fileName)
{
	FILE *f = fopen(fileName, "rb");
	fread(data, dataSize * sizeof(T), 1, f);
	fclose(f);
}

template <typename T> void WriteToBIN(const T *data, int dataSize, const char *fileName)
{
	FILE *f = fopen(fileName, "wb");
	fwrite(data, dataSize * sizeof(T), 1, f);
	fclose(f);
}

