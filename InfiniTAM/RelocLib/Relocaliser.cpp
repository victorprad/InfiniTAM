// Copyright 2014-2016 Isis Innovation Limited and the authors of InfiniTAM

#include "Relocaliser.h"

#define TREAT_HOLES

using namespace RelocLib;

Relocaliser::Relocaliser(ORUtils::Vector2<int> imgSize, ORUtils::Vector2<float> range, float harvestingThreshold, int numFerns, int numDecisionsPerFern)
{
	static const int levels = 5;
	mEncoding = new FernConservatory(numFerns, imgSize / (1 << levels), range, numDecisionsPerFern);
	mDatabase = new RelocDatabase(numFerns, mEncoding->getNumCodes());
	mKeyframeHarvestingThreshold = harvestingThreshold;

	processedImage1 = new ORUtils::Image<float>(imgSize, MEMORYDEVICE_CPU);
	processedImage2 = new ORUtils::Image<float>(imgSize, MEMORYDEVICE_CPU);
}

Relocaliser::~Relocaliser(void)
{
	delete mEncoding;
	delete mDatabase;
	delete processedImage1;
	delete processedImage2;
}

void createGaussianFilter(int masksize, float sigma, float *coeff)
{
	int s2 = masksize/2;
	for (int i = 0; i < masksize; ++i) coeff[i] = exp(-(i-s2)*(i-s2)/(2.0f*sigma*sigma));
}

void filterSeparable_x(const ORUtils::Image<float> *input, ORUtils::Image<float> *output, int masksize, const float *coeff)
{
	int s2 = masksize / 2;
	ORUtils::Vector2<int> imgSize = input->noDims;
	output->ChangeDims(imgSize);

	const float *imageData_in = input->GetData(MEMORYDEVICE_CPU);
	float *imageData_out = output->GetData(MEMORYDEVICE_CPU);

	for (int y = 0; y < imgSize.y; y++) for (int x = 0; x < imgSize.x; x++) {
		float sum_v = 0.0f;
		float sum_c = 0.0f;
		float v;
		for (int i = 0; i < masksize; ++i) {
			if (x + i - s2 < 0) continue;
			if (x + i - s2 >= imgSize.x) continue;
			v = imageData_in[y*imgSize.x + x + i - s2];
#ifdef TREAT_HOLES
			if (!(v > 0.0f)) continue;
#endif
			sum_c += coeff[i];
			sum_v += coeff[i] * v;
		}
		if (sum_c > 0.0f) v = sum_v/sum_c;
		else v = 0.0f;
		imageData_out[y*imgSize.x + x] = v;
	}
}

void filterSeparable_y(const ORUtils::Image<float> *input, ORUtils::Image<float> *output, int masksize, const float *coeff)
{
	int s2 = masksize / 2;
	ORUtils::Vector2<int> imgSize = input->noDims;
	output->ChangeDims(imgSize);

	const float *imageData_in = input->GetData(MEMORYDEVICE_CPU);
	float *imageData_out = output->GetData(MEMORYDEVICE_CPU);

	for (int y = 0; y < imgSize.y; y++) for (int x = 0; x < imgSize.x; x++) {
		float sum_v = 0.0f;
		float sum_c = 0.0f;
		float v;
		for (int i = 0; i < masksize; ++i) {
			if (y + i - s2 < 0) continue;
			if (y + i - s2 >= imgSize.y) continue;
			v = imageData_in[(y+i-s2)*imgSize.x + x];
#ifdef TREAT_HOLES
			if (!(v > 0.0f)) continue;
#endif
			sum_c += coeff[i];
			sum_v += coeff[i] * v;
		}
		if (sum_c > 0.0f) v = sum_v/sum_c;
		else v = 0.0f;
		imageData_out[y*imgSize.x + x] = v;
	}
}

void filterGaussian(const ORUtils::Image<float> *input, ORUtils::Image<float> *output, float sigma)
{
	int filtersize = (int)(2.0f*3.5f*sigma);
	if ((filtersize & 1) == 0) filtersize += 1;
	float *coeff = new float[filtersize];
	ORUtils::Image<float> tmpimg(input->noDims, MEMORYDEVICE_CPU);

	createGaussianFilter(filtersize, sigma, coeff);
	filterSeparable_x(input, &tmpimg, filtersize, coeff);
	filterSeparable_y(&tmpimg, output, filtersize, coeff);
}

void filterSubsample(const ORUtils::Image<float> *input, ORUtils::Image<float> *output)
{
	ORUtils::Vector2<int> imgSize_in = input->noDims;
	ORUtils::Vector2<int> imgSize_out(imgSize_in.x/2, imgSize_in.y/2);
	output->ChangeDims(imgSize_out, false);

	const float *imageData_in = input->GetData(MEMORYDEVICE_CPU);
	float *imageData_out = output->GetData(MEMORYDEVICE_CPU);

	for (int y = 0; y < imgSize_out.y; y++) for (int x = 0; x < imgSize_out.x; x++) {
		int x_src = x*2;
		int y_src = y*2;
		int num = 0; float sum = 0.0f;
		float v = imageData_in[x_src + y_src * imgSize_in.x];
#ifdef TREAT_HOLES
		if (v > 0.0f)
#endif
		{ num++; sum+=v; }
		v = imageData_in[x_src + 1 + y_src * imgSize_in.x];
#ifdef TREAT_HOLES
		if (v > 0.0f)
#endif
		{ num++; sum+=v; }
		v = imageData_in[x_src + (y_src + 1) * imgSize_in.x];
#ifdef TREAT_HOLES
		if (v > 0.0f) 
#endif
		{ num++; sum+=v; }
		v = imageData_in[x_src + 1 + (y_src + 1) * imgSize_in.x];
#ifdef TREAT_HOLES
		if (v > 0.0f)
#endif
		{ num++; sum+=v; }

		if (num > 0) v = sum/(float)num;
		else v = 0.0f;
		imageData_out[x + y * imgSize_out.x] = v;
	}
}

int Relocaliser::ProcessFrame(const ORUtils::Image<float> *img_d, int k, int nearestNeighbours[], float *distances, bool harvestKeyframes) const
{
	// downsample and preprocess image => processedImage1
	//ORUtils::Image<float> processedImage1(ORUtils::Vector2<int>(1,1), MEMORYDEVICE_CPU), processedImage2(ORUtils::Vector2<int>(1,1), MEMORYDEVICE_CPU);
	filterSubsample(img_d, processedImage1); // 320x240
	filterSubsample(processedImage1, processedImage2); // 160x120
	filterSubsample(processedImage2, processedImage1); // 80x60
	filterSubsample(processedImage1, processedImage2); // 40x30
	
	filterGaussian(processedImage2, processedImage1, 2.5f);

	// compute code
	int codeLength = mEncoding->getNumFerns();
	char *code = new char[codeLength];
	mEncoding->computeCode(processedImage1, code);
	//for (int i = 0; i < codeLength; ++i) fprintf(stderr, "%i ", code[i]);
	//fprintf(stderr, "\n");

	// prepare outputs
	int ret = -1;
	bool releaseDistances = (distances == NULL);
	if (distances == NULL) distances = new float[k];

	// find similar frames
	int similarFound = mDatabase->findMostSimilar(code, nearestNeighbours, distances, k);

	// add keyframe to database
	if (harvestKeyframes) {
		if (similarFound == 0) ret = mDatabase->addEntry(code);
		else if (distances[0] > mKeyframeHarvestingThreshold) ret = mDatabase->addEntry(code);
	}

/*
	for (int i = 0; i < similarFound; ++i) {
		fprintf(stderr, "found similar entry #%i: %i (%f)\n", i+1, nearestNeighbours[i], distances[i]);
	}
*/

	// cleanup and return
	delete[] code;
	if (releaseDistances) delete[] distances;
	return ret;
}

