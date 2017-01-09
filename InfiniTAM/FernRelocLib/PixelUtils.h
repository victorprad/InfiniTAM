// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../ORUtils/Vector.h"

namespace FernRelocLib
{
	inline void createGaussianFilter(int masksize, float sigma, float *coeff)
	{
		int s2 = masksize / 2;
		for (int i = 0; i < masksize; ++i) coeff[i] = exp(-(i - s2)*(i - s2) / (2.0f*sigma*sigma));
	}

	inline void filterSeparable_x(const ORUtils::Image<float> *input, ORUtils::Image<float> *output, int masksize, const float *coeff)
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
			if (sum_c > 0.0f) v = sum_v / sum_c;
			else v = 0.0f;
			imageData_out[y*imgSize.x + x] = v;
		}
	}

	inline void filterSeparable_x(const ORUtils::Image<ORUtils::Vector4<unsigned char>> *input, ORUtils::Image<ORUtils::Vector4<unsigned char>> *output, int masksize, const float *coeff) 
	{
		int s2 = masksize / 2;
		ORUtils::Vector2<int> imgSize = input->noDims;
		output->ChangeDims(imgSize);

		const ORUtils::Vector4<unsigned char> *imageData_in = input->GetData(MEMORYDEVICE_CPU);
		ORUtils::Vector4<unsigned char> *imageData_out = output->GetData(MEMORYDEVICE_CPU);

		for (int y = 0; y < imgSize.y; y++) {
			for (int x = 0; x < imgSize.x; x++) {
				for (int n = 0; n < 3; n++) {
					float sum_v = 0.0f, sum_c = 0.0f, v;
					for (int i = 0; i < masksize; ++i) {
						if (x + i - s2 < 0) continue;
						if (x + i - s2 >= imgSize.x) continue;
						v = imageData_in[y * imgSize.x + x + i - s2][n];
#ifdef TREAT_HOLES
						if (!(v > 0)) continue;
#endif
						sum_c += coeff[i];
						sum_v += coeff[i] * v;
					}
					if (sum_c > 0) v = sum_v / sum_c;
					else v = 0.0f;
					imageData_out[y*imgSize.x + x][n] = (unsigned char)v;
				}
			}
		}
	}

	inline void filterSeparable_y(const ORUtils::Image<float> *input, ORUtils::Image<float> *output, int masksize, const float *coeff)
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
				v = imageData_in[(y + i - s2)*imgSize.x + x];
#ifdef TREAT_HOLES
				if (!(v > 0.0f)) continue;
#endif
				sum_c += coeff[i];
				sum_v += coeff[i] * v;
			}
			if (sum_c > 0.0f) v = sum_v / sum_c;
			else v = 0.0f;
			imageData_out[y*imgSize.x + x] = v;
		}
	}

	inline void filterSeparable_y(const ORUtils::Image<ORUtils::Vector4<unsigned char>> *input, ORUtils::Image<ORUtils::Vector4<unsigned char>> *output, int masksize, const float *coeff)
	{
		int s2 = masksize / 2;
		ORUtils::Vector2<int> imgSize = input->noDims;
		output->ChangeDims(imgSize);

		const ORUtils::Vector4<unsigned char> *imageData_in = input->GetData(MEMORYDEVICE_CPU);
		ORUtils::Vector4<unsigned char> *imageData_out = output->GetData(MEMORYDEVICE_CPU);

		for (int y = 0; y < imgSize.y; y++) {
			for (int x = 0; x < imgSize.x; x++) {
				for (int n = 0; n < 3; n++) {
					float sum_v = 0.0f, sum_c = 0.0f, v;
					for (int i = 0; i < masksize; ++i) {
						if (y + i - s2 < 0) continue;
						if (y + i - s2 >= imgSize.y) continue;
						v = imageData_in[(y + i - s2)*imgSize.x + x][n];
#ifdef TREAT_HOLES
						if (!(v > 0)) continue;
#endif
						sum_c += coeff[i];
						sum_v += coeff[i] * v;
					}
					if (sum_c > 0) v = sum_v / sum_c;
					else v = 0.0f;
					imageData_out[y*imgSize.x + x][n] = (unsigned char)v;
				}
			}
		}
	}

	template <typename T>
	inline void filterGaussian(const ORUtils::Image<T> *input, ORUtils::Image<T> *output, float sigma)
	{
		int filtersize = (int)(2.0f*3.5f*sigma);
		if ((filtersize & 1) == 0) filtersize += 1;
		float *coeff = new float[filtersize];
		ORUtils::Image<T> tmpimg(input->noDims, MEMORYDEVICE_CPU);

		createGaussianFilter(filtersize, sigma, coeff);
		filterSeparable_x(input, &tmpimg, filtersize, coeff);
		filterSeparable_y(&tmpimg, output, filtersize, coeff);
	}

	inline void filterSubsample(const ORUtils::Image<float> *input, ORUtils::Image<float> *output)
	{
		ORUtils::Vector2<int> imgSize_in = input->noDims;
		ORUtils::Vector2<int> imgSize_out(imgSize_in.x / 2, imgSize_in.y / 2);
		output->ChangeDims(imgSize_out, true);

		const float *imageData_in = input->GetData(MEMORYDEVICE_CPU);
		float *imageData_out = output->GetData(MEMORYDEVICE_CPU);

		for (int y = 0; y < imgSize_out.y; y++) for (int x = 0; x < imgSize_out.x; x++) {
			int x_src = x * 2;
			int y_src = y * 2;
			int num = 0; float sum = 0.0f;
			float v = imageData_in[x_src + y_src * imgSize_in.x];
#ifdef TREAT_HOLES
			if (v > 0.0f)
#endif
			{
				num++; sum += v;
			}
			v = imageData_in[x_src + 1 + y_src * imgSize_in.x];
#ifdef TREAT_HOLES
			if (v > 0.0f)
#endif
			{
				num++; sum += v;
			}
			v = imageData_in[x_src + (y_src + 1) * imgSize_in.x];
#ifdef TREAT_HOLES
			if (v > 0.0f)
#endif
			{
				num++; sum += v;
			}
			v = imageData_in[x_src + 1 + (y_src + 1) * imgSize_in.x];
#ifdef TREAT_HOLES
			if (v > 0.0f)
#endif
			{
				num++; sum += v;
			}

			if (num > 0) v = sum / (float)num;
			else v = 0.0f;
			imageData_out[x + y * imgSize_out.x] = v;
		}
	}

	inline void filterSubsample(const ORUtils::Image<ORUtils::Vector4<unsigned char>> *input, ORUtils::Image<ORUtils::Vector4<unsigned char>> *output) {
		ORUtils::Vector2<int> imgSize_in = input->noDims;
		ORUtils::Vector2<int> imgSize_out(imgSize_in.x / 2, imgSize_in.y / 2);
		output->ChangeDims(imgSize_out, true);

		const ORUtils::Vector4<unsigned char> *imageData_in = input->GetData(MEMORYDEVICE_CPU);
		ORUtils::Vector4<unsigned char> *imageData_out = output->GetData(MEMORYDEVICE_CPU);

		for (int y = 0; y < imgSize_out.y; y++) {
			for (int x = 0; x < imgSize_out.x; x++) {
				int x_src = x * 2;
				int y_src = y * 2;
				unsigned char num = 0; ORUtils::Vector4<unsigned char> sum(0, 0, 0, 0);
				ORUtils::Vector4<unsigned char> v = imageData_in[x_src + y_src * imgSize_in.x];
#ifdef TREAT_HOLES
				if (v[3] > 0)
#endif
				{
					num++; sum += v;
				}
				v = imageData_in[x_src + 1 + y_src * imgSize_in.x];
#ifdef TREAT_HOLES
				if (v[3] > 0)
#endif
				{
					num++; sum += v;
				}
				v = imageData_in[x_src + (y_src + 1) * imgSize_in.x];
#ifdef TREAT_HOLES
				if (v[3] > 0)
#endif
				{
					num++; sum += v;
				}
				v = imageData_in[x_src + 1 + (y_src + 1) * imgSize_in.x];
#ifdef TREAT_HOLES
				if (v[3] > 0)
#endif
				{
					num++; sum += v;
				}
				if (num > 0) v = sum / num;
				else v[0] = v[1] = v[2] = v[3] = 0;
				imageData_out[x + y * imgSize_out.x] = v;
			}
		}
	}
}
