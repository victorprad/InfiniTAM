// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#include "MathUtils.h"
#include <math.h>
#include <string.h>

/*This class is based on the homkermap.c functions from VLFeat www.vlfeat.org */

namespace ORUtils
{
	class HomkerMap
	{
	private:
		float gamma; int order; float period;
		float subdivision; int numSubdivisions;
		int minExponent, maxExponent;
		float *table;

	private:
		float vl_homogeneouskernelmap_get_spectrum(float omega)
		{
			return 2.0f / (exp(PI * omega) + exp(-PI * omega));
		}

		float sinc(float x)
		{
			if (x == 0.0f) return 1.0f;
			return sin(x) / x;
		}

		float getSmoothSpectrum(float period, float omega)
		{
			float kappa_hat = 0;
			float omegap;
			float epsilon = 1e-2f;
			float const omegaRange = 2.0f / (period * epsilon);
			float const domega = 2.0f * omegaRange / (2.0f * 1024.0f + 1.0f);

			for (omegap = -omegaRange; omegap <= omegaRange; omegap += domega)
			{
				float win = sinc((period / 2.0f) * omegap);
				win *= (period / (2.0f * PI));
				kappa_hat += win * vl_homogeneouskernelmap_get_spectrum(omegap + omega);
			}

			kappa_hat *= domega;
			kappa_hat = MAX(kappa_hat, 0.0f);

			return kappa_hat;
		}

	public:
		HomkerMap(int order)
		{
			int tableWidth, tableHeight;

			period = 8.80f * sqrt(order + 4.44f) - 12.6f;

			period = MAX(period, 1.0f);

			gamma = 1.0f;
			this->order = order;
			numSubdivisions = 8 + 8 * order;
			subdivision = 1.0f / numSubdivisions;
			minExponent = -20;
			maxExponent = 8;

			tableHeight = (int)(2 * order + 1);
			tableWidth = (int)(numSubdivisions * (maxExponent - minExponent + 1));

			table = new float[tableHeight * tableWidth + 2 * (1 + order)];
			memset(table, 0, sizeof(float) * tableHeight * tableWidth + 2 * (1 + order));

			int exponent;
			int i, j;
			float *tablep = table;
			float *kappa = table + tableHeight * tableWidth;
			float *freq = kappa + (1 + order);
			float L = 2.0f * PI / period;

			/* precompute the sampled periodicized spectrum */
			j = 0; i = 0;
			while (i <= order)
			{
				freq[i] = (float)j;
				kappa[i] = getSmoothSpectrum(period, j * L);
				++j;
				if (kappa[i] > 0 || j >= 3 * i) ++i;
			}

			/* fill table */
			for (exponent = minExponent; exponent <= maxExponent; ++exponent) 
			{

				float x, Lxgamma, Llogx, xgamma;
				float sqrt2kappaLxgamma;
				float mantissa = 1.0f;

				for (i = 0; i < numSubdivisions; ++i, mantissa += subdivision) 
				{
					x = ldexp(mantissa, (int)exponent);
					xgamma = pow(x, gamma);
					Lxgamma = L * xgamma;
					Llogx = L * log(x);

					*tablep++ = sqrt(Lxgamma * kappa[0]);
					for (j = 1; j <= order; ++j) 
					{
						sqrt2kappaLxgamma = sqrt(2.0f * Lxgamma * kappa[j]);
						*tablep++ = sqrt2kappaLxgamma * cos(freq[j] * Llogx);
						*tablep++ = sqrt2kappaLxgamma * sin(freq[j] * Llogx);
					}
				} /* next mantissa */
			} /* next exponent */
		}

		void evaluate(float *destination, int stride, float x)
		{
			/* break value into exponent and mantissa */
			int exponent;
			int j;
			float mantissa = frexp(x, &exponent);
			float sign = (mantissa >= 0.0f) ? +1.0f : -1.0f;
			mantissa *= 2.0f * sign;
			exponent--;

			if (mantissa == 0 || exponent <= minExponent || exponent >= maxExponent)
			{
				for (j = 0; j < 2 * order + 1; ++j)
				{
					*destination = (float) 0.0f;
					destination += stride;
				}
				return;
			}

			int featureDimension = 2 * order + 1;
			float const *v1 = table + (exponent - minExponent) * numSubdivisions * featureDimension;
			float const * v2;
			float f1, f2;

			mantissa -= 1.0f;
			while (mantissa >= subdivision) 
			{
				mantissa -= subdivision;
				v1 += featureDimension;
			}

			v2 = v1 + featureDimension;

			for (j = 0; j < featureDimension; ++j) {
				f1 = *v1++;
				f2 = *v2++;
				*destination = (float)sign * ((f2 - f1) * (numSubdivisions * mantissa) + f1);
				destination += stride;
			}
		}

		inline int getDescriptorSize() { return 2 * order + 1; }
		inline int getDescriptorSize(int inputSize) { return inputSize * (2 * order + 1); }

		void evaluate(float *destination, float *input, int inputSize)
		{
			memset(destination, 0, sizeof(float) * getDescriptorSize(inputSize));
			
			int position = 0;
			for (int j = 0; j < inputSize; ++j)
			{
				evaluate(&destination[position], 1, *input++);
				position += getDescriptorSize();
			}
		}

		~HomkerMap()
		{
			delete table;
			table = NULL;
		}
	};
}