// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#include <vector>

namespace ORUtils
{
	template<class F>
	class GenericCholesky
	{
	private:
		std::vector<F> cholesky;
		int size, rank;

	public:
		GenericCholesky(const F *mat, int size)
		{
			this->size = size;
			this->cholesky.resize(size*size);

			for (int i = 0; i < size * size; i++) cholesky[i] = mat[i];

			for (int c = 0; c < size; c++)
			{
				F inv_diag = 1;
				for (int r = c; r < size; r++)
				{
					F val = cholesky[c + r * size];
					for (int c2 = 0; c2 < c; c2++) 
						val -= cholesky[c + c2 * size] * cholesky[c2 + r * size];

					if (r == c)
					{
						cholesky[c + r * size] = val;
						if (val == 0) { rank = r; }
						inv_diag = 1.0f / val;
					}
					else
					{
						cholesky[r + c * size] = val;
						cholesky[c + r * size] = val * inv_diag;
					}
				}
			}

			rank = size;
		}

		F Determinant(void) const
		{
			F ret = 1.0f;
			for (int i = 0; i < size; ++i) {
				ret *= cholesky[i + i * size];
			}
			return ret * ret;
		}

		void Backsub(F *result, const F *v) const
		{
			std::vector<F> y(size);
			for (int i = 0; i < size; i++)
			{
				F val = v[i];
				for (int j = 0; j < i; j++) val -= cholesky[j + i * size] * y[j];
				y[i] = val;
			}

			for (int i = 0; i < size; i++) y[i] /= cholesky[i + i * size];

			for (int i = size - 1; i >= 0; i--)
			{
				F val = y[i];
				for (int j = i + 1; j < size; j++) val -= cholesky[i + j * size] * result[j];
				result[i] = val;
			}
		}

		~GenericCholesky(void)
		{
		}
	};

	typedef GenericCholesky<float> Cholesky;
}
