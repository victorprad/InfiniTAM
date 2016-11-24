// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#include "LevenbergMarquardtMethod.h"
#include "../ORUtils/MathUtils.h"

#include <vector>
#include <math.h>

//#define DEBUG
//#include <stdio.h>

using namespace MiniSlamGraph;

static const double TR_QUALITY_GAMMA1 = 0.75;
static const double TR_QUALITY_GAMMA2 = 0.25;
static const double TR_REGION_INCREASE = 2.0;
static const double TR_REGION_DECREASE = 0.25;
static const double MIN_STEPLENGTH = 1e-6f;
static const int MAX_NUMBER_STEPS = 100;
static const double MIN_DECREASE = 1e-6f;

bool stepConsideredSmallMAX(const SlamGraphErrorFunction & f, const double *step)
{
	double MAXnorm = 0.0;
	for (int i = 0; i < f.numParameters(); i++) {
		double tmp = fabs(step[i]);
		if (tmp > MAXnorm) MAXnorm = tmp;
	}

	return (MAXnorm < MIN_STEPLENGTH);
}

static inline double stepQuality(SlamGraphErrorFunction::EvaluationPoint *x, SlamGraphErrorFunction::EvaluationPoint *x2, const double *step, const double *grad, const Matrix *B)
{
	int numPara = B->numRows();
	double actual_reduction = x->f() - x2->f();
	double predicted_reduction = 0.0;
	double *tmp = new double[numPara];
	B->multiply(step, tmp);
	for (int i = 0; i < numPara; i++) {
		predicted_reduction -= grad[i] * step[i] + 0.5*step[i] * tmp[i];
	}
	if (predicted_reduction < 0) {
		delete[] tmp;
		return actual_reduction / fabs(predicted_reduction);
	}
	delete[] tmp;
	return actual_reduction / predicted_reduction;
}

int LevenbergMarquardtMethod::minimize(const SlamGraphErrorFunction & f, SlamGraphErrorFunction::Parameters & initialization)
{
	int ret = 0;
	int numPara = f.numParameters();
	std::vector<double> d(numPara);
	double lambda = 0.01;
	int step_counter = 0;

	SlamGraphErrorFunction::EvaluationPoint *x = f.evaluateAt(initialization.clone());
	SlamGraphErrorFunction::EvaluationPoint *x2 = NULL;
	initialization.clear();

	if (!portable_finite((float)x->f())) {
		delete x;
		return -1;
	}

	do {
		// debug output
#ifdef DEBUG
		fprintf(stderr, "step number: %i\n", step_counter);
		//x->getParameter().print();
		fprintf(stderr, "function value: %f\n", x->f());
#endif

#ifdef DEBUG
		fprintf(stderr, "LM: lambda %f\n", lambda);
#endif
		const double *grad;
		const Matrix *B;

		grad = x->nabla_f();
		B = x->hessian_GN();

		bool success;
		{
			Matrix *A = B->clone();
			/*if (regularize_sphere) A->addDiagonal(lambda);
			else*/ A->multDiagonal(lambda);
			success = A->solve(grad, &(d[0]));
			delete A;
		}

		if (success) {
			if (stepConsideredSmallMAX(f, &(d[0]))) break;
			for (int i = 0; i < numPara; i++) d[i] = -d[i];
			// make step
			SlamGraphErrorFunction::Parameters *tmp_para = x->getParameter().clone();
			f.applyDelta(x->getParameter(), &(d[0]), *tmp_para);

			// check whether step reduces error function and
			// compute a new value of lambda
			x2 = f.evaluateAt(tmp_para);
			double q = stepQuality(x, x2, &(d[0]), grad, B);
			if (q > TR_QUALITY_GAMMA1) {
				// very successful step
				success = true;
				lambda = lambda / TR_REGION_INCREASE;
			}
			else if (q > TR_QUALITY_GAMMA2) {
				// kind of successful step
				success = true;
				//lambda = lambda; //lambda doesn't change
			}
			else {
				// step failed
				success = false;
				lambda = lambda / TR_REGION_DECREASE;
			}
		}
		else {
			x2 = NULL;
			// can't compute a step quality here...
			lambda = lambda / TR_REGION_DECREASE;
		}

		if (success) {
			// accept step
#ifdef DEBUG
			fprintf(stderr, "accept %p (new function value: %f)\n", (void*)x2, x2->f());
#endif

			bool continueIteration = true;
			// did the function decrease sufficiently?
			if (!(x2->f() < (x->f() - fabs(x->f()) * MIN_DECREASE))) continueIteration = false;

			delete x;
			x = x2;

			if (!continueIteration) break;
		}
		else {
#ifdef DEBUG
			if (x2 != NULL) fprintf(stderr, "reject %p (function value would increase to: %f)\n", (void*)x2, x2->f());
			else fprintf(stderr, "reject (could not solve for new parameters)\n");
#endif
			if (x2 != NULL) delete x2;
		}
		// C allows a nice syntax with ->, --> and even ++>
		// ...just mentioned to make bored programmers happy
		if (step_counter++ >= MAX_NUMBER_STEPS) break;
	} while (1);

	initialization.copyFrom(x->getParameter());
	delete x;

#ifdef DEBUG
	fprintf(stderr, "total number of steps: %i\n", step_counter);
#endif
	return ret;
}

