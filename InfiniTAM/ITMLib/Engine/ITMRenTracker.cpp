#include "ITMRenTracker.h"
#include "../Utils/ITMCholesky.h"

#include <math.h>
#include <stdio.h>

using namespace ITMLib::Engine;
using namespace ITMLib::Utils;

template<class TVoxel, class TIndex>
static inline bool minimizeLM(const ITMRenTracker<TVoxel,TIndex> & tracker, ITMPose & initialization);

static inline void GetRotationMatrixFromMRP(float *outR, const float* r)
{
	float t1 = r[0], t2 = r[1], t3 = r[2];

	float tsq = t1*t1 + t2*t2 + t3*t3;

	float tsum = 1 - tsq;

	outR[0] = 4 * t1*t1 - 4 * t2*t2 - 4 * t3*t3 + tsum*tsum;	outR[1] = 8 * t1*t2 - 4 * t3*tsum;	outR[2] = 8 * t1*t3 + 4 * t2*tsum;
	outR[3] = 8 * t1*t2 + 4 * t3*tsum;	outR[4] = 4 * t2*t2 - 4 * t1*t1 - 4 * t3*t3 + tsum*tsum;	outR[5] = 8 * t2*t3 - 4 * t1*tsum;
	outR[6] = 8 * t1*t3 - 4 * t2*tsum;	outR[7] = 8 * t2*t3 + 4 * t1*tsum;	outR[8] = 4 * t3*t3 - 4 * t2*t2 - 4 * t1*t1 + tsum*tsum;

	for (int i = 0; i<9; i++) outR[i] /= ((1 + tsq)*(1 + tsq));
}

// // used by carl LM
static inline void GetMFromParam(float* pose, Matrix4f& M)
{
	float R[9];

	GetRotationMatrixFromMRP(R, &(pose[3]));

	M.m00 = R[0]; M.m01 = R[3]; M.m02 = R[6]; M.m03 = 0;
	M.m10 = R[1]; M.m11 = R[4]; M.m12 = R[7]; M.m13 = 0;
	M.m20 = R[2]; M.m21 = R[5]; M.m22 = R[8]; M.m23 = 0;
	M.m30 = pose[0]; M.m31 = pose[1]; M.m32 = pose[2]; M.m33 = 1;
}


void ComputeSingleStep(float *step, float *ATA, float *ATb, float lambda)
{
	float tmpATA[6 * 6];
	memcpy(tmpATA, ATA, 6 * 6 * sizeof(float));
	memset(step, 0, 6 * sizeof(float));

	for (int i = 0; i < 6 * 6; i += 7) tmpATA[i] += lambda * ATA[i];
	for (int i = 0; i < 6; i++) step[i] = 0;

	ITMCholesky cholA(tmpATA, 6);
	cholA.Backsub(step, ATb);

	for (int i = 0; i < 6; i++) step[i] = -step[i];
}


template<class TVoxel, class TIndex>
ITMRenTracker<TVoxel, TIndex>::ITMRenTracker(Vector2i imgSize, int noHierarchyLevels, ITMLowLevelEngine *lowLevelEngine, ITMScene<TVoxel,TIndex> *scene, bool useGPU)
{ 
	//TODO from parameters, rotationOnly not implemented

	viewHierarchy = new ITMImageHierarchy<ITMTemplatedHierarchyLevel<ITMFloat4Image> >(imgSize, noHierarchyLevels, 0, useGPU);

	tempImage1 = new ITMFloatImage(imgSize, useGPU);
	tempImage2 = new ITMFloatImage(imgSize, useGPU);

	this->lowLevelEngine = lowLevelEngine;
	this->scene = scene;
};

template<class TVoxel, class TIndex>
ITMRenTracker<TVoxel,TIndex>::~ITMRenTracker(void)
{
	delete this->viewHierarchy;

	delete tempImage1;
	delete tempImage2;
};

template<class TVoxel, class TIndex>
void ITMRenTracker<TVoxel,TIndex>::PrepareForEvaluation(const ITMView *view)
{
	Vector4f intrinsics = view->calib->intrinsics_d.projectionParamsSimple.all;

	this->tempImage1->noDims = view->depth->noDims;
	this->tempImage1->dataSize = view->depth->dataSize;
	lowLevelEngine->CopyImage(this->tempImage1, view->depth);

	viewHierarchy->levels[0]->intrinsics = intrinsics;
	UnprojectDepthToCam(view->depth, viewHierarchy->levels[0]->depth, intrinsics);

	for (int i = 1; i < viewHierarchy->noLevels; i++)
	{
		ITMTemplatedHierarchyLevel<ITMFloat4Image> *currentLevelView = viewHierarchy->levels[i], *previousLevelView = viewHierarchy->levels[i - 1];
		
		lowLevelEngine->FilterSubsampleWithHoles(tempImage2, tempImage1);

		currentLevelView->intrinsics = previousLevelView->intrinsics * 0.5f;
		UnprojectDepthToCam(tempImage2, viewHierarchy->levels[i]->depth, viewHierarchy->levels[i]->intrinsics);

		this->tempImage1->noDims = this->tempImage2->noDims;
		this->tempImage1->dataSize = this->tempImage2->dataSize;
		lowLevelEngine->CopyImage(this->tempImage1, this->tempImage2);
	}
}

template<class TVoxel, class TIndex>
void ITMRenTracker<TVoxel,TIndex>::TrackCamera(ITMTrackingState *trackingState, const ITMView *view)
{
	this->PrepareForEvaluation(view);

	// // Olaf LM

	//ITMPose currentPara(*trackingState->pose_d);
	//for (int levelId = viewHierarchy->noLevels - 1; levelId >= 2; levelId--) //skips full resolution
	//{
	//	this->levelId = levelId;
	//	this->rotationOnly = rotationOnly; //ignored 

	//	minimizeLM(*this, currentPara);
	//}

	//Matrix4f M; currentPara.invM.inv(M);
	//trackingState->pose_d->SetFrom(M);


	// // Carl lm

	//float lastEnergy = 0.0f, currentEnergy = 0.0f, lambda = 1000.0f;
	//float step[6]; Matrix4f invM, tmpM;

	//bool converged = false;
	//int iter;

	//this->levelId = 0;

	//invM = trackingState->pose_d->invM;

	//F_oneLevel(&lastEnergy, invM);

	//for (iter = 0; iter < 200; iter++)
	//{
	//	G_oneLevel(ATb_host, ATA_host, invM);

	//	while (true)
	//	{
	//		if (lambda>1e6) { converged = true; break; }

	//		ComputeSingleStep(step, ATA_host, ATb_host, lambda);
	//		GetMFromParam(step, tmpM);

	//		F_oneLevel(&currentEnergy, tmpM * invM);

	//		if (currentEnergy < lastEnergy)
	//		{
	//			lastEnergy = currentEnergy;
	//			lambda /= 100.0f;
	//			invM = tmpM * invM;

	//			break;
	//		}
	//		else lambda *= 10.0f;
	//	}

	//	if (converged) break;
	//}

	//invM.inv(tmpM); trackingState->pose_d->SetFrom(tmpM);


	//// Carl Gaussian Newton

	float step[6]; Matrix4f invM, tmpM;
	invM = trackingState->pose_d->invM;

	for (int mlevelId = viewHierarchy->noLevels - 1; mlevelId >= 0; mlevelId--)
	{
		this->levelId = mlevelId;
		
		for (int iterNo = 0; iterNo < 10; iterNo++)
		{
			float normal = 0.0f;
			G_oneLevel(ATb_host, ATA_host, invM);
			ComputeSingleStep(step, ATA_host, ATb_host, 0.0f);

			for (int i = 0; i < 6; i++)
			{
				step[i] *= 0.0001f;
				normal += step[i] * step[i];
			}
		
			if (normal < 1e-9f) {break; }

			GetMFromParam(step, tmpM);
			invM = tmpM * invM;
		}
	}

	invM.inv(tmpM); trackingState->pose_d->SetFrom(tmpM);
}

template<class TVoxel, class TIndex>
void ITMRenTracker<TVoxel,TIndex>::EvaluationPoint::computeGradients(bool hessianRequired)
{
	int numPara = mParent->numParameters();
	cacheNabla = new float[numPara];
	cacheHessian = new float[numPara*numPara];

	mParent->G_oneLevel(cacheNabla, cacheHessian, mPara->invM);
}

template<class TVoxel, class TIndex>
ITMRenTracker<TVoxel,TIndex>::EvaluationPoint::EvaluationPoint(ITMPose *pos, const ITMRenTracker *f_parent)
{
	this->mPara = pos; this->mParent = f_parent;
	ITMRenTracker *parent = (ITMRenTracker *)mParent;
	parent->F_oneLevel(&cacheF, mPara->invM);

	cacheHessian = NULL; cacheNabla = NULL;
}

template<class TVoxel, class TIndex>
void ITMRenTracker<TVoxel,TIndex>::applyDelta(const ITMPose & para_old, const float *delta, ITMPose & para_new) const
{
	float R[9];

	Matrix4f deltaM;

	GetRotationMatrixFromMRP(R, &(delta[3]));

	deltaM.m00 = R[0];		deltaM.m01 = R[3];		deltaM.m02 = R[6];		deltaM.m03 = 0.0f;
	deltaM.m10 = R[1];		deltaM.m11 = R[4];		deltaM.m12 = R[7];		deltaM.m13 = 0.0f;
	deltaM.m20 = R[2];		deltaM.m21 = R[5];		deltaM.m22 = R[8];		deltaM.m23 = 0.0f;
	deltaM.m30 = delta[0];	deltaM.m31 = delta[1];	deltaM.m32 = delta[2];	deltaM.m33 = 1.0f;

	para_new.invM = deltaM * para_old.invM;
}

// LM optimisation -- same as ITMColorTracker

template<class TVoxel, class TIndex>
static inline double stepQuality(typename ITMRenTracker<TVoxel,TIndex>::EvaluationPoint *x, typename ITMRenTracker<TVoxel,TIndex>::EvaluationPoint *x2, const float *step, const float *grad, const float *B, int numPara)
{
	double actual_reduction = x->f() - x2->f();
	double predicted_reduction = 0.0;
	float *tmp = new float[numPara];

	matmul(B, step, tmp, numPara, numPara);
	for (int i = 0; i < numPara; i++) predicted_reduction -= grad[i] * step[i] + 0.5*step[i] * tmp[i];
	delete[] tmp;

	if (predicted_reduction < 0) return actual_reduction / fabs(predicted_reduction);
	return actual_reduction / predicted_reduction;
}

template<class TVoxel, class TIndex>
static inline bool minimizeLM(const ITMRenTracker<TVoxel,TIndex> & tracker, ITMPose & initialization)
{
	// These are some sensible default parameters for Levenberg Marquardt.
	// The first three control the convergence criteria, the others might
	// impact convergence speed.
	static const int MAX_STEPS = 100;
	static const float MIN_STEP = 0.00005f;
	static const float MIN_DECREASE = 0.00001f;
	static const float TR_QUALITY_GAMMA1 = 0.75f;
	static const float TR_QUALITY_GAMMA2 = 0.25f;
	static const float TR_REGION_INCREASE = 2.0f;
	static const float TR_REGION_DECREASE = 0.3f;

	int numPara = tracker.numParameters();
	float *d = new float[numPara];
	float lambda = 1000.0f;
	int step_counter = 0;

	typename ITMRenTracker<TVoxel,TIndex>::EvaluationPoint *x = tracker.evaluateAt(new ITMPose(initialization));
	typename ITMRenTracker<TVoxel,TIndex>::EvaluationPoint *x2 = NULL;

	if (!portable_finite(x->f())) { delete[] d; delete x; return false; }

	do
	{
		const float *grad;
		const float *B;

		grad = x->nabla_f();
		B = x->hessian_GN();

		bool success;
		{
			float *A = new float[numPara*numPara];
			for (int i = 0; i < numPara*numPara; ++i) A[i] = B[i];
			for (int i = 0; i < numPara; ++i)
			{
				float & ele = A[i*(numPara + 1)];
				if (!(fabs(ele) < 1e-15f)) ele *= (1.0f + lambda); else ele = lambda*1e-10f;
			}

			ITMLib::Utils::ITMCholesky cholA(A, numPara);

			cholA.Backsub(&(d[0]), grad);
			// TODO: if Cholesky failed, set success to false!

			success = true;
			delete[] A;
		}

		if (success)
		{
			float MAXnorm = 0.0;
			for (int i = 0; i<numPara; i++) { float tmp = fabs(d[i]); if (tmp>MAXnorm) MAXnorm = tmp; }

			if (MAXnorm < MIN_STEP) break;
			for (int i = 0; i < numPara; i++) d[i] = -d[i];

			// make step
			ITMPose *tmp_para = new ITMPose(x->getParameter());
			tracker.applyDelta(x->getParameter(), &(d[0]), *tmp_para);

			// check whether step reduces error function and
			// compute a new value of lambda
			x2 = tracker.evaluateAt(tmp_para);

			double rho = stepQuality<TVoxel,TIndex>(x, x2, &(d[0]), grad, B, numPara);
			if (rho > TR_QUALITY_GAMMA1)
				lambda = lambda / TR_REGION_INCREASE;
			else if (rho <= TR_QUALITY_GAMMA2)
			{
				success = false;
				lambda = lambda / TR_REGION_DECREASE;
			}
		}
		else
		{
			x2 = NULL;
			// can't compute a step quality here...
			lambda = lambda / TR_REGION_DECREASE;
		}

		if (success)
		{
			// accept step
			bool continueIteration = true;
			if (!(x2->f() < (x->f() - fabs(x->f()) * MIN_DECREASE))) continueIteration = false;


			delete x;
			x = x2;

			if (!continueIteration) break;
		}
		else if (x2 != NULL) delete x2;
		if (step_counter++ >= MAX_STEPS - 1) break;
	} while (true);

	initialization.SetFrom(&(x->getParameter()));
	delete x;

	delete[] d;

	return true;
}

template class ITMLib::Engine::ITMRenTracker<ITMVoxel, ITMVoxelIndex>;

