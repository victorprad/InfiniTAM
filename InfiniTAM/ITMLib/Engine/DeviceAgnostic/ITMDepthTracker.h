// Copyright 2014 Isis Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../../Utils/ITMLibDefines.h"
#include "../../Utils/ITMPixelUtils.h"

_CPU_AND_GPU_CODE_ inline bool computePerPointGH_Depth(float *localNabla, float *localHessian, int x, int y, float *depth, Vector2i viewImageSize, Vector4f viewIntrinsics,
	Vector2i sceneImageSize, Vector4f sceneIntrinsics, Matrix4f approxInvPose, Matrix4f scenePose, Vector4f *pointsMap, Vector4f *normalsMap, float distThresh, 
	bool rotationOnly, int noPara)
{
	float tmpD = depth[x + y * viewImageSize.x];

	if (tmpD <= 1e-8f) return false; //check if valid -- != 0.0f

	Vector4f tmp3Dpoint, tmp3Dpoint_reproj, curr3Dpoint, corr3Dnormal, ptDiff; Vector2f tmp2Dpoint;
	float A[6];

	tmp3Dpoint.x = tmpD * ((float(x) - viewIntrinsics.z) / viewIntrinsics.x);
	tmp3Dpoint.y = tmpD * ((float(y) - viewIntrinsics.w) / viewIntrinsics.y);
	tmp3Dpoint.z = tmpD; tmp3Dpoint.w = 1.0f;

	// transform to previous frame coordinates
	tmp3Dpoint = approxInvPose * tmp3Dpoint;

	// project into previous rendered image
	tmp3Dpoint_reproj = scenePose * tmp3Dpoint;
	if (tmp3Dpoint_reproj.z <= 0.0f) return false;
	tmp2Dpoint.x = sceneIntrinsics.x * tmp3Dpoint_reproj.x / tmp3Dpoint_reproj.z + sceneIntrinsics.z;
	tmp2Dpoint.y = sceneIntrinsics.y * tmp3Dpoint_reproj.y / tmp3Dpoint_reproj.z + sceneIntrinsics.w;

	if (!((tmp2Dpoint.x >= 0.0f) && (tmp2Dpoint.x <= sceneImageSize.x - 2) && (tmp2Dpoint.y >= 0.0f) && (tmp2Dpoint.y <= sceneImageSize.y - 2)))
		return false;

	curr3Dpoint = interpolateBilinear_withHoles(pointsMap, tmp2Dpoint, sceneImageSize);
	if (curr3Dpoint.w < 0.0f) return false;

	ptDiff = curr3Dpoint - tmp3Dpoint;
	float dist = ptDiff.x * ptDiff.x + ptDiff.y * ptDiff.y + ptDiff.z * ptDiff.z;

	if (dist > distThresh) return false;

	corr3Dnormal = interpolateBilinear_withHoles(normalsMap, tmp2Dpoint, sceneImageSize);
	if (corr3Dnormal.w < 0.0f) return false;

	// TODO check whether normal matches normal from image, done in the original paper, but does not seem to be required

	A[0] = +tmp3Dpoint.z * corr3Dnormal.y - tmp3Dpoint.y * corr3Dnormal.z;
	A[1] = -tmp3Dpoint.z * corr3Dnormal.x + tmp3Dpoint.x * corr3Dnormal.z;
	A[2] = +tmp3Dpoint.y * corr3Dnormal.x - tmp3Dpoint.x * corr3Dnormal.y;
	if (!rotationOnly) { A[3] = corr3Dnormal.x; A[4] = corr3Dnormal.y; A[5] = corr3Dnormal.z; }

	float b = corr3Dnormal.x * ptDiff.x + corr3Dnormal.y * ptDiff.y + corr3Dnormal.z * ptDiff.z;

	for (int r = 0, counter = 0; r < noPara; r++)
	{
		localNabla[r] = b * A[r];
		for (int c = 0; c <= r; c++, counter++) localHessian[counter] = A[r] * A[c];
	}

	return true;
}
