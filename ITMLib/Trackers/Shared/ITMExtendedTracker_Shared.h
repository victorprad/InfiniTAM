// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../../Utils/ITMPixelUtils.h"
#include "../../Utils/ITMProjectionUtils.h"

// Tukey loss
_CPU_AND_GPU_CODE_ inline float tukey_rho(float r, float c)
{
	const float c_sq_6 = c * c / 6.f;
	if(fabs(r) <= c)
	{
		float tukey_r = r / c;
		tukey_r *= tukey_r;
		tukey_r = 1 - tukey_r;
		tukey_r = tukey_r * tukey_r * tukey_r;

		return c_sq_6 * (1.f - tukey_r);
	}
	else
	{
		return c_sq_6;
	}
}

_CPU_AND_GPU_CODE_ inline float tukey_rho_deriv(float r, float c)
{
	if(fabs(r) <= c)
	{
		float tukey_r = r / c;
		tukey_r *= tukey_r;
		tukey_r = 1 - tukey_r;
		tukey_r *= tukey_r;

		return r * tukey_r;
	}
	else
	{
		return 0.f;
	}
}

_CPU_AND_GPU_CODE_ inline float tukey_rho_deriv2(float r, float c)
{
	return fabs(r) < c ? 1.0f : 0.0f;
}

// Depth Tracker Norm
_CPU_AND_GPU_CODE_ inline float rho(float r, float huber_b)
{
	float tmp = fabs(r) - huber_b;
	tmp = MAX(tmp, 0.0f);
	return r*r - tmp*tmp;
}

_CPU_AND_GPU_CODE_ inline float rho_deriv(float r, float huber_b)
{
	return 2.0f * CLAMP(r, -huber_b, huber_b);
}

_CPU_AND_GPU_CODE_ inline float rho_deriv2(float r, float huber_b)
{
	return fabs(r) < huber_b ? 2.0f : 0.0f;
}

template<bool shortIteration, bool rotationOnly, bool useWeights>
_CPU_AND_GPU_CODE_ inline bool computePerPointGH_exDepth_Ab(THREADPTR(float) *A, THREADPTR(float) &b,
	const THREADPTR(int) & x, const THREADPTR(int) & y, const CONSTPTR(float) &depth, THREADPTR(float) &depthWeight,
	const CONSTPTR(Vector2i) & viewImageSize, const CONSTPTR(Vector4f) & viewIntrinsics, const CONSTPTR(Vector2i) & sceneImageSize,
	const CONSTPTR(Vector4f) & sceneIntrinsics, const CONSTPTR(Matrix4f) & approxInvPose, const CONSTPTR(Matrix4f) & scenePose, const CONSTPTR(Vector4f) *pointsMap,
	const CONSTPTR(Vector4f) *normalsMap, float spaceThresh, float viewFrustum_min, float viewFrustum_max, float tukeyCutOff, int framesToSkip, int framesToWeight)
{
	depthWeight = 0;

	if (depth <= 1e-8f) return false; //check if valid -- != 0.0f

	Vector4f tmp3Dpoint, tmp3Dpoint_reproj; Vector3f ptDiff;
	Vector4f curr3Dpoint, corr3Dnormal; Vector2f tmp2Dpoint;

	tmp3Dpoint.x = depth * ((float(x) - viewIntrinsics.z) / viewIntrinsics.x);
	tmp3Dpoint.y = depth * ((float(y) - viewIntrinsics.w) / viewIntrinsics.y);
	tmp3Dpoint.z = depth;
	tmp3Dpoint.w = 1.0f;

	// transform to previous frame coordinates
	tmp3Dpoint = approxInvPose * tmp3Dpoint;
	tmp3Dpoint.w = 1.0f;

	// project into previous rendered image
	tmp3Dpoint_reproj = scenePose * tmp3Dpoint;
	if (tmp3Dpoint_reproj.z <= 0.0f) return false;
	tmp2Dpoint.x = sceneIntrinsics.x * tmp3Dpoint_reproj.x / tmp3Dpoint_reproj.z + sceneIntrinsics.z;
	tmp2Dpoint.y = sceneIntrinsics.y * tmp3Dpoint_reproj.y / tmp3Dpoint_reproj.z + sceneIntrinsics.w;

	if (!((tmp2Dpoint.x >= 0.0f) && (tmp2Dpoint.x <= sceneImageSize.x - 2) && (tmp2Dpoint.y >= 0.0f) && (tmp2Dpoint.y <= sceneImageSize.y - 2)))
		return false;

	curr3Dpoint = interpolateBilinear_withHoles(pointsMap, tmp2Dpoint, sceneImageSize);
	if (curr3Dpoint.w < 0.0f) return false;

	ptDiff.x = curr3Dpoint.x - tmp3Dpoint.x;
	ptDiff.y = curr3Dpoint.y - tmp3Dpoint.y;
	ptDiff.z = curr3Dpoint.z - tmp3Dpoint.z;
	float dist = ptDiff.x * ptDiff.x + ptDiff.y * ptDiff.y + ptDiff.z * ptDiff.z;

	if (dist > tukeyCutOff * spaceThresh) return false;

	corr3Dnormal = interpolateBilinear_withHoles(normalsMap, tmp2Dpoint, sceneImageSize);
	//if (corr3Dnormal.w < 0.0f) return false;

	depthWeight = MAX(0.0f, 1.0f - (depth - viewFrustum_min) / (viewFrustum_max - viewFrustum_min));
	depthWeight *= depthWeight;

	if (useWeights)
	{
		if (curr3Dpoint.w < framesToSkip) return false;
		depthWeight *= (curr3Dpoint.w - framesToSkip) / framesToWeight;
	}

	b = corr3Dnormal.x * ptDiff.x + corr3Dnormal.y * ptDiff.y + corr3Dnormal.z * ptDiff.z;

	// TODO check whether normal matches normal from image, done in the original paper, but does not seem to be required
	if (shortIteration)
	{
		if (rotationOnly)
		{
			A[0] = +tmp3Dpoint.z * corr3Dnormal.y - tmp3Dpoint.y * corr3Dnormal.z;
			A[1] = -tmp3Dpoint.z * corr3Dnormal.x + tmp3Dpoint.x * corr3Dnormal.z;
			A[2] = +tmp3Dpoint.y * corr3Dnormal.x - tmp3Dpoint.x * corr3Dnormal.y;
		}
		else { A[0] = corr3Dnormal.x; A[1] = corr3Dnormal.y; A[2] = corr3Dnormal.z; }
	}
	else
	{
		A[0] = +tmp3Dpoint.z * corr3Dnormal.y - tmp3Dpoint.y * corr3Dnormal.z;
		A[1] = -tmp3Dpoint.z * corr3Dnormal.x + tmp3Dpoint.x * corr3Dnormal.z;
		A[2] = +tmp3Dpoint.y * corr3Dnormal.x - tmp3Dpoint.x * corr3Dnormal.y;
		A[!shortIteration ? 3 : 0] = corr3Dnormal.x; A[!shortIteration ? 4 : 1] = corr3Dnormal.y; A[!shortIteration ? 5 : 2] = corr3Dnormal.z;
	}

	return true;
}

template<bool shortIteration, bool rotationOnly>
_CPU_AND_GPU_CODE_ inline bool computePerPointGH_exRGB_inv_Ab(
		THREADPTR(float) &localResidual,
		THREADPTR(float) *localGradient,
		THREADPTR(float) *localHessian,
		int x,
		int y,
		const DEVICEPTR(Vector4f) *points_curr,
		const DEVICEPTR(float) *intensities_curr,
		const DEVICEPTR(float) *intensities_prev,
		const DEVICEPTR(Vector2f) *gradients,
		const CONSTPTR(Vector2i) &imgSize_depth, // and also size of intensities_curr due to the caching
		const CONSTPTR(Vector2i) &imgSize_rgb,
		const Vector4f &intrinsics_depth,
		const Vector4f &intrinsics_rgb,
		const Matrix4f &approxInvPose,
		const Matrix4f &scenePose,
		float colourThresh,
		float minGradient,
		float viewFrustum_min,
		float viewFrustum_max,
		float tukeyCutoff
		)
{
	// Before invoking this method, projectPoint_exRGB is invoked to compute the intensity
	// associated to each depth pixel. Intensities_curr is not the "input" intensity image
	// but the output of such caching. Its size is therefore the same as imgSize_depth
	if (x >= imgSize_depth.x || y >= imgSize_depth.y) return false;

	// Point in current camera coordinates
	const Vector4f pt_curr = points_curr[y * imgSize_depth.x + x];
	// no need to interpolate, has already been done in projectPoint_exRGB
	const float intensity_curr = intensities_curr[y * imgSize_depth.x + x];

	// Invalid point or too far away
	if (pt_curr.w < 0.f || intensity_curr < 0.f || pt_curr.z < 1e-3f || pt_curr.z > viewFrustum_max) return false;

	// Transform the point in world coordinates
	const Vector3f pt_world = approxInvPose * pt_curr.toVector3();

	// Transform the point in previous camera coordinates
	const Vector3f pt_prev = scenePose * pt_world;

	if (pt_prev.z <= 0.0f) return false; // Point behind the camera

	// Project the point in the previous intensity frame
	const Vector2f pt_prev_proj = project(pt_prev, intrinsics_rgb);

	// Outside the image plane
	if (pt_prev_proj.x < 0 || pt_prev_proj.x >= imgSize_rgb.x - 1 ||
		pt_prev_proj.y < 0 || pt_prev_proj.y >= imgSize_rgb.y - 1) return false;

	// Point should be valid, sample intensities and gradients
	const float intensity_prev = interpolateBilinear_single(intensities_prev, pt_prev_proj, imgSize_rgb);
	const Vector2f gradient_prev = interpolateBilinear_Vector2(gradients, pt_prev_proj, imgSize_rgb);

	const float intensity_diff = intensity_prev - intensity_curr;

	if (fabs(intensity_diff) >= tukeyCutoff * colourThresh) return false; // Difference too big
	if (fabs(gradient_prev.x) < minGradient || fabs(gradient_prev.y) < minGradient) return false; // Gradient too small

	// Cache rows of the scenePose rotation matrix, to be used in the pose derivative
	const Vector3f scene_rot_row_0 = scenePose.getRow(0).toVector3();
	const Vector3f scene_rot_row_1 = scenePose.getRow(1).toVector3();
	const Vector3f scene_rot_row_2 = scenePose.getRow(2).toVector3();

	// Precompute projection derivatives
	const float pt_prev_inv_z = 1.f / pt_prev.z;
	const float pt_prev_inv_z_sq = pt_prev_inv_z * pt_prev_inv_z;

	const Vector3f d_proj_x(intrinsics_rgb.x * pt_prev_inv_z,
							0.f,
							-intrinsics_rgb.x * pt_prev.x * pt_prev_inv_z_sq);
	const Vector3f d_proj_y(0.f,
							intrinsics_rgb.y * pt_prev_inv_z,
							-intrinsics_rgb.y * pt_prev.y  * pt_prev_inv_z_sq);

	float nabla[6];

	const int numPara = shortIteration ? 3 : 6;
	for (int para = 0; para < numPara; para++)
	{
		// Derivatives of approxInvPose wrt. the current parameter
		Vector3f d_point_col;
		switch (para + (shortIteration && !rotationOnly ? 3 : 0))
		{
		case 0: //rx
			d_point_col = Vector3f(0, -pt_world.z, pt_world.y);
			break;
		case 1: // ry
			d_point_col = Vector3f(pt_world.z, 0, -pt_world.x);
			break;
		case 2: // rz
			d_point_col = Vector3f(-pt_world.y, pt_world.x, 0);
			break; //rz
		case 3: //tx
			// Identity matrix
			// We negate it because the ApplyDelta function uses the KinectFusion
			// skew symmetric matrix and that matrix has negated rotation components.
			// In order to use the rgb tracker we would need to negate the entire computed step, but given
			// the peculiar structure of the increment matrix we only need to negate the translation component.
			d_point_col = -Vector3f(1,0,0);
			break;
		case 4: //ty
			d_point_col = -Vector3f(0,1,0);
			break;
		case 5: //tz
			d_point_col = -Vector3f(0,0,1);
			break;
		default:
			d_point_col = Vector3f(0,0,0); // Should never happen
			break;
		};

		// Chain the above with scenePose
		const Vector3f d_point_col_rot(dot(scene_rot_row_0, d_point_col),
									   dot(scene_rot_row_1, d_point_col),
									   dot(scene_rot_row_2, d_point_col));

		// Chain with the projection
		const Vector2f d_proj_point(dot(d_proj_x, d_point_col_rot),
									dot(d_proj_y, d_point_col_rot));

		// Chain with the intensity gradient
		nabla[para] = dot(gradient_prev, d_proj_point);
	}

	// Weigh less the points far away from the camera
	float depthWeight = 1.0f - (pt_curr.z - viewFrustum_min) / (viewFrustum_max - viewFrustum_min);
	depthWeight = MAX(depthWeight, 0.f);
	depthWeight *= depthWeight;

	// Compute the residual
	localResidual = depthWeight * tukey_rho(intensity_diff, colourThresh);

	// Precompute tukey weights
	const float tukey_coeff_gradient = depthWeight * tukey_rho_deriv(intensity_diff, colourThresh);
	const float tukey_coeff_hessian = depthWeight * tukey_rho_deriv2(intensity_diff, colourThresh);

	// Fill gradient and hessian
	for (int para = 0, counter = 0; para < numPara; para++)
	{
		localGradient[para] = tukey_coeff_gradient * nabla[para];

		for (int col = 0; col <= para; col++)
		{
			localHessian[counter++] = tukey_coeff_hessian * nabla[para] * nabla[col];
		}
	}

	return true;
}

template<bool shortIteration, bool rotationOnly, bool useWeights>
_CPU_AND_GPU_CODE_ inline bool computePerPointGH_exDepth(THREADPTR(float) *localNabla, THREADPTR(float) *localHessian, THREADPTR(float) &localF,
	const THREADPTR(int) & x, const THREADPTR(int) & y, const CONSTPTR(float) &depth, THREADPTR(float) &depthWeight, CONSTPTR(Vector2i) & viewImageSize, const CONSTPTR(Vector4f) & viewIntrinsics,
	const CONSTPTR(Vector2i) & sceneImageSize, const CONSTPTR(Vector4f) & sceneIntrinsics, const CONSTPTR(Matrix4f) & approxInvPose, const CONSTPTR(Matrix4f) & scenePose,
	const CONSTPTR(Vector4f) *pointsMap, const CONSTPTR(Vector4f) *normalsMap, float spaceThresh, float viewFrustum_min, float viewFrustum_max, float tukeyCutOff, int framesToSkip, int framesToWeight)
{
	const int noPara = shortIteration ? 3 : 6;
	float A[noPara];
	float b;

	bool ret = computePerPointGH_exDepth_Ab<shortIteration, rotationOnly, useWeights>(A, b, x, y, depth, depthWeight, viewImageSize, viewIntrinsics, sceneImageSize, sceneIntrinsics,
		approxInvPose, scenePose, pointsMap, normalsMap, spaceThresh, viewFrustum_min, viewFrustum_max, tukeyCutOff, framesToSkip, framesToWeight);

	if (!ret) return false;

	localF = rho(b, spaceThresh) * depthWeight;

#if (defined(__CUDACC__) && defined(__CUDA_ARCH__)) || (defined(__METALC__))
#pragma unroll
#endif
	for (int r = 0, counter = 0; r < noPara; r++)
	{
		localNabla[r] = rho_deriv(b, spaceThresh) * depthWeight * A[r];
#if (defined(__CUDACC__) && defined(__CUDA_ARCH__)) || (defined(__METALC__))
#pragma unroll
#endif
		for (int c = 0; c <= r; c++, counter++) localHessian[counter] = rho_deriv2(b, spaceThresh) * depthWeight * A[r] * A[c];
	}

	return true;
}

_CPU_AND_GPU_CODE_ inline bool computePerPointProjectedColour_exRGB(
		THREADPTR(int) x,
		THREADPTR(int) y,
		DEVICEPTR(Vector4f) *out_points,
		DEVICEPTR(float) *out_rgb,
		const DEVICEPTR(float) *in_rgb,
		const DEVICEPTR(float) *in_depths,
		const CONSTPTR(Vector2i) &imageSize,
		const CONSTPTR(int) sceneIdx,
		const CONSTPTR(Vector4f) &intrinsics_rgb,
		const CONSTPTR(Vector4f) &intrinsics_depth,
		const CONSTPTR(Matrix4f) &scenePose)
{
	float depth_camera = in_depths[sceneIdx];

	// Invalid point
	if (depth_camera <= 0.f) return false;

	const Vector3f pt_camera = unproject(x, y, depth_camera, intrinsics_depth);

	// Transform the point in the RGB sensor frame
	const Vector3f pt_image = scenePose * pt_camera;

	// Point behind the camera
	if(pt_image.z <= 0.f) return false;

	// Project the point onto the previous frame
	const Vector2f pt_image_proj = project(pt_image, intrinsics_rgb);

	// Projection outside the previous rgb frame
	if (pt_image_proj.x < 0 || pt_image_proj.x >= imageSize.x - 1 ||
		pt_image_proj.y < 0 || pt_image_proj.y >= imageSize.y - 1) return false;

	out_rgb[sceneIdx] = interpolateBilinear_single(in_rgb, pt_image_proj, imageSize);
	out_points[sceneIdx] = Vector4f(pt_camera, 1.f);
	return true;
}

_CPU_AND_GPU_CODE_ inline void projectPoint_exRGB(
		THREADPTR(int) x,
		THREADPTR(int) y,
		DEVICEPTR(Vector4f) *out_points,
		DEVICEPTR(float) *out_rgb,
		const DEVICEPTR(float) *in_rgb,
		const DEVICEPTR(float) *in_depths,
		const CONSTPTR(Vector2i) &imageSize_rgb,
		const CONSTPTR(Vector2i) &imageSize_depth,
		const CONSTPTR(Vector4f) &intrinsics_rgb,
		const CONSTPTR(Vector4f) &intrinsics_depth,
		const CONSTPTR(Matrix4f) &scenePose)
{
	if (x >= imageSize_depth.x || y >= imageSize_depth.y) return;

	int sceneIdx = y * imageSize_depth.x + x;

	if (!computePerPointProjectedColour_exRGB(x, y, out_points, out_rgb, in_rgb, in_depths,
		 imageSize_rgb, sceneIdx, intrinsics_rgb, intrinsics_depth, scenePose))
	{
		out_rgb[sceneIdx] = -1.f; // Mark as invalid
		out_points[sceneIdx] = Vector4f(0.f, 0.f, 0.f, -1.f); // Mark as invalid
	}
}
