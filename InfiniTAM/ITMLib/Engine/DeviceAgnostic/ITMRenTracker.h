// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../../Utils/ITMLibDefines.h"
#include "ITMPixelUtils.h"
#include "ITMSceneReconstructionEngine.h"

// sigma that controls the basin of attraction
#define DTUNE 6.0f

_CPU_AND_GPU_CODE_ inline void unprojectPtWithIntrinsic(const THREADPTR(Vector4f) &intrinsic, const THREADPTR(Vector3f) &inpt, THREADPTR(Vector4f) &outpt)
{
	outpt.x = intrinsic.x * inpt.x + intrinsic.z * inpt.z;
	outpt.y = intrinsic.y * inpt.y + intrinsic.w * inpt.z;
	outpt.z = inpt.z;
	outpt.w = 1.0f;
}

template<class TVoxel, class TIndex>
_CPU_AND_GPU_CODE_ inline float computePerPixelEnergy(const THREADPTR(Vector4f) &inpt, const CONSTPTR(TVoxel) *voxelBlocks,
	const CONSTPTR(typename TIndex::IndexData) *index, float oneOverVoxelSize, Matrix4f invM)
{
	Vector3f pt; bool dtIsFound;
	pt = TO_VECTOR3(invM * inpt) * oneOverVoxelSize;

	// faster but theoretically worse
	float dt = readFromSDF_float_uninterpolated(voxelBlocks, index, pt, dtIsFound);

	//typename TIndex::IndexCache cache;
	//float dt = readFromSDF_float_interpolated(voxelBlocks, index, pt, dtIsFound, cache);

	if (dt == 1.0f) return 0.0f;

	float expdt = exp(-dt * DTUNE);
	return 4.0f * expdt / ((expdt + 1.0f)*(expdt + 1.0f));
}

template<class TVoxel, class TIndex>
_CPU_AND_GPU_CODE_ inline Vector3f computeDDT(const CONSTPTR(Vector3f) &pt_f, const THREADPTR(TVoxel) *voxelBlocks,
	const THREADPTR(typename TIndex::IndexData) *index, float oneOverVoxelSize, DEVICEPTR(bool) &ddtFound)
{
	
	Vector3f ddt;

	Vector3i pt = TO_INT_ROUND3(pt_f);

	bool isFound; float dt1, dt2;

	dt1 = TVoxel::SDF_valueToFloat(readVoxel(voxelBlocks, index, pt + Vector3i(1, 0, 0), isFound).sdf);
	if (!isFound || dt1 == 1.0f) { ddtFound = false; return Vector3f(0.0f); }
	dt2 = TVoxel::SDF_valueToFloat(readVoxel(voxelBlocks, index, pt + Vector3i(-1, 0, 0), isFound).sdf);
	if (!isFound || dt2 == 1.0f) { ddtFound = false; return Vector3f(0.0f); }
	ddt.x = (dt1 - dt2) * 0.5f;

	dt1 = TVoxel::SDF_valueToFloat(readVoxel(voxelBlocks, index, pt + Vector3i(0, 1, 0), isFound).sdf);
	if (!isFound || dt1 == 1.0f) { ddtFound = false; return Vector3f(0.0f); }
	dt2 = TVoxel::SDF_valueToFloat(readVoxel(voxelBlocks, index, pt + Vector3i(0, -1, 0), isFound).sdf);
	if (!isFound || dt2 == 1.0f) { ddtFound = false; return Vector3f(0.0f); }
	ddt.y = (dt1 - dt2) * 0.5f;

	dt1 = TVoxel::SDF_valueToFloat(readVoxel(voxelBlocks, index, pt + Vector3i(0, 0, 1), isFound).sdf);
	if (!isFound || dt1 == 1.0f) { ddtFound = false; return Vector3f(0.0f); }
	dt2 = TVoxel::SDF_valueToFloat(readVoxel(voxelBlocks, index, pt + Vector3i(0, 0, -1), isFound).sdf);
	if (!isFound || dt2 == 1.0f) { ddtFound = false; return Vector3f(0.0f); }
	ddt.z = (dt1 - dt2) * 0.5f;

	ddtFound = true; return ddt;
}

template<class TVoxel, class TIndex>
_CPU_AND_GPU_CODE_ inline bool computePerPixelJacobian(THREADPTR(float) *jacobian, const THREADPTR(Vector4f) &inpt, 
	const CONSTPTR(TVoxel) *voxelBlocks, const CONSTPTR(typename TIndex::IndexData) *index, float oneOverVoxelSize, Matrix4f invM)
{

	bool isFound;

	Vector3f cPt, dDt, pt;

	cPt = TO_VECTOR3(invM * inpt);

	pt = cPt * oneOverVoxelSize;

	//typename TIndex::IndexCache cache;
	//float dt = readFromSDF_float_interpolated(voxelBlocks, index, pt, isFound, cache);

	float dt = readFromSDF_float_uninterpolated(voxelBlocks, index, pt, isFound);

	if (dt == 1.0f || !isFound) return false;


	dDt = computeDDT<TVoxel, TIndex>(pt, voxelBlocks, index, oneOverVoxelSize, isFound);
	if (!isFound) return false;

	float expdt = exp(-dt * DTUNE);
	float deto = expdt + 1;

	float prefix = 4.0f * DTUNE * (2.0f * exp(-dt * 2.0f * DTUNE) / (deto * deto * deto) - expdt / (deto * deto));

	dDt *= prefix;

	jacobian[0] = dDt.x; jacobian[1] = dDt.y; jacobian[2] = dDt.z;

	jacobian[3] = 4.0f * (dDt.z * cPt.y - dDt.y * cPt.z);
	jacobian[4] = 4.0f * (dDt.x * cPt.z - dDt.z * cPt.x);
	jacobian[5] = 4.0f * (dDt.y * cPt.x - dDt.x * cPt.y);

	return true;
}
