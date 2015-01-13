// Copyright 2014 Isis Innovation Limited and the authors of InfiniTAM

#pragma once

inline __device__ void warpReduce(volatile float* sdata, int tid) {
	sdata[tid] += sdata[tid + 32];
	sdata[tid] += sdata[tid + 16];
	sdata[tid] += sdata[tid + 8];
	sdata[tid] += sdata[tid + 4];
	sdata[tid] += sdata[tid + 2];
	sdata[tid] += sdata[tid + 1];
}

inline __device__ void warpReduce3(volatile float* sdata, int tid) {
	sdata[3*tid+0] += sdata[3*(tid + 32)+0];
	sdata[3*tid+1] += sdata[3*(tid + 32)+1];
	sdata[3*tid+2] += sdata[3*(tid + 32)+2];
	sdata[3*tid+0] += sdata[3*(tid + 16)+0];
	sdata[3*tid+1] += sdata[3*(tid + 16)+1];
	sdata[3*tid+2] += sdata[3*(tid + 16)+2];
	sdata[3*tid+0] += sdata[3*(tid + 8)+0];
	sdata[3*tid+1] += sdata[3*(tid + 8)+1];
	sdata[3*tid+2] += sdata[3*(tid + 8)+2];
	sdata[3*tid+0] += sdata[3*(tid + 4)+0];
	sdata[3*tid+1] += sdata[3*(tid + 4)+1];
	sdata[3*tid+2] += sdata[3*(tid + 4)+2];
	sdata[3*tid+0] += sdata[3*(tid + 2)+0];
	sdata[3*tid+1] += sdata[3*(tid + 2)+1];
	sdata[3*tid+2] += sdata[3*(tid + 2)+2];
	sdata[3*tid+0] += sdata[3*(tid + 1)+0];
	sdata[3*tid+1] += sdata[3*(tid + 1)+1];
	sdata[3*tid+2] += sdata[3*(tid + 1)+2];
}

template <typename T> 
__device__ int computePrefixSum_device(uint element, T *sum, int localSize, int localId)
{
	// TODO: should be localSize...
	__shared__ uint prefixBuffer[16 * 16];
	__shared__ uint groupOffset;

	prefixBuffer[localId] = element;
	__syncthreads();

	int s1, s2;

	for (s1 = 1, s2 = 1; s1 < localSize; s1 <<= 1)
	{
		s2 |= s1;
		if ((localId & s2) == s2) prefixBuffer[localId] += prefixBuffer[localId - s1];
		__syncthreads();
	}

	for (s1 >>= 2, s2 >>= 1; s1 >= 1; s1 >>= 1, s2 >>= 1)
	{
		if (localId != localSize - 1 && (localId & s2) == s2) prefixBuffer[localId + s1] += prefixBuffer[localId];
		__syncthreads();
	}

	if (localId == 0 && prefixBuffer[localSize - 1] > 0) groupOffset = atomicAdd(sum, prefixBuffer[localSize - 1]);
	__syncthreads();

	int offset;// = groupOffset + prefixBuffer[localId] - 1;
	if (localId == 0) {
		if (prefixBuffer[localId] == 0) offset = -1;
		else offset = groupOffset;
	} else {
		if (prefixBuffer[localId] == prefixBuffer[localId - 1]) offset = -1;
		else offset = groupOffset + prefixBuffer[localId-1];
	}

	return offset;
}

__device__ static inline void atomicMin(float* address, float val)
{
	int* address_as_i = (int*)address;
	int old = *address_as_i, assumed;
	do {
		assumed = old;
		old = ::atomicCAS(address_as_i, assumed,
			__float_as_int(::fminf(val, __int_as_float(assumed))));
	} while (assumed != old);
}

__device__ static inline void atomicMax(float* address, float val)
{
	int* address_as_i = (int*)address;
	int old = *address_as_i, assumed;
	do {
		assumed = old;
		old = ::atomicCAS(address_as_i, assumed,
			__float_as_int(::fmaxf(val, __int_as_float(assumed))));
	} while (assumed != old);
}