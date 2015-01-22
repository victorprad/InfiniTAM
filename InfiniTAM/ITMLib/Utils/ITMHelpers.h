// Copyright 2014 Isis Innovation Limited and the authors of InfiniTAM

#pragma once

#if (defined __CUDACC__)&&(defined __CUDA_ARCH__)

// on the GPU, we just use the cuda builtin default functions

_CPU_AND_GPU_CODE_ inline int myAtomicCAS(int *addr, int oldValue, int newValue)
{ return atomicCAS(addr, oldValue, newValue); }

_CPU_AND_GPU_CODE_ inline int myAtomicSub(int *addr, int v = 1)
{ return atomicSub(addr, v); }

_CPU_AND_GPU_CODE_ inline int myAtomicAdd(int *addr, int v = 1)
{ return atomicAdd(addr, v); }

#else

// this is only ever used on the host - they are not actually atomic!

inline int myAtomicCAS(int *addr, int oldValue, int newValue)
{
	int a = *addr;
	if (a == oldValue) *addr = newValue;
	return a;
}

inline int myAtomicSub(int *addr, int v = 1)
{
	int a = *addr;
	*addr = a - v;
	return a;
}

inline int myAtomicAdd(int *addr, int v = 1)
{
	int a = *addr;
	*addr = a + v;
	return a;
}

#endif
