// Copyright 2014-2016 Isis Innovation Limited and the authors of InfiniTAM
// Uses code from MEstimator.h in PTAM

#pragma once

#include "ITMMath.h"

struct Tukey
{
    inline static float SquareRootWeight(float dErrorSquared, float dSigmaSquared);
    inline static float Weight(float dErrorSquared, float dSigmaSquared);
    inline static float ObjectiveScore(float dErrorSquared, float dSigmaSquared);
};

struct Cauchy
{
    inline static float SquareRootWeight(float dErrorSquared, float dSigmaSquared);
    inline static float Weight(float dErrorSquared, float dSigmaSquared);
    inline static float ObjectiveScore(float dErrorSquared, float dSigmaSquared);
};

struct Huber
{
    inline static float SquareRootWeight(float dErrorSquared, float dSigmaSquared);
    inline static float Weight(float dErrorSquared, float dSigmaSquared);
    inline static float ObjectiveScore(float dErrorSquared, float dSigmaSquared);
};

struct LeastSquares
{
    inline static float SquareRootWeight(float dErrorSquared, float dSigmaSquared);
    inline static float Weight(float dErrorSquared, float dSigmaSquared);
    inline static float ObjectiveScore(float dErrorSquared, float dSigmaSquared);
};


_CPU_AND_GPU_CODE_ inline float Tukey::Weight(float dErrorSquared, float dSigmaSquared)
{
    float dSqrt = SquareRootWeight(dErrorSquared, dSigmaSquared);
    return dSqrt * dSqrt;
}

_CPU_AND_GPU_CODE_ inline float Tukey::SquareRootWeight(float dErrorSquared, float dSigmaSquared)
{
    if(dErrorSquared > dSigmaSquared) return 0.0f;
    else return 1.0f - (dErrorSquared / dSigmaSquared);
}

_CPU_AND_GPU_CODE_ inline float Tukey::ObjectiveScore(float dErrorSquared, const float dSigmaSquared)
{
    // NB All returned are scaled because
    // I'm not multiplying by sigmasquared/6.0
    if(dErrorSquared > dSigmaSquared) return 1.0f;
    float d = 1.0f - dErrorSquared / dSigmaSquared;
    return (1.0f - d*d*d);
}

_CPU_AND_GPU_CODE_ inline float Cauchy::Weight(float dErrorSquared, float dSigmaSquared)
{
    return 1.0f / (1.0f + dErrorSquared / dSigmaSquared);
}

_CPU_AND_GPU_CODE_ inline float Cauchy::SquareRootWeight(float dErrorSquared, float dSigmaSquared)
{
    return sqrt(Weight(dErrorSquared, dSigmaSquared));
}

_CPU_AND_GPU_CODE_ inline float Cauchy::ObjectiveScore(float dErrorSquared, const float dSigmaSquared)
{
    return log(1.0f + dErrorSquared / dSigmaSquared);
}

_CPU_AND_GPU_CODE_ inline float Huber::Weight(float dErrorSquared, float dSigmaSquared)
{
    if(dErrorSquared < dSigmaSquared) return 1;
    else return sqrt(dSigmaSquared / dErrorSquared);
}

_CPU_AND_GPU_CODE_ inline float Huber::SquareRootWeight(float dErrorSquared, float dSigmaSquared)
{
    return sqrt(Weight(dErrorSquared, dSigmaSquared));
}

_CPU_AND_GPU_CODE_ inline float Huber::ObjectiveScore(float dErrorSquared, const float dSigmaSquared)
{
    if(dErrorSquared< dSigmaSquared) return 0.5f * dErrorSquared;
    else
    {
        float dSigma = sqrt(dSigmaSquared);
        float dError = sqrt(dErrorSquared);
        return dSigma * ( dError - 0.5 * dSigma);
    }
}

_CPU_AND_GPU_CODE_ inline float LeastSquares::Weight(float dErrorSquared, float dSigmaSquared)
{
    return 1.0f;
}

_CPU_AND_GPU_CODE_ inline float LeastSquares::SquareRootWeight(float dErrorSquared, float dSigmaSquared)
{
    return 1.0f;
}

_CPU_AND_GPU_CODE_ inline float LeastSquares::ObjectiveScore(float dErrorSquared, const float dSigmaSquared)
{
    return dErrorSquared;
}