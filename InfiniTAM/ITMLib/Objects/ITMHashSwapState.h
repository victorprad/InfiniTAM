// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM

#pragma once

#include "ITMMath.h"

struct ITMHashSwapState
{
	/// 0 - most recent data is on host, data not currently in active
	///     memory
	/// 1 - data both on host and in active memory, information has not
	///     yet been combined
	/// 2 - most recent data is in active memory, should save this data
	///     back to host at some point
	uchar state;
};
