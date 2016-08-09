// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM

#pragma once

namespace ITMLib
{
	/// The tracker iteration type used to define the tracking iteration regime
	enum TrackerIterationType
	{
		TRACKER_ITERATION_ROTATION = 1,
		TRACKER_ITERATION_TRANSLATION = 2,
		TRACKER_ITERATION_BOTH = 3,
		TRACKER_ITERATION_NONE = 4
	};
}
