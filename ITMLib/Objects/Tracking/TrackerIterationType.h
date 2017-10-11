// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

namespace ITMLib
{
	/// The tracker iteration type used to define the tracking iteration regime
	enum TrackerIterationType
	{
		TRACKER_ITERATION_ROTATION,
		TRACKER_ITERATION_TRANSLATION,
		TRACKER_ITERATION_BOTH,
		TRACKER_ITERATION_NONE
	};
}
