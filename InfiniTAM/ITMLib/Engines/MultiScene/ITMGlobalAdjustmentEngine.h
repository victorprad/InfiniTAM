// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../../../MiniSlamGraphLib/PoseGraph.h"
#include "ITMMapGraphManager.h"

namespace ITMLib {

	/** This engine computes global pose adjustments using pose graph optimisation.
		The basic idea is that whenever some "main engine" considers it necessary,
		it should send new information to such a global adjustment step using the
		function updateMeasurements(). Should for whatever reason the engine reject
		these new measurements, this function should return false immediately, and
		the main engine has to keep resubmitting the same data again. On the other
		hand, retrieveNewEstimates() allows the main engine to retrieve the results
		from pose graph optimisation, if there are any, or false, if there aren't.

		The pose graph optimisation itself can be called explicitly using the method
		runGlobalAdjustment(). However, the whole class is also designed to be run
		in a separate thread in the background. The corresponding methods are
		startSeparateThread() and stopSeparateThread(), and whenever new
		measurements are being passed, a call to wakeupSeparateThread() is also
		recommended. The thread will reject new data while a pose graph optimisation
		is currently in progress, and it may go to sleep otherwise.
	*/
	class ITMGlobalAdjustmentEngine {
	private:
		struct PrivateData;

	public:
		ITMGlobalAdjustmentEngine(void);
		~ITMGlobalAdjustmentEngine(void);

		bool hasNewEstimates(void) const;

		// Check whether pose graph optimisation has converged and produced a
		// new result. if it hasn't return false, otherwise copy them over
		bool retrieveNewEstimates(ITMMapGraphManager & dest);

		bool isBusyEstimating(void) const;

		// Check whether thread is busy, if it is, return false, otherwise
		// create a copy of all new measurements and make it busy
		bool updateMeasurements(const ITMMapGraphManager & src);

		bool runGlobalAdjustment(bool blockingWait = false);

		bool startSeparateThread(void);
		bool stopSeparateThread(void);
		void wakeupSeparateThread(void);

	private:
		void estimationThreadMain(void);

		static void MultiSceneToPoseGraph(const ITMMapGraphManager & src, MiniSlamGraph::PoseGraph & dest);
		static void PoseGraphToMultiScene(const MiniSlamGraph::PoseGraph & src, ITMMapGraphManager & dest);

		MiniSlamGraph::PoseGraph *workingData;
		MiniSlamGraph::PoseGraph *processedData;

		PrivateData *privateData;
	};
}
