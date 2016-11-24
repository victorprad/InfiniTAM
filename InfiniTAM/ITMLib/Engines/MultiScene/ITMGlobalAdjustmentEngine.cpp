// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#include "ITMGlobalAdjustmentEngine.h"

#include "../../../MiniSlamGraphLib/GraphNodeSE3.h"
#include "../../../MiniSlamGraphLib/GraphEdgeSE3.h"
#include "../../../MiniSlamGraphLib/SlamGraphErrorFunction.h"
#include "../../../MiniSlamGraphLib/LevenbergMarquardtMethod.h"

#include <mutex>
#include <thread>
#include <condition_variable>

using namespace ITMLib;

struct ITMGlobalAdjustmentEngine::PrivateData 
{
	PrivateData(void) { stopThread = false; wakeupSent = false; }
	std::mutex workingData_mutex;
	std::mutex processedData_mutex;
	std::thread processingThread;
	bool stopThread;

	std::mutex wakeupMutex;
	std::condition_variable wakeupCond;
	bool wakeupSent;
};

ITMGlobalAdjustmentEngine::ITMGlobalAdjustmentEngine(void)
{
	privateData = new PrivateData();
	workingData = NULL;
	processedData = NULL;
}

ITMGlobalAdjustmentEngine::~ITMGlobalAdjustmentEngine(void)
{
	stopSeparateThread();
	if (workingData != NULL) delete workingData;
	if (processedData != NULL) delete processedData;
	delete privateData;
}

bool ITMGlobalAdjustmentEngine::hasNewEstimates(void) const
{
	return (processedData != NULL);
}

bool ITMGlobalAdjustmentEngine::retrieveNewEstimates(ITMMapGraphManager & dest)
{
	if (processedData == NULL) return false;

	privateData->processedData_mutex.lock();
	PoseGraphToMultiScene(*processedData, dest);
	delete processedData;
	processedData = NULL;
	privateData->processedData_mutex.unlock();

	return true;
}

bool ITMGlobalAdjustmentEngine::isBusyEstimating(void) const
{
	// if someone else is currently using the mutex (most likely the
	// consumer thread), we consider the global adjustment engine to
	// be busy
	if (!privateData->workingData_mutex.try_lock()) return true;

	privateData->workingData_mutex.unlock();
	return false;
}

bool ITMGlobalAdjustmentEngine::updateMeasurements(const ITMMapGraphManager & src)
{
	// busy, can't accept new measurements at the moment
	if (!privateData->workingData_mutex.try_lock()) return false;

	if (workingData == NULL) workingData = new MiniSlamGraph::PoseGraph;
	MultiSceneToPoseGraph(src, *workingData);
	privateData->workingData_mutex.unlock();
	return true;
}

bool ITMGlobalAdjustmentEngine::runGlobalAdjustment(bool blockingWait)
{
	// first make sure there is new data and we have exclusive access to it
	if (workingData == NULL) return false;

	if (blockingWait) privateData->workingData_mutex.lock();
	else if (!privateData->workingData_mutex.try_lock()) return false;

	// now run the actual global adjustment
	workingData->prepareEvaluations();
	MiniSlamGraph::SlamGraphErrorFunction errf(*workingData);
	MiniSlamGraph::SlamGraphErrorFunction::Parameters para(*workingData);
	MiniSlamGraph::LevenbergMarquardtMethod::minimize(errf, para);
	workingData->setNodeIndex(para.getNodes());

	// copy data to output buffer
	privateData->processedData_mutex.lock();
	if (processedData != NULL) delete processedData;
	processedData = workingData;
	workingData = NULL;
	privateData->processedData_mutex.unlock();

	privateData->workingData_mutex.unlock();

	return true;
}

bool ITMGlobalAdjustmentEngine::startSeparateThread(void)
{
	if (privateData->processingThread.joinable()) return false;

	privateData->processingThread = std::thread(&ITMGlobalAdjustmentEngine::estimationThreadMain, this);
	return true;
}

bool ITMGlobalAdjustmentEngine::stopSeparateThread(void)
{
	if (!privateData->processingThread.joinable()) return false;

	privateData->stopThread = true;
	wakeupSeparateThread();
	privateData->processingThread.join();
	return true;
}

void ITMGlobalAdjustmentEngine::estimationThreadMain(void)
{
	while (!privateData->stopThread)
	{
		runGlobalAdjustment(true);
		std::unique_lock<std::mutex> lck(privateData->wakeupMutex);
		if (!privateData->wakeupSent) privateData->wakeupCond.wait(lck);
		privateData->wakeupSent = false;
	}
}

void ITMGlobalAdjustmentEngine::wakeupSeparateThread(void)
{
	std::unique_lock<std::mutex> lck(privateData->wakeupMutex);
	privateData->wakeupSent = true;
	privateData->wakeupCond.notify_all();
}

void ITMGlobalAdjustmentEngine::MultiSceneToPoseGraph(const ITMMapGraphManager & src, MiniSlamGraph::PoseGraph & dest)
{
	for (int localMapId = 0; localMapId < (int)src.numLocalMaps(); ++localMapId)
	{
		MiniSlamGraph::GraphNodeSE3 *pose = new MiniSlamGraph::GraphNodeSE3();

		pose->setId(localMapId);
		pose->setPose(src.getEstimatedGlobalPose(localMapId));
		if (localMapId == 0) pose->setFixed(true);
		
		dest.addNode(pose);
	}

	for (int localMapId = 0; localMapId < (int)src.numLocalMaps(); ++localMapId) 
	{
		const ConstraintList & constraints = src.getConstraints(localMapId);
		for (ConstraintList::const_iterator it = constraints.begin(); it != constraints.end(); ++it) 
		{
			MiniSlamGraph::GraphEdgeSE3 *odometry = new MiniSlamGraph::GraphEdgeSE3();
			
			odometry->setFromNodeId(localMapId);
			odometry->setToNodeId(it->first);
			odometry->setMeasurementSE3(it->second.GetAccumulatedObservations());
			
			//TODO odometry->setInformation
			dest.addEdge(odometry);
		}
	}
}

void ITMGlobalAdjustmentEngine::PoseGraphToMultiScene(const MiniSlamGraph::PoseGraph & src, ITMMapGraphManager & dest)
{
	for (int localMapId = 0; localMapId < (int)dest.numLocalMaps(); ++localMapId) 
	{
		MiniSlamGraph::SlamGraph::NodeIndex::const_iterator it = src.getNodeIndex().find(localMapId);
		if (it == src.getNodeIndex().end()) continue;
		const MiniSlamGraph::GraphNodeSE3 *pose = (const MiniSlamGraph::GraphNodeSE3*)it->second;
		ORUtils::SE3Pose outpose = pose->getPose();
		dest.setEstimatedGlobalPose(localMapId, outpose);
	}
}

