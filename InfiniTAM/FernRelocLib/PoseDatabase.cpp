// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#include "PoseDatabase.h"

#include <fstream>
#include <iterator>

using namespace FernRelocLib;

PoseDatabase::PoseDatabase(void) {}
PoseDatabase::~PoseDatabase(void) {}

void PoseDatabase::storePose(int id, const ORUtils::SE3Pose & pose, int sceneId)
{
	if (id < 0) return;
	if ((unsigned)id >= mPoses.size()) mPoses.resize(id + 1);

	mPoses[id] = PoseInScene(pose, sceneId);
}

PoseDatabase::PoseInScene PoseDatabase::retrieveWAPose(int k, int ids[], float distances[]) const
{
	ORUtils::Matrix4<float> m;
	m.setZeros();

	int sceneID = -1;
	float sumWeights = 0.0f;
	for (int i = 0; i < k; ++i)
	{
		const PoseInScene & pose = retrievePose(ids[i]);
		if (sceneID == -1) sceneID = pose.sceneIdx;
		else if (sceneID != pose.sceneIdx) continue;

		float weight = 1.0f - distances[i];
		m += pose.pose.GetM() * weight;
		sumWeights += weight;
	}

	m = m * (1.0f / sumWeights);
	return PoseDatabase::PoseInScene(ORUtils::SE3Pose(m), sceneID);
}

void PoseDatabase::SaveToFile(const std::string &fileName)
{
	std::ofstream ofs(fileName.c_str());
	if (!ofs) throw std::runtime_error("Could not open " + fileName + " for reading");

	size_t numPoses = mPoses.size();
	ofs << numPoses << '\n';

	for (size_t i = 0; i < numPoses; i++)
	{
		ofs << mPoses[i].sceneIdx << ' ';

		const float *params = mPoses[i].pose.GetParams();
		std::copy(params, params + 6, std::ostream_iterator<float>(ofs, " "));

		ofs << '\n';
	}
}

void PoseDatabase::LoadFromFile(const std::string &fileName)
{
	int tot, sceneID;
	float tx, ty, tz, rx, ry, rz;

	std::ifstream ifs(fileName.c_str());
	if (!ifs) throw std::runtime_error("unable to open " + fileName);

	ifs >> tot;
	for (int i = 0; i < tot; i++)
	{
		ifs >> sceneID >> tx >> ty >> tz >> rx >> ry >> rz;
		ORUtils::SE3Pose pose(tx, ty, tz, rx, ry, rz);
		storePose(i, pose, sceneID);
	}
}
