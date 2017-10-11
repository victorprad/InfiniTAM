// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#include "GraphEdge.h"

#include "../ORUtils/SE3Pose.h"

namespace MiniSlamGraph
{
	class GraphEdgeSE3 : public GraphEdge
	{
	public:
		typedef ORUtils::SE3Pose SE3;

		int getMeasureDimensions(void) const
		{
			return 6;
		}
		void setMeasurement(const double *v)
		{
			for (int i = 0; i < 6; ++i) mMeasuredPose[i] = v[i];
		}
		void getMeasurement(double *v) const
		{
			for (int i = 0; i < 6; ++i) v[i] = mMeasuredPose[i];
		}

		void setMeasurementSE3(const SE3 & pose);
		SE3 getMeasurementSE3(void) const;

		void computeResidualVector(const NodeIndex & nodes, double *dest) const;
		bool computeJacobian(const NodeIndex & nodes, int id, double *j) const;

	private:
		double mMeasuredPose[6];
	};
}