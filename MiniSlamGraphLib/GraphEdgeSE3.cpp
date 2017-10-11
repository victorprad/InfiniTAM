// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#include "GraphEdgeSE3.h"
#include "GraphNodeSE3.h"

#include "QuaternionHelpers.h"

#include <stdio.h>

using namespace MiniSlamGraph;

static void MatrixToMQT(const ORUtils::Matrix4<float> & m, double *qt)
{
	double R[9];
	for (int r = 0; r < 3; ++r) for (int c = 0; c < 3; ++c) R[r * 3 + c] = m.m[c * 4 + r];
	double qtmp[4];
	QuaternionHelpers::QuaternionFromRotationMatrix(R, qtmp);

	for (int i = 0; i < 3; ++i) qt[i] = qtmp[i + 1];
	for (int i = 0; i < 3; ++i) qt[3 + i] = m.m[3 * 4 + i];
}

static void MQTToMatrix(const double *qt, ORUtils::Matrix4<float> & m)
{
	double qtmp[4];
	for (int i = 0; i < 3; ++i) qtmp[i + 1] = qt[i];
	qtmp[0] = sqrt(1.0f - qt[0] * qt[0] - qt[1] * qt[1] - qt[2] * qt[2]);

	double R[9];
	QuaternionHelpers::RotationMatrixFromQuaternion(qtmp, R);
	for (int r = 0; r < 3; ++r) for (int c = 0; c < 3; ++c) m.m[c * 4 + r] = (float)R[r * 3 + c];
	for (int i = 0; i < 3; ++i) m.m[3 * 4 + i] = (float)qt[3 + i];
	m.m[0 * 4 + 3] = m.m[1 * 4 + 3] = m.m[2 * 4 + 3] = 0.0f;
	m.m[3 * 4 + 3] = 1.0f;
}

static ORUtils::Matrix4<float> se3_generator(int idx)
{
	ORUtils::Matrix4<float> ret;
	ret.setZeros();
	if (idx < 3) {
		ret.m[3 * 4 + idx] = 1.0f;
	}
	else {
		int r = (idx + 1) % 3;
		int c = (idx + 2) % 3;
		ret.m[c * 4 + r] = -1.0f;
		ret.m[r * 4 + c] = 1.0f;
	}
	return ret;
}

void GraphEdgeSE3::setMeasurementSE3(const SE3 & pose)
{
	MatrixToMQT(pose.GetM(), mMeasuredPose);
}

GraphEdgeSE3::SE3 GraphEdgeSE3::getMeasurementSE3(void) const
{
	ORUtils::Matrix4<float> m;
	MQTToMatrix(mMeasuredPose, m);
	return SE3(m);
}

void GraphEdgeSE3::computeResidualVector(const GraphEdgeSE3::NodeIndex & nodes, double *dest) const
{
	// get poses of "from" and "to" nodes
	const GraphNodeSE3 *fromNode = (const GraphNodeSE3*)nodes.find(fromNodeId())->second;
	const GraphNodeSE3 *toNode = (const GraphNodeSE3*)nodes.find(toNodeId())->second;
	const SE3 & fromPose = fromNode->getPose();
	const SE3 & toPose = toNode->getPose();

	// get measured pose as a matrix
	ORUtils::Matrix4<float> m;
	MQTToMatrix(mMeasuredPose, m);

	//compute residual
	ORUtils::Matrix4<float> residualPose(fromPose.GetM() * toPose.GetInvM() * m);
	MatrixToMQT(residualPose, dest);
}

bool GraphEdgeSE3::computeJacobian(const NodeIndex & nodes, int id, double *jacobian) const
{
	const GraphNodeSE3 *node_f = (const GraphNodeSE3*)nodes.find(fromNodeId())->second;
	const GraphNodeSE3 *node_t = (const GraphNodeSE3*)nodes.find(toNodeId())->second;
	const SE3 & fromPose = node_f->getPose();
	const SE3 & toPose = node_t->getPose();

	// get measured pose as a matrix
	ORUtils::Matrix4<float> m;
	MQTToMatrix(mMeasuredPose, m);

	//compute residual
	ORUtils::Matrix4<float> AB(fromPose.GetM() * toPose.GetInvM());

	ORUtils::Matrix4<float> dAB_dx[6];
	if (id == node_f->getId()) {
		for (int i = 0; i < 6; ++i) dAB_dx[i] = se3_generator(i) * AB;
	}
	else if (id == node_t->getId()) {
		for (int i = 0; i < 6; ++i) dAB_dx[i] = AB * se3_generator(i) * -1.0f;
	}
	else return false;

	double dQ_dR[4 * 9];
	{
		ORUtils::Matrix4<float> ABm = AB * m;
		double ABm_array[9];
		for (int r = 0; r < 3; ++r) for (int c = 0; c < 3; ++c) ABm_array[r * 3 + c] = ABm.m[c * 4 + r];
		QuaternionHelpers::dQuaternion_dRotationMatrix(ABm_array, dQ_dR);
	}
	for (int gi = 0; gi < 6; ++gi) {
		ORUtils::Matrix4<float> d_inner_dx = dAB_dx[gi] * m;
		for (int qi = 0; qi < 3; ++qi) {
			jacobian[qi * 6 + gi] = 0.0f;
			for (int r = 0; r < 3; ++r) for (int c = 0; c < 3; ++c) jacobian[qi * 6 + gi] += dQ_dR[(qi + 1) * 9 + (r * 3 + c)] * d_inner_dx.m[c * 4 + r];
		}
		for (int ti = 0; ti < 3; ++ti) {
			jacobian[(ti + 3) * 6 + gi] = d_inner_dx.m[3 * 4 + ti];
		}
	}
	return true;
}