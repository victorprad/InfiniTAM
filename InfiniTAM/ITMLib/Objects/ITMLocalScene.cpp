#include "ITMLocalScene.h"

using namespace ITMLib;

ITMPoseConstraint::ITMPoseConstraint(void)
{
	accu_poses.setZeros();
	accu_num = 0;
}

void ITMPoseConstraint::AddObservation(const Matrix4f & relative_pose)
{
	accu_poses = (accu_poses * (float)accu_num + relative_pose)/(float)(accu_num+1);
	accu_num++;
}

Matrix4f ITMPoseConstraint::GetAccumulatedInfo(void) const
{
	ITMPose tmp_pose;
	tmp_pose.SetM(accu_poses);
	tmp_pose.Coerce();
	return tmp_pose.GetM();
}

