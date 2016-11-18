// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../../../ORUtils/SE3Pose.h"

namespace ITMLib
{
	class ITMIMUCalibrator
	{
	public:
		virtual void RegisterMeasurement(const Matrix3f & R) = 0;
		virtual Matrix3f GetDifferentialRotationChange() = 0;

		ITMIMUCalibrator() { }

		virtual ~ITMIMUCalibrator(void) { }

		// Suppress the default copy constructor and assignment operator
		ITMIMUCalibrator(const ITMIMUCalibrator&);
		ITMIMUCalibrator& operator=(const ITMIMUCalibrator&);
	};

	class ITMIMUCalibrator_iPad : public ITMIMUCalibrator
	{
	private:
		ORUtils::SE3Pose *imuPose_imucoords, *imuPose_cameracoords;
		Vector3f t_imu, r_imu;
		Matrix3f inv_oldR_imu;
		Matrix3f newR_imu, oldR_imu;
		bool hasTwoFrames;

	public: 
		void RegisterMeasurement(const Matrix3f & R)
		{
			oldR_imu = imuPose_imucoords->GetR();

			imuPose_imucoords->SetR(R);

			imuPose_imucoords->GetParams(t_imu, r_imu);
			imuPose_imucoords->SetFrom(t_imu, -r_imu);

			newR_imu = imuPose_imucoords->GetR();
		}

		Matrix3f GetDifferentialRotationChange()
		{
			if (hasTwoFrames)
			{
				oldR_imu.inv(inv_oldR_imu);
				imuPose_cameracoords->SetR(imuPose_imucoords->GetR() * inv_oldR_imu);

				imuPose_cameracoords->GetParams(t_imu, r_imu);
				imuPose_cameracoords->SetFrom(t_imu.x, t_imu.y, t_imu.z, -r_imu.y, -r_imu.x, -r_imu.z);
			}
			
			hasTwoFrames = true;
			return imuPose_cameracoords->GetR();
		}

		ITMIMUCalibrator_iPad() : ITMIMUCalibrator() 
		{ 
			hasTwoFrames = false;

			imuPose_imucoords = new ORUtils::SE3Pose();
			imuPose_imucoords->SetFrom(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);

			imuPose_cameracoords = new ORUtils::SE3Pose();
			imuPose_cameracoords->SetFrom(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);

			oldR_imu.setIdentity();
		}

		~ITMIMUCalibrator_iPad(void) 
		{
			delete imuPose_imucoords;
			delete imuPose_cameracoords;
		}
	};
}
