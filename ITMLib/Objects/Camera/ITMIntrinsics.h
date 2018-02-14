// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../../Utils/ITMMath.h"

namespace ITMLib
{
	/**
	 * \brief Represents the intrinsic parameters for a projective camera.
	 */
	class ITMIntrinsics
	{
	public:
		/** The image size. */
		Vector2i imgSize;

		/** The actual intrinsic calibration parameters. */
		struct ProjectionParamsSimple
		{
			Vector4f all;
			float fx, fy, px, py;
		} projectionParamsSimple;

		/**
		 * \brief Makes a rescaled set of intrinsic parameters to work with a different image size.
		 *
		 * \param originalImageSize The original image size.
		 * \param newImageSize      The new image size.
		 */
		ITMIntrinsics MakeRescaled(const Vector2i& originalImageSize, const Vector2i& newImageSize) const
		{
			float fxNew = projectionParamsSimple.fx * newImageSize.x / originalImageSize.x;
			float fyNew = projectionParamsSimple.fy * newImageSize.y / originalImageSize.y;
			float pxNew = projectionParamsSimple.px * newImageSize.x / originalImageSize.x;
			float pyNew = projectionParamsSimple.py * newImageSize.y / originalImageSize.y;

			ITMIntrinsics intrinsics;
			intrinsics.SetFrom(newImageSize.width, newImageSize.height, fxNew, fyNew, pxNew, pyNew);
			return intrinsics;
		}

		/** Setup all the internal members of this class from
		    the given parameters. Everything is in pixel
		    coordinates.
			@param width  The image width
			@param height The image height
			@param fx     Focal length in x direction
			@param fy     Focal length in y direction
			@param cx     Principal point in x direction
			@param cy     Principal point in y direction
		*/
		void SetFrom(int width, int height, float fx, float fy, float cx, float cy)
		{
			imgSize.x = width;
			imgSize.y = height;

			projectionParamsSimple.fx = fx; 
			projectionParamsSimple.fy = fy;
			projectionParamsSimple.px = cx; 
			projectionParamsSimple.py = cy;

			projectionParamsSimple.all.x = fx; 
			projectionParamsSimple.all.y = fy;
			projectionParamsSimple.all.z = cx; 
			projectionParamsSimple.all.w = cy;
		}

		/**
		 * @brief Returns true if the two focal lengths have a different sign.
		 *
		 * @note  This is used to handle datasets such as ICL_NUIM and other non standard inputs
		 * 	      where one of the two focal lengths is negative: that causes the normals to point
		 * 	      away from the camera. This causes valid points to be ignored during visualisation
		 * 	      and tracking.
		 *
		 * 	      The problem presents itself only when computing normals as cross product of the
		 * 	      difference vectors between raycasted points and is thus solved by flipping
		 * 	      the normal direction.
		 */
		bool FocalLengthSignsDiffer() const
		{
			return projectionParamsSimple.fx * projectionParamsSimple.fy < 0.f;
		}

		ITMIntrinsics(void)
		{
			// standard calibration parameters for Kinect RGB camera. Not at all
			// accurate, though...
			SetFrom(640, 480, 580, 580, 320, 240);
		}
	};
}
