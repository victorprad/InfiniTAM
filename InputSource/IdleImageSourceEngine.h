// Copyright 2014-2018 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#include "ImageSourceEngine.h"

namespace InputSource {

/**
 * \brief An instance of this class can be used to promise RGB and depth images of known sizes but never actually yield any.
 *
 * This is useful when loading a model - the pipeline still needs to know the image sizes, but will never receive any images.
 */
class IdleImageSourceEngine : public BaseImageSourceEngine
{
  //#################### CONSTRUCTORS ####################
public:
  /**
   * \brief Constructs an idle image source engine.
   *
   * \param calibFilename The name of the file containing the calibration parameters for the camera.
   */
  explicit IdleImageSourceEngine(const char *calibFilename);

  //#################### PUBLIC MEMBER FUNCTIONS ####################
public:
  /** Override */
  virtual Vector2i getDepthImageSize() const;

  /** Override */
  virtual void getImages(ITMUChar4Image *rgb, ITMShortImage *rawDepth);

  /** Override */
  virtual Vector2i getRGBImageSize() const;

  /** Override */
  virtual bool hasImagesNow() const;

  /** Override */
  virtual bool hasMoreImages() const;
};

}
