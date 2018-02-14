// Copyright 2014-2018 Oxford University Innovation Limited and the authors of InfiniTAM

#include "IdleImageSourceEngine.h"

#include <stdexcept>

namespace InputSource {

//#################### CONSTRUCTORS ####################

IdleImageSourceEngine::IdleImageSourceEngine(const char *calibFilename)
: BaseImageSourceEngine(calibFilename)
{}

//#################### PUBLIC MEMBER FUNCTIONS ####################

Vector2i IdleImageSourceEngine::getDepthImageSize() const
{
  return calib.intrinsics_d.imgSize;
}

void IdleImageSourceEngine::getImages(ITMUChar4Image *rgb, ITMShortImage *rawDepth)
{
  // This should never be called.
  throw std::runtime_error("Error: Attempting to get images from an idle image source engine");
}

Vector2i IdleImageSourceEngine::getRGBImageSize() const
{
  return calib.intrinsics_rgb.imgSize;
}

bool IdleImageSourceEngine::hasImagesNow() const
{
  // An idle image source engine is never ready to yield images.
  return false;
}

bool IdleImageSourceEngine::hasMoreImages() const
{
  // An idle image source engine always has more images available in principle, even it is not willing to yield any.
  return true;
}

}
