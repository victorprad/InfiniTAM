// InfiniTAM: Surffuse. Copyright (c) Torr Vision Group and the authors of InfiniTAM, 2016.

#include "ITMSurfelRenderState.h"

namespace ITMLib
{

//#################### CONSTRUCTORS ####################

ITMSurfelRenderState::ITMSurfelRenderState(const Vector2i& indexImageSize, int supersamplingFactor)
{
  depthBuffer = new ORUtils::Image<int>(indexImageSize, true, true);
  surfelIndexImage = new ORUtils::Image<unsigned int>(indexImageSize, true, true);

  Vector2i indexImageSizeSuper = indexImageSize * supersamplingFactor;
  depthBufferSuper = new ORUtils::Image<int>(indexImageSizeSuper, true, true);
  surfelIndexImageSuper = new ORUtils::Image<unsigned int>(indexImageSizeSuper, true, true);
}

//#################### DESTRUCTOR ####################

ITMSurfelRenderState::~ITMSurfelRenderState()
{
  delete depthBuffer;
  delete depthBufferSuper;
  delete surfelIndexImage;
  delete surfelIndexImageSuper;
}

//#################### PUBLIC MEMBER FUNCTIONS ####################

ORUtils::Image<int> *ITMSurfelRenderState::GetDepthBuffer()
{
  return depthBuffer;
}

const ORUtils::Image<int> *ITMSurfelRenderState::GetDepthBuffer() const
{
  return depthBuffer;
}

ORUtils::Image<int> *ITMSurfelRenderState::GetDepthBufferSuper()
{
  return depthBufferSuper;
}

const ORUtils::Image<int> *ITMSurfelRenderState::GetDepthBufferSuper() const
{
  return depthBufferSuper;
}

ORUtils::Image<unsigned int> *ITMSurfelRenderState::GetIndexImage()
{
  return surfelIndexImage;
}

const ORUtils::Image<unsigned int> *ITMSurfelRenderState::GetIndexImage() const
{
  return surfelIndexImage;
}

ORUtils::Image<unsigned int> *ITMSurfelRenderState::GetIndexImageSuper()
{
  return surfelIndexImageSuper;
}

const ORUtils::Image<unsigned int> *ITMSurfelRenderState::GetIndexImageSuper() const
{
  return surfelIndexImageSuper;
}

}
