// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#include "CompositeImageSourceEngine.h"

namespace InputSource {

//#################### CONSTRUCTORS ####################

CompositeImageSourceEngine::CompositeImageSourceEngine()
: m_curSubengineIndex(0)
{}

//#################### DESTRUCTOR ####################

CompositeImageSourceEngine::~CompositeImageSourceEngine()
{
  for(size_t i = 0, size = m_subengines.size(); i < size; ++i)
  {
    delete m_subengines[i];
  }
}

//#################### PUBLIC MEMBER FUNCTIONS ####################

void CompositeImageSourceEngine::addSubengine(ImageSourceEngine *subengine)
{
  m_subengines.push_back(subengine);
}

ITMLib::ITMRGBDCalib CompositeImageSourceEngine::getCalib(void) const
{
  // There is an assumption being made that the calibrations for all the sub-engines are the same,
  // although this is not currently being enforced.
  if(!m_subengines.empty()) return m_subengines[0]->getCalib();
  else throw std::runtime_error("Cannot get calibration parameters from an empty composite image source engine");
}

const ImageSourceEngine *CompositeImageSourceEngine::getCurrentSubengine(void) const
{
  return m_curSubengineIndex < m_subengines.size() ? m_subengines[m_curSubengineIndex] : NULL;
}

Vector2i CompositeImageSourceEngine::getDepthImageSize(void) const
{
  // There is an assumption being made that the depth image sizes for all the sub-engines are the same,
  // although this is not currently being enforced.
  if(!m_subengines.empty()) return m_subengines[0]->getDepthImageSize();
  else throw std::runtime_error("Cannot get the depth image size from an empty composite image source engine");
}

void CompositeImageSourceEngine::getImages(ITMUChar4Image *rgb, ITMShortImage *rawDepth)
{
  if(advanceToNextImages()) m_subengines[m_curSubengineIndex]->getImages(rgb, rawDepth);
}

Vector2i CompositeImageSourceEngine::getRGBImageSize(void) const
{
  // There is an assumption being made that the RGB image sizes for all the sub-engines are the same,
  // although this is not currently being enforced.
  if(!m_subengines.empty()) return m_subengines[0]->getRGBImageSize();
  else throw std::runtime_error("Cannot get the RGB image size from an empty composite image source engine");
}

bool CompositeImageSourceEngine::hasImagesNow(void) const
{
  const ImageSourceEngine *curSubengine = advanceToNextImages();
  return curSubengine != NULL && curSubengine->hasImagesNow();
}

bool CompositeImageSourceEngine::hasMoreImages(void) const
{
  return advanceToNextImages() != NULL;
}

//#################### PRIVATE MEMBER FUNCTIONS ####################

const ImageSourceEngine *CompositeImageSourceEngine::advanceToNextImages(void) const
{
  const ImageSourceEngine *curSubengine = getCurrentSubengine();
  while(curSubengine && !curSubengine->hasMoreImages())
  {
    ++m_curSubengineIndex;
    curSubengine = getCurrentSubengine();
  }
  return curSubengine;
}

}
