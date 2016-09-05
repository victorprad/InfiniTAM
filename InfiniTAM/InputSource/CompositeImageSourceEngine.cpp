// Copyright 2014-2016 Isis Innovation Limited and the authors of InfiniTAM

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

ImageSourceEngine *CompositeImageSourceEngine::getCurrentSubengine(void)
{
  return m_curSubengineIndex < m_subengines.size() ? m_subengines[m_curSubengineIndex] : NULL;
}

Vector2i CompositeImageSourceEngine::getDepthImageSize(void)
{
  // There is an assumption being made that the depth image sizes for all the sub-engines are the same,
  // although this is not currently being enforced.
  if(!m_subengines.empty()) return m_subengines[0]->getDepthImageSize();
  else throw std::runtime_error("Cannot get the depth image size from an empty composite image source engine");
}

void CompositeImageSourceEngine::getImages(ITMUChar4Image *rgb, ITMShortImage *rawDepth)
{
  ImageSourceEngine *curSubengine = advanceToNextImages();
  if(curSubengine) curSubengine->getImages(rgb, rawDepth);
}

Vector2i CompositeImageSourceEngine::getRGBImageSize(void)
{
  // There is an assumption being made that the RGB image sizes for all the sub-engines are the same,
  // although this is not currently being enforced.
  if(!m_subengines.empty()) return m_subengines[0]->getRGBImageSize();
  else throw std::runtime_error("Cannot get the RGB image size from an empty composite image source engine");
}

bool CompositeImageSourceEngine::hasMoreImages(void)
{
  return advanceToNextImages() != NULL;
}

//#################### PRIVATE MEMBER FUNCTIONS ####################

ImageSourceEngine *CompositeImageSourceEngine::advanceToNextImages(void)
{
  ImageSourceEngine *curSubengine = getCurrentSubengine();
  while(curSubengine && !curSubengine->hasMoreImages())
  {
    ++m_curSubengineIndex;
    curSubengine = getCurrentSubengine();
  }
  return curSubengine;
}

}
