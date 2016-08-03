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

ImageSourceEngine *CompositeImageSourceEngine::getCurrentSubengine(void)
{
  return m_curSubengineIndex < m_subengines.size() ? m_subengines[m_curSubengineIndex] : NULL;
}

Vector2i CompositeImageSourceEngine::getDepthImageSize(void)
{
  ImageSourceEngine *curSubengine = getCurrentSubengine();
  return curSubengine ? curSubengine->getDepthImageSize() : Vector2i(0,0);
}

void CompositeImageSourceEngine::getImages(ITMUChar4Image *rgb, ITMShortImage *rawDepth)
{
  ImageSourceEngine *curSubengine = advanceToNextImages();
  if(curSubengine) curSubengine->getImages(rgb, rawDepth);
}

Vector2i CompositeImageSourceEngine::getRGBImageSize(void)
{
  ImageSourceEngine *curSubengine = getCurrentSubengine();
  return curSubengine ? curSubengine->getRGBImageSize() : Vector2i(0,0);
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
