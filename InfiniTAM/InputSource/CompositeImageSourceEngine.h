// Copyright 2014-2016 Isis Innovation Limited and the authors of InfiniTAM

#pragma once

#include <vector>

#include "ImageSourceEngine.h"

namespace InputSource {

/**
 * \brief TODO
 */
class CompositeImageSourceEngine : public ImageSourceEngine
{
  //#################### PRIVATE VARIABLES ####################
private:
  /** The index of the current sub-engine. */
  size_t m_curSubengineIndex;

  /** TODO */
  std::vector<ImageSourceEngine*> m_subengines;

  //#################### CONSTRUCTORS ####################
public:
  /**
   * \brief Constructs a composite image source engine.
   */
  CompositeImageSourceEngine();

  //#################### DESTRUCTOR ####################
public:
  /**
   * \brief Destroys the composite image source engine.
   */
  ~CompositeImageSourceEngine();

  //#################### COPY CONSTRUCTOR & ASSIGNMENT OPERATOR ####################
private:
  // Deliberately private and unimplemented.
  CompositeImageSourceEngine(const CompositeImageSourceEngine&);
  CompositeImageSourceEngine& operator=(const CompositeImageSourceEngine&);

  //#################### PUBLIC MEMBER FUNCTIONS ####################
public:
  /**
   * \brief TODO
   */
  void addSubengine(ImageSourceEngine *subengine);

  /**
   * \brief Gets the current sub-engine.
   *
   * \return  The current sub-engine.
   */
  ImageSourceEngine *getCurrentSubengine(void);

  /** Override */
  virtual Vector2i getDepthImageSize(void);

  /** Override */
  virtual void getImages(ITMUChar4Image *rgb, ITMShortImage *rawDepth);

  /** Override */
  virtual Vector2i getRGBImageSize(void);

  /** Override */
  virtual bool hasMoreImages(void);

  //#################### PRIVATE MEMBER FUNCTIONS ####################
private:
  /**
   * \brief TODO
   */
  ImageSourceEngine *advanceToNextImages(void);
};

}
