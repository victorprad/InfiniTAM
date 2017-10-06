// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#include <vector>

#include "ImageSourceEngine.h"

namespace InputSource {

/**
 * \brief An instance of this class can be used to compose multiple image source engines sequentially.
 */
class CompositeImageSourceEngine : public ImageSourceEngine
{
  //#################### PRIVATE VARIABLES ####################
private:
  /** The index of the current sub-engine. */
  mutable size_t m_curSubengineIndex;

  /** The sequence of image source engines from which the composite will yield images. */
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
   * \brief Adds an image source engine to the composite.
   *
   * \param subengine The image source engine to add.
   */
  void addSubengine(ImageSourceEngine *subengine);

  /** Override */
  virtual ITMLib::ITMRGBDCalib getCalib(void) const;

  /**
   * \brief Gets the current sub-engine.
   *
   * \return  The current sub-engine.
   */
  const ImageSourceEngine *getCurrentSubengine(void) const;

  /** Override */
  virtual Vector2i getDepthImageSize(void) const;

  /** Override */
  virtual void getImages(ITMUChar4Image *rgb, ITMShortImage *rawDepth);

  /** Override */
  virtual Vector2i getRGBImageSize(void) const;

  /** Override */
  virtual bool hasImagesNow(void) const;

  /** Override */
  virtual bool hasMoreImages(void) const;

  //#################### PRIVATE MEMBER FUNCTIONS ####################
private:
  /**
   * \brief Advances to the sub-engine (if any) that can provide the next images.
   *
   * \return  The sub-engine, if any, that can provide the next images, or NULL if there are no more images to provide.
   */
  const ImageSourceEngine *advanceToNextImages(void) const;
};

}
