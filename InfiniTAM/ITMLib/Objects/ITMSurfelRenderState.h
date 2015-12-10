// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../../ORUtils/Image.h"
#include "../Utils/ITMMath.h"

namespace ITMLib
{
  /**
   * \brief TODO
   */
  class ITMSurfelRenderState
  {
    //#################### PRIVATE VARIABLES ####################
  private:
    /** TODO */
    ORUtils::Image<int> *depthBuffer;

    /** TODO */
    ORUtils::Image<int> *depthBufferSuper;

    /** TODO */
    ORUtils::Image<unsigned int> *surfelIndexImage;

    /** TODO */
    ORUtils::Image<unsigned int> *surfelIndexImageSuper;

    //#################### CONSTRUCTORS ####################
  public:
    /**
     * \brief TODO
     */
    ITMSurfelRenderState(const Vector2i& indexImageSize);

    //#################### DESTRUCTOR ####################
  public:
    /**
     * \brief Destroys the render state.
     */
    ~ITMSurfelRenderState();

    //#################### COPY CONSTRUCTOR & ASSIGNMENT OPERATOR ####################
  private:
    // Deliberately private and unimplemented.
    ITMSurfelRenderState(const ITMSurfelRenderState&);
    ITMSurfelRenderState& operator=(const ITMSurfelRenderState&);

    //#################### PUBLIC MEMBER FUNCTIONS ####################
  public:
    /**
     * \brief TODO
     */
    ORUtils::Image<int> *GetDepthBuffer();

    /**
     * \brief TODO
     */
    const ORUtils::Image<int> *GetDepthBuffer() const;

    /**
     * \brief TODO
     */
    ORUtils::Image<int> *GetDepthBufferSuper();

    /**
     * \brief TODO
     */
    const ORUtils::Image<int> *GetDepthBufferSuper() const;

    /**
     * \brief TODO
     */
    ORUtils::Image<unsigned int> *GetIndexImage();

    /**
     * \brief TODO
     */
    const ORUtils::Image<unsigned int> *GetIndexImage() const;

    /**
     * \brief TODO
     */
    ORUtils::Image<unsigned int> *GetIndexImageSuper();

    /**
     * \brief TODO
     */
    const ORUtils::Image<unsigned int> *GetIndexImageSuper() const;

    /**
     * \brief TODO
     */
    int GetSuperScaleFactor() const;
  };
}
