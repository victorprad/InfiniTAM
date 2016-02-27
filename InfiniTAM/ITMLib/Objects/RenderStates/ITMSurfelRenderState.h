// InfiniTAM: Surffuse. Copyright (c) Torr Vision Group and the authors of InfiniTAM, 2016.

#pragma once

#include "../../../ORUtils/Image.h"
#include "../../Utils/ITMMath.h"

namespace ITMLib
{
  /**
   * \brief An instance of this class can be used to hold surfel index images.
   */
  class ITMSurfelRenderState
  {
    //#################### PRIVATE VARIABLES ####################
  private:
    /** The depth buffer for the normal index image. */
    ORUtils::Image<int> *depthBuffer;

    /** The depth buffer for the supersampled index image. */
    ORUtils::Image<int> *depthBufferSuper;

    /** The normal index image. */
    ORUtils::Image<unsigned int> *surfelIndexImage;

    /** The supersampled index image. */
    ORUtils::Image<unsigned int> *surfelIndexImageSuper;

    //#################### CONSTRUCTORS ####################
  public:
    /**
     * \brief Constructs a surfel render state.
     *
     * \param indexImageSize      The size of the normal index image.
     * \param supersamplingFactor The supersampling scaling factor (applied to each axis separately).
     */
    ITMSurfelRenderState(const Vector2i& indexImageSize, int supersamplingFactor);

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
     * \brief Gets the depth buffer for the normal index image.
     *
     * \return  The depth buffer for the normal index image.
     */
    ORUtils::Image<int> *GetDepthBuffer();

    /**
     * \brief Gets the depth buffer for the normal index image.
     *
     * \return  The depth buffer for the normal index image.
     */
    const ORUtils::Image<int> *GetDepthBuffer() const;

    /**
     * \brief Gets the depth buffer for the supersampled index image.
     *
     * \return  The depth buffer for the supersampled index image.
     */
    ORUtils::Image<int> *GetDepthBufferSuper();

    /**
     * \brief Gets the depth buffer for the supersampled index image.
     *
     * \return  The depth buffer for the supersampled index image.
     */
    const ORUtils::Image<int> *GetDepthBufferSuper() const;

    /**
     * \brief Gets the normal index image.
     *
     * \return  The normal index image.
     */
    ORUtils::Image<unsigned int> *GetIndexImage();

    /**
     * \brief Gets the normal index image.
     *
     * \return  The normal index image.
     */
    const ORUtils::Image<unsigned int> *GetIndexImage() const;

    /**
     * \brief Gets the supersampled index image.
     *
     * \return  The supersampled index image.
     */
    ORUtils::Image<unsigned int> *GetIndexImageSuper();

    /**
     * \brief Gets the supersampled index image.
     *
     * \return  The supersampled index image.
     */
    const ORUtils::Image<unsigned int> *GetIndexImageSuper() const;
  };
}
