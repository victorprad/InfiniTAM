// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM

#pragma once

#include <cassert>

#include "../../../ORUtils/MemoryBlock.h"
#include "../../Utils/ITMSurfelSceneParams.h"

namespace ITMLib
{
  //#################### CONSTANTS ####################

  /** The maximum number of surfels that we can store in a scene. */
  const size_t MAX_SURFEL_COUNT = 5000000;/*67108864;*/ // 2^26

  //#################### TYPES ####################

  /**
   * \brief An instance of an instantiation of this class template represents a surfel-based scene.
   */
  template <typename TSurfel>
  class ITMSurfelScene
  {
    //#################### PRIVATE VARIABLES ####################
  private:
    /** The type of memory in which the scene is stored. */
    MemoryDeviceType m_memoryType;

    /** The scene parameters. */
    const ITMSurfelSceneParams *m_params;

    /** The number of surfels currently in the scene. */
    size_t m_surfelCount;

    /** The surfels in the scene. */
    ORUtils::MemoryBlock<TSurfel> *m_surfelsMB;

    //#################### CONSTRUCTORS ####################
  public:
    /**
     * \brief Constructs a surfel-based scene.
     *
     * \param params      The scene parameters.
     * \param memoryType  The type of memory in which to store the scene.
     */
    ITMSurfelScene(const ITMSurfelSceneParams *params, MemoryDeviceType memoryType)
      : m_memoryType(memoryType),
        m_params(params),
        m_surfelCount(0),
        m_surfelsMB(new ORUtils::MemoryBlock<TSurfel>(MAX_SURFEL_COUNT, true, true))
    {}

    //#################### DESTRUCTOR ####################
  public:
    /**
     * \brief Destroys the scene.
     */
    ~ITMSurfelScene()
    {
      delete m_surfelsMB;
    }

    //#################### COPY CONSTRUCTOR & ASSIGNMENT OPERATOR ####################
  private:
    // Deliberately private and unimplemented
    ITMSurfelScene(const ITMSurfelScene&);
    ITMSurfelScene& operator=(const ITMSurfelScene&);

    //#################### PUBLIC MEMBER FUNCTIONS ####################
  public:
    /**
     * \brief TODO
     */
    TSurfel *AllocateSurfels(size_t newSurfelCount)
    {
      if(m_surfelCount + newSurfelCount > m_surfelsMB->dataSize) return NULL;
      TSurfel *newSurfels = m_surfelsMB->GetData(m_memoryType) + m_surfelCount;
      m_surfelCount += newSurfelCount;
      return newSurfels;
    }

    /**
     * \brief TODO
     */
    void DeallocateRemovedSurfels(size_t removedSurfelCount)
    {
      m_surfelCount -= removedSurfelCount;
    }

    /**
     * \brief Gets the scene parameters.
     *
     * \return  The scene parameters.
     */
    const ITMSurfelSceneParams& GetParams() const
    {
      return *m_params;
    }

    /**
     * \brief Gets the number of surfels currently in the scene.
     *
     * \return  The number of surfels currently in the scene.
     */
    size_t GetSurfelCount() const
    {
      return m_surfelCount;
    }

    /**
     * \brief TODO
     */
    ORUtils::MemoryBlock<TSurfel> *GetSurfels()
    {
      return m_surfelsMB;
    }

    /**
     * \brief TODO
     */
    const ORUtils::MemoryBlock<TSurfel> *GetSurfels() const
    {
      return m_surfelsMB;
    }

    /**
     * \brief Resets the scene.
     */
    void Reset()
    {
      m_surfelCount = 0;
    }
  };
}
