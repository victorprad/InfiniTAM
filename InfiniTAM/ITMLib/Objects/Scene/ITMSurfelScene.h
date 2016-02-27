// InfiniTAM: Surffuse. Copyright (c) Torr Vision Group and the authors of InfiniTAM, 2016.

#pragma once

#include <cassert>

#include "../../../ORUtils/MemoryBlock.h"
#include "../../Utils/ITMSurfelSceneParams.h"

namespace ITMLib
{
  //#################### CONSTANTS ####################

  /** The maximum number of surfels that we can store in a scene. */
  const size_t MAX_SURFEL_COUNT = 5000000;

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
    // Deliberately private and unimplemented.
    ITMSurfelScene(const ITMSurfelScene&);
    ITMSurfelScene& operator=(const ITMSurfelScene&);

    //#################### PUBLIC MEMBER FUNCTIONS ####################
  public:
    /**
     * \brief Allocates a contiguous block of memory to store the specified number of new surfels.
     *
     * \param newSurfelCount  The number of new surfels for which to allocate space.
     * \return                A pointer to the start of the allocated memory.
     */
    TSurfel *AllocateSurfels(size_t newSurfelCount)
    {
      if(m_surfelCount + newSurfelCount > m_surfelsMB->dataSize) return NULL;
      TSurfel *newSurfels = m_surfelsMB->GetData(m_memoryType) + m_surfelCount;
      m_surfelCount += newSurfelCount;
      return newSurfels;
    }

    /**
     * \brief Deallocates the specified number of "removed" surfels.
     *
     * Surfel removal is implemented by moving the surfels to remove to the end of the surfel array.
     * Deallocation thus simply involves decreasing our count of the number of surfels allocated.
     *
     * \param removedSurfelCount  The number of "removed" surfels that should be deallocated.
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
     * \brief Gets the memory block containing the surfels in the scene.
     *
     * \return  The memory block containing the surfels in the scene.
     */
    ORUtils::MemoryBlock<TSurfel> *GetSurfels()
    {
      return m_surfelsMB;
    }

    /**
     * \brief Gets the memory block containing the surfels in the scene.
     *
     * \return  The memory block containing the surfels in the scene.
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
