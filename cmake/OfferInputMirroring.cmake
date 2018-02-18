#############################
# OfferInputMirroring.cmake #
#############################

OPTION(USE_INPUT_MIRRORING "Use input mirroring?" OFF)

IF(USE_INPUT_MIRRORING)
  ADD_DEFINITIONS(-DUSE_INPUT_MIRRORING)
ENDIF()
