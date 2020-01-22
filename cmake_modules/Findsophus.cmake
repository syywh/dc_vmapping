# Locate the g2o libraries
# A general framework for graph optimization.
#
# This module defines
# libg2o_FOUND, if false, do not try to link against g2o
# libg2o_LIBRARIES, path to the libg2o
# libg2o_INCLUDE_DIR, where to find the g2o header files
#
# Niko Suenderhauf <niko@etit.tu-chemnitz.de>
# Adapted by Felix Endres <endres@informatik.uni-freiburg.de>

IF(UNIX)

  IF(libsophus_INCLUDE_DIR AND libsophus_LIBRARIES)
    # in cache already
    SET(libg2o_FIND_QUIETLY TRUE)
  ENDIF(libsophus_INCLUDE_DIR AND libsophus_LIBRARIES)

  MESSAGE(STATUS "Searching for g2o ...")
  FIND_PATH(libsophus_INCLUDE_DIR
    NAMES core math_groups types
    PATHS /opt/ros/kinetic/
    PATH_SUFFIXES include/g2o include)

  IF (libsophus_INCLUDE_DIR)
    MESSAGE(STATUS "Found sophus headers in: ${libsophus_INCLUDE_DIR}")
  ENDIF (libsophus_INCLUDE_DIR)

 
  IF(libsophus_INCLUDE_DIR)
    SET(libsophus_FOUND "YES")
    IF(NOT libsophus_FIND_QUIETLY)
      MESSAGE(STATUS "Found libsophus: ${libsophus_LIBRARIES}")
    ENDIF(NOT libsophus_FIND_QUIETLY)
  ELSE(libsophus_LIBRARIES AND libsophus_INCLUDE_DIR)
    IF(NOT libsophus_LIBRARIES)
      IF(libsophus_FIND_REQUIRED)
        message(FATAL_ERROR "Could not find libsophus!")
      ENDIF(libsophus_FIND_REQUIRED)
    ENDIF(NOT libsophus_LIBRARIES)
  ENDIF( libg2o_INCLUDE_DIR)


ENDIF(UNIX)
