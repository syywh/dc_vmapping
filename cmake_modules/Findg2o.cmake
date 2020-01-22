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

  IF(libg2o_INCLUDE_DIR AND libg2o_LIBRARIES)
    # in cache already
    SET(libg2o_FIND_QUIETLY TRUE)
  ENDIF(libg2o_INCLUDE_DIR AND libg2o_LIBRARIES)

  MESSAGE(STATUS "Searching for g2o ...")
  FIND_PATH(libg2o_INCLUDE_DIR
    NAMES core math_groups types
    PATHS /opt/ros/kinetic/
    PATH_SUFFIXES include/g2o include)

  IF (libg2o_INCLUDE_DIR)
    MESSAGE(STATUS "Found g2o headers in: ${libg2o_INCLUDE_DIR}")
  ENDIF (libg2o_INCLUDE_DIR)

  FIND_LIBRARY(libg2o_CORE_LIB             
    NAMES g2o_core 
    PATHS /opt/ros/kinetic /usr 
    PATH_SUFFIXES lib)
  FIND_LIBRARY(libg2o_STUFF_LIB            
    NAMES g2o_stuff 
    PATHS /opt/ros/kinetic /usr 
    PATH_SUFFIXES lib)
  FIND_LIBRARY(libg2o_TYPES_SLAM3D_LIB     
    NAMES g2o_types_slam3d 
    PATHS /opt/ros/kinetic /usr 
    PATH_SUFFIXES lib)
  FIND_LIBRARY(libg2o_TYPES_SBA_LIB     
    NAMES g2o_types_sba 
    PATHS /opt/ros/kinetic /usr 
    PATH_SUFFIXES lib)
  FIND_LIBRARY(libg2o_SOLVER_CHOLMOD_LIB   
    NAMES g2o_solver_cholmod 
    PATHS /opt/ros/kinetic /usr 
    PATH_SUFFIXES lib)
  FIND_LIBRARY(libg2o_SOLVER_PCG_LIB       
    NAMES g2o_solver_pcg 
    PATHS /opt/ros/kinetic /usr 
    PATH_SUFFIXES lib)
  FIND_LIBRARY(libg2o_SOLVER_CSPARSE_LIB   
    NAMES g2o_solver_csparse 
    PATHS /opt/ros/kinetic /usr 
    PATH_SUFFIXES lib)
  FIND_LIBRARY(libg2o_INCREMENTAL_LIB      
    NAMES g2o_incremental 
    PATHS /opt/ros/kinetic /usr 
    PATH_SUFFIXES lib)
  FIND_LIBRARY(libg2o_CSPARSE_EXTENSION_LIB
    NAMES g2o_csparse_extension
    PATHS /opt/ros/kinetic /usr
    PATH_SUFFIXES lib)

  SET(libg2o_LIBRARIES ${libg2o_CSPARSE_EXTENSION_LIB}
                    ${libg2o_CORE_LIB}           
                    ${libg2o_STUFF_LIB}          
                    ${libg2o_TYPES_SLAM3D_LIB}   
                    ${libg2o_SOLVER_CHOLMOD_LIB} 
                    ${libg2o_SOLVER_PCG_LIB}     
                    ${libg2o_SOLVER_CSPARSE_LIB} 
                    ${libg2o_INCREMENTAL_LIB}  
					${libg2o_TYPES_SBA_LIB}                      
                    )

  IF(libg2o_LIBRARIES AND libg2o_INCLUDE_DIR)
    SET(libg2o_FOUND "YES")
    IF(NOT libg2o_FIND_QUIETLY)
      MESSAGE(STATUS "Found libg2o: ${libg2o_LIBRARIES}")
    ENDIF(NOT libg2o_FIND_QUIETLY)
  ELSE(libg2o_LIBRARIES AND libg2o_INCLUDE_DIR)
    IF(NOT libg2o_LIBRARIES)
      IF(libg2o_FIND_REQUIRED)
        message(FATAL_ERROR "Could not find libg2o!")
      ENDIF(libg2o_FIND_REQUIRED)
    ENDIF(NOT libg2o_LIBRARIES)

    IF(NOT libg2o_INCLUDE_DIR)
      IF(libg2o_FIND_REQUIRED)
        message(FATAL_ERROR "Could not find g2o include directory!")
      ENDIF(libg2o_FIND_REQUIRED)
    ENDIF(NOT libg2o_INCLUDE_DIR)
  ENDIF(libg2o_LIBRARIES AND libg2o_INCLUDE_DIR)

ENDIF(UNIX)
