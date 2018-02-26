#
# Try to find the FreeImage library and include path.
# Once done this will define
#
# FREEIMAGE_FOUND
# FREEIMAGE_INCLUDE_PATH
# FREEIMAGE_LIBRARY
# FREEIMAGE_STATIC_LIBRARY
# FREEIMAGE_DYNAMIC_LIBRARY
#

OPTION(USE_FreeImage_STATIC "Use Static FreeImage Lib" OFF)

FIND_PATH(FreeImage_INCLUDE_PATH
    NAMES FreeImage.h
    PATHS
    /usr/include
    /usr/local/include
    /sw/include
    /opt/local/include
    DOC "The directory where FreeImage.h resides")

FIND_LIBRARY(FreeImage_DYNAMIC_LIBRARY
    NAMES FreeImage freeimage
    PATHS
    /usr/lib64
    /usr/lib
    /usr/local/lib64
    /usr/local/lib
    /sw/lib
    /opt/local/lib
    DOC "The FreeImage library")

SET(PX ${CMAKE_STATIC_LIBRARY_PREFIX})
SET(SX ${CMAKE_STATIC_LIBRARY_SUFFIX})

FIND_LIBRARY(FreeImage_STATIC_LIBRARY
    NAMES ${PX}FreeImageLIB${SX} ${PX}FreeImage${SX} ${PX}freeimage${SX}
    HINTS ${PROJECT_SOURCE_DIR}/FreeImage
    PATHS
    /usr/lib64
    /usr/lib
    /usr/local/lib64
    /usr/local/lib
    /sw/lib
    /opt/local/lib
    DOC "The FreeImage library")
  
UNSET(PX)
UNSET(SX)

IF(USE_FreeImage_STATIC)
  
  IF(FreeImage_STATIC_LIBRARY)
    ADD_DEFINITIONS(-D$FREEIMAGE_LIB)
    SET(FreeImage_LIBRARY ${FreeImage_STATIC_LIBRARY})
    MESSAGE(STATUS "Using Static FreeImage Lib FreeImage_LIBRARY = " ${FreeImage_LIBRARY})
  else(FreeImage_STATIC_LIBRARY)  
    if(FreeImage_FIND_REQUIRED AND NOT FreeImage_FIND_QUIETLY)
      message(FATAL_ERROR "Could not find FREEIMAGE static libs ")
    endif(FreeImage_FIND_REQUIRED AND NOT FreeImage_FIND_QUIETLY)
  endif(FreeImage_STATIC_LIBRARY)
  

ELSE(USE_FreeImage_STATIC)

  IF(FreeImage_DYNAMIC_LIBRARY)
    REMOVE_DEFINITIONS(-DFREEIMAGE_LIB)
    SET(FreeImage_LIBRARY ${FreeImage_DYNAMIC_LIBRARY})
    MESSAGE(STATUS "Using Dynamic FreeImage Lib FreeImage_LIBRARY = " ${FreeImage_LIBRARY})
  else(FreeImage_DYNAMIC_LIBRARY)  
    if(FreeImage_FIND_REQUIRED AND NOT FreeImage_FIND_QUIETLY)
      message(FATAL_ERROR "Could not find FREEIMAGE dynamic libs ")
    endif(FreeImage_FIND_REQUIRED AND NOT FreeImage_FIND_QUIETLY)
  endif(FreeImage_DYNAMIC_LIBRARY)

ENDIF(USE_FreeImage_STATIC)


IF(FreeImage_INCLUDE_PATH)
    MESSAGE(STATUS "Using FreeImage include dir") 
    SET(FreeImage_INCLUDE_DIRS ${FreeImage_INCLUDE_PATH})   
else(FreeImage_INCLUDE_PATH)  
  if(FreeImage_FIND_REQUIRED AND NOT FreeImage_FIND_QUIETLY)
    message(FATAL_ERROR "Could not find FREEIMAGE include directory")
  endif(FreeImage_FIND_REQUIRED AND NOT FreeImage_FIND_QUIETLY)
endif(FreeImage_INCLUDE_PATH)



INCLUDE(FindPackageHandleStandardArgs)
  
FIND_PACKAGE_HANDLE_STANDARD_ARGS(FREEIMAGE DEFAULT_MSG
  FreeImage_INCLUDE_DIRS FreeImage_LIBRARY)





MARK_AS_ADVANCED(
    FreeImage_INCLUDE_DIRS
    FreeImage_DYNAMIC_LIBRARY
    FreeImage_STATIC_LIBRARY
    FreeImage_LIBRARY
    )