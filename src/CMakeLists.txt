CMAKE_MINIMUM_REQUIRED ( VERSION  2.6 )
INCLUDE ( CheckCXXCompilerFlag )

# Project information and versioning
PROJECT ( spel )

# CMake compatibility issues: don't modify this, please!
MARK_AS_ADVANCED ( CMAKE_BACKWARDS_COMPATIBILITY )
SET ( CMAKE_ALLOW_LOOSE_LOOP_CONSTRUCTS TRUE )
SET ( CMAKE_VERBOSE_MAKEFILE ON )

# Enable c++11 in gcc
IF ( CMAKE_COMPILER_IS_GNUCXX )
  EXECUTE_PROCESS ( COMMAND ${CMAKE_CXX_COMPILER} -dumpversion OUTPUT_VARIABLE GCC_VERSION )
  IF ( GCC_VERSION VERSION_GREATER 4.7 OR GCC_VERSION VERSION_EQUAL 4.7 )
    SET ( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11" )
  ELSEIF (GCC_VERSION VERSION_GREATER 4.6 OR GCC_VERSION VERSION_EQUAL 4.6 )
    SET ( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x" )
  ENDIF()
ENDIF()

IF ( UNIX AND NOT CMAKE_COMPILER_IS_GNUCXX )
  CHECK_CXX_COMPILER_FLAG ( "-std=c++11" COMPILER_SUPPORTS_CXX11 )
  CHECK_CXX_COMPILER_FLAG ( "-std=c++0x" COMPILER_SUPPORTS_CXX0X )
  IF ( COMPILER_SUPPORTS_CXX11 )
    SET ( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11" )
  ELSEIF ( COMPILER_SUPPORTS_CXX0X )
    SET ( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x" )
  ELSE ()
    MESSAGE ( STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler." )
  ENDIF ()
ENDIF ()

# Show all warnings
IF ( MSVC )
  # Force to always compile with W4
  IF ( CMAKE_CXX_FLAGS MATCHES "/W[0-4]" )
    STRING ( REGEX REPLACE "/W[0-4]" "/W4" CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}" )
  ELSE ()
    SET ( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /W4" )
  ENDIF ()
ELSEIF ( CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX )
  # Update if necessary
  SET ( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wno-long-long -pedantic" )
ENDIF ()

SET ( CMAKE_MODULE_PATH "${${PROJECT_NAME}_SOURCE_DIR}/cmake" )

# find common packages
# use OpenGM
FIND_PACKAGE ( OpenGM REQUIRED )
INCLUDE_DIRECTORIES ( ${OpenGM_INCLUDE_DIR} )

# use OpenCV
FIND_PACKAGE ( OpenCV REQUIRED )
ADD_DEFINITIONS ( -DOpenCV_VERSION_MAJOR=${OpenCV_VERSION_MAJOR} )
ADD_DEFINITIONS ( -DOpenCV_VERSION_MINOR=${OpenCV_VERSION_MINOR} )
ADD_DEFINITIONS ( -DOpenCV_VERSION_PATCH=${OpenCV_VERSION_PATCH} )
ADD_DEFINITIONS ( -DOpenCV_VERSION_TWEAK=${OpenCV_VERSION_TWEAK} )

INCLUDE_DIRECTORIES ( ${OpenCV_INCLUDE_DIRS} )

# use tree.hh
INCLUDE_DIRECTORIES ( "${${PROJECT_NAME}_SOURCE_DIR}/../thirdparty/tree/src" )

# use Eigen3
FIND_PACKAGE ( Eigen3 REQUIRED )
INCLUDE_DIRECTORIES (${Eigen3_INCLUDE_DIR} )

# use HDF5
IF ( WIN32 )
# FindHDF5 is broken on Windows
# it wrongly discovers the dlls instead of the lib files
# see: www.cmake.org/Bug/bug_relationship_graph.php?bug_id=14111
# therefore we set the variable manually
  FIND_LIBRARY ( HDF5_IMPORT_LIB NAMES hdf5 hdf5dll )
  FIND_LIBRARY ( HDF5_HL_IMPORT_LIB NAMES hdf5_hl hdf5_hldll )
  FIND_PATH ( HDF5_INCLUDE_DIR hdf5.h )
  SET ( HDF5_LIBRARIES ${HDF5_IMPORT_LIB} ${HDF5_HL_IMPORT_LIB} )
  MESSAGE ( STATUS "HDF5: autodiscovery is broken on WIN32; using global search paths")
  MESSAGE ( STATUS "HDF5: found ${HDF5_LIBRARIES}")
ELSE ()
  FIND_PACKAGE ( HDF5 REQUIRED )
ENDIF ()
INCLUDE_DIRECTORIES ( ${HDF5_INCLUDE_DIR} )
ADD_DEFINITIONS ( ${HDF5_CPPFLAGS} )

# spel source
INCLUDE_DIRECTORIES ( "${${PROJECT_NAME}_SOURCE_DIR}" )


# Command line options, their defaults and checks


# Project configuration types
SET ( CMAKE_CONFIGURATION_TYPES Release; Debug )

# set default cmake build type to Debug
IF ( NOT CMAKE_BUILD_TYPE )
    SET ( CMAKE_BUILD_TYPE Debug )
ENDIF()

SET ( GOOGLE_TEST_PATH "../thirdparty/googletest" CACHE STRING "Path to Google Test Framework" )

IF ( CMAKE_BUILD_TYPE STREQUAL "Debug" )
  OPTION ( GOOGLE_TEST "Build tests with Google Test Framework" ON )
ELSE ()
  OPTION ( GOOGLE_TEST "Build tests with Google Test Framework" OFF )
ENDIF ()

# Nessessary definitions

IF ( UNIX )
  ADD_DEFINITIONS ( "-DUNIX" )
ELSEIF( WIN32 )
  ADD_DEFINITIONS ( "-DWINDOWS" )
ENDIF()

IF ( CMAKE_BUILD_TYPE STREQUAL "Debug" )
  ADD_DEFINITIONS ( "-DDEBUG" )
ENDIF()

IF ( NOT GOOGLE_TEST )
  SET ( GOOGLE_TEST:BOOL OFF )
ENDIF()

SET ( TINYXML2_PATH "../thirdparty/tinyxml2" CACHE STRING "Path to TinyXml2 library" )
IF ( NOT TINYXML2_PATH )
  MESSAGE ( FATAL_ERROR "TimyXml was not found" )
ENDIF()

# Recurse into subdirectory
# Compile Google Test Framework
IF ( CMAKE_BUILD_TYPE STREQUAL "Debug" )
  IF ( GOOGLE_TEST AND GOOGLE_TEST_PATH )
    ADD_DEFINITIONS ( "-DPOSE_TEST" )
    IF ( MSVC )
      ADD_DEFINITIONS ( -D_VARIADIC_MAX=10 )
    ENDIF ()
  # If we are using gcc under linux in debug mode - lets add code coverage test
  #  IF ( CMAKE_BUILD_TYPE STREQUAL "Debug" )
  #    IF ( UNIX )
  #      IF ( CMAKE_COMPILER_IS_GNUCXX )
  #        SET ( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fprofile-arcs -ftest-coverage -fPIC -O0" )
  #      ENDIF()
  #    ENDIF()
  #  ENDIF()
    ADD_SUBDIRECTORY ( tests )
  # custom target
    ADD_CUSTOM_TARGET ( tests DEPENDS ${SPEL_MODULE} ${TESTS_MODULE} )

  ENDIF()
ENDIF()

ADD_SUBDIRECTORY ( spel )
ADD_SUBDIRECTORY ( utils )

# Check for appropriate build parameters
# If ok BUILD_OK is set to ON
SET ( BUILD_OK:BOOL OFF )
IF ( DEFINED CMAKE_BUILD_TYPE )
	FOREACH ( CFG ${CMAKE_CONFIGURATION_TYPES} )
		IF ( CMAKE_BUILD_TYPE STREQUAL CFG )
			SET ( BUILD_OK ON )
			BREAK()
		ENDIF()
	ENDFOREACH()
	IF ( NOT BUILD_OK )
		MESSAGE ( FATAL_ERROR "Wrong build type parameter. Possible values for build type are: ${CMAKE_CONFIGURATION_TYPES}. Please choose one of those" )
	ENDIF()
ELSE()
	MESSAGE ( FATAL_ERROR "Build type is not defined. Possible values are: ${CMAKE_CONFIGURATION_TYPES}. Please specify one" )
ENDIF()

# Cache some of the command-line parameters
SET ( CMAKE_BUILD_TYPE "${CMAKE_BUILD_TYPE}" CACHE STRING "The build type for the project" FORCE )
SET ( GOOGLE_TEST_PATH "../thirdparty/googletest" CACHE STRING "Path to Google Test Framework" )
SET ( TINYXML2_PATH "../thirdparty/tinyxml2" CACHE STRING "Path to TinyXml2 library" )
