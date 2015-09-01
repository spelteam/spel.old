# spel module
# Creates a spel static library 

# spel module parameters

# Configure modules path
SET ( MODULE_PATH "${${PROJECT_NAME}_SOURCE_DIR}/cmake" )
INCLUDE ( "${MODULE_PATH}/modules.cmake" )

INCLUDE_DIRECTORIES ( "${${PROJECT_NAME}_SOURCE_DIR}" )
IF ( GOOGLE_TEST AND GOOGLE_TEST_PATH )
  INCLUDE_DIRECTORIES ( "${${PROJECT_NAME}_SOURCE_DIR}/${GOOGLE_TEST_PATH}/include" )
ENDIF()

# Command line options, their defaults and checks

# Set correct defines and include correct files
LIST ( APPEND ${SPEL_MODULE}_SRC skeleton.cpp )
LIST ( APPEND ${SPEL_MODULE}_SRC limbLabel.cpp )
LIST ( APPEND ${SPEL_MODULE}_SRC frame.cpp )
LIST ( APPEND ${SPEL_MODULE}_SRC colorHistDetector.cpp )
LIST ( APPEND ${SPEL_MODULE}_SRC surfDetector.cpp )
LIST ( APPEND ${SPEL_MODULE}_SRC hogDetector.cpp )
LIST ( APPEND ${SPEL_MODULE}_SRC score.cpp )
LIST ( APPEND ${SPEL_MODULE}_SRC bodyJoint.cpp )
LIST ( APPEND ${SPEL_MODULE}_SRC bodyPart.cpp )
LIST ( APPEND ${SPEL_MODULE}_SRC keyframe.cpp )
LIST ( APPEND ${SPEL_MODULE}_SRC interpolation.cpp )
LIST ( APPEND ${SPEL_MODULE}_SRC lockframe.cpp )
LIST ( APPEND ${SPEL_MODULE}_SRC solvlet.cpp )
LIST ( APPEND ${SPEL_MODULE}_SRC solver.cpp )
LIST ( APPEND ${SPEL_MODULE}_SRC spelHelper.cpp )
LIST ( APPEND ${SPEL_MODULE}_SRC imagesimilaritymatrix.cpp )
LIST ( APPEND ${SPEL_MODULE}_SRC minspanningtree.cpp )
LIST ( APPEND ${SPEL_MODULE}_SRC nskpsolver.cpp )
LIST ( APPEND ${SPEL_MODULE}_SRC tlpssolver.cpp )
LIST ( APPEND ${SPEL_MODULE}_SRC detector.cpp )
LIST ( APPEND ${SPEL_MODULE}_SRC sequence.cpp )

LIST ( APPEND ${SPEL_MODULE}_HDR bodyJoint.hpp )
LIST ( APPEND ${SPEL_MODULE}_HDR bodyPart.hpp )
LIST ( APPEND ${SPEL_MODULE}_HDR colorHistDetector.hpp )
LIST ( APPEND ${SPEL_MODULE}_HDR detector.hpp )
LIST ( APPEND ${SPEL_MODULE}_HDR frame.hpp )
LIST ( APPEND ${SPEL_MODULE}_HDR hogDetector.hpp )
LIST ( APPEND ${SPEL_MODULE}_HDR imagesimilaritymatrix.hpp )
LIST ( APPEND ${SPEL_MODULE}_HDR interpolation.hpp )
LIST ( APPEND ${SPEL_MODULE}_HDR keyframe.hpp )
LIST ( APPEND ${SPEL_MODULE}_HDR limbLabel.hpp )
LIST ( APPEND ${SPEL_MODULE}_HDR lockframe.hpp )
LIST ( APPEND ${SPEL_MODULE}_HDR minspanningtree.hpp )
LIST ( APPEND ${SPEL_MODULE}_HDR nskpsolver.hpp )
LIST ( APPEND ${SPEL_MODULE}_HDR spelHelper.hpp )
LIST ( APPEND ${SPEL_MODULE}_HDR predef.hpp )
LIST ( APPEND ${SPEL_MODULE}_HDR score.hpp )
LIST ( APPEND ${SPEL_MODULE}_HDR sequence.hpp )
LIST ( APPEND ${SPEL_MODULE}_HDR skeleton.hpp )
LIST ( APPEND ${SPEL_MODULE}_HDR solver.hpp )
LIST ( APPEND ${SPEL_MODULE}_HDR solvlet.hpp )
LIST ( APPEND ${SPEL_MODULE}_HDR spel.hpp )
LIST ( APPEND ${SPEL_MODULE}_HDR surfDetector.hpp )
LIST ( APPEND ${SPEL_MODULE}_HDR tlpssolver.hpp )

# Create a static library
ADD_LIBRARY ( ${SPEL_MODULE} STATIC ${${SPEL_MODULE}_SRC} ${${SPEL_MODULE}_HDR} )

# Installation instructions for the spel module
# Please specify the installation instructions sprcific 
# To the architecture the project is being built for.

IF ( UNIX )
	INSTALL (
		TARGETS ${SPEL_MODULE}
		RUNTIME DESTINATION bin/
		LIBRARY DESTINATION lib/${PROJECT_NAME}
		ARCHIVE DESTINATION lib/static/${PROJECT_NAME}
	)
  INSTALL (
    DIRECTORY ./ DESTINATION include/${PROJECT_NAME} FILES_MATCHING PATTERN "*.hpp"
  )
ELSEIF ( WIN32 )
	INSTALL (
		TARGETS ${SPEL_MODULE}
		RUNTIME DESTINATION bin/
		LIBRARY DESTINATION lib/
		ARCHIVE DESTINATION lib/static/
	)
  INSTALL (
    DIRECTORY ./ DESTINATION include/ FILES_MATCHING PATTERN "*.hpp"
  )
ENDIF()

