# Tests

# Configure modules path
SET ( MODULE_PATH "${${PROJECT_NAME}_SOURCE_DIR}/cmake" )
INCLUDE ( "${MODULE_PATH}/modules.cmake" )

INCLUDE_DIRECTORIES ( "${${PROJECT_NAME}_SOURCE_DIR}" )
INCLUDE_DIRECTORIES ( "${${PROJECT_NAME}_SOURCE_DIR}/${GOOGLE_TEST_PATH}" )
INCLUDE_DIRECTORIES ( "${${PROJECT_NAME}_SOURCE_DIR}/${GOOGLE_TEST_PATH}/include" )
INCLUDE_DIRECTORIES ( "${${PROJECT_NAME}_SOURCE_DIR}/${SPEL_MODULE}" )
INCLUDE_DIRECTORIES ( "${${PROJECT_NAME}_SOURCE_DIR}/utils/general" )

INCLUDE_DIRECTORIES ( "${${PROJECT_NAME}_SOURCE_DIR}/${TINYXML2_PATH}" )

OPTION ( DISABLE_PTHREADS "Do not use pthreads" OFF )

# Module dependencies
# Ensure that those targets are built before current one
SET ( ${TESTS_MODULE}_DEPENDS "GoogleTestFramework" ${SPEL_MODULE} )

IF ( NOT DISABLE_PTHREADS )
# Defines CMAKE_USE_PTHREADS_INIT and CMAKE_THREAD_LIBS_INIT.
  FIND_PACKAGE ( Threads )
endif()

IF ( CMAKE_USE_PTHREADS_INIT )  # The pthreads library is available and allowed.
  ADD_DEFINITIONS ( -DGTEST_HAS_PTHREAD=1 )
  LIST ( APPEND ${TESTS_MODULE}_LNK ${CMAKE_THREAD_LIBS_INIT} )
else()
  ADD_DEFINITIONS ( -DGTEST_HAS_PTHREAD=0 )
endif()

# main test file
LIST ( APPEND ${TESTS_MODULE}_SRC ${${PROJECT_NAME}_SOURCE_DIR}/${GOOGLE_TEST_PATH}/src/gtest-all test_exec )

# project loader: need for some tests with input data
LIST ( APPEND ${TESTS_MODULE}_SRC ${${PROJECT_NAME}_SOURCE_DIR}/utils/general/projectLoader.cpp )
LIST ( APPEND ${TESTS_MODULE}_SRC ${${PROJECT_NAME}_SOURCE_DIR}/${TINYXML2_PATH}/tinyxml2.cpp )

# libpose tests
LIST ( APPEND ${TESTS_MODULE}_SRC spel/TestsFunctions.cpp )
LIST ( APPEND ${TESTS_MODULE}_SRC spel/bodyJoint_get_and_set_tests.cpp )
LIST ( APPEND ${TESTS_MODULE}_SRC spel/bodyJoint_constructor_tests.cpp )
LIST ( APPEND ${TESTS_MODULE}_SRC spel/bodyJoint_operators_tests.cpp )
LIST ( APPEND ${TESTS_MODULE}_SRC spel/bodyPart_get_and_set_tests.cpp )
LIST ( APPEND ${TESTS_MODULE}_SRC spel/bodyPart_constructor_tests.cpp )
LIST ( APPEND ${TESTS_MODULE}_SRC spel/bodyPart_operators_tests.cpp )
LIST ( APPEND ${TESTS_MODULE}_SRC spel/keyframe_get_and_set_tests.cpp )
LIST ( APPEND ${TESTS_MODULE}_SRC spel/lockframe_get_and_set_tests.cpp )
LIST ( APPEND ${TESTS_MODULE}_SRC spel/interpolation_get_and_set_tests.cpp )
LIST ( APPEND ${TESTS_MODULE}_SRC spel/frame_general_tests.cpp )
LIST ( APPEND ${TESTS_MODULE}_SRC spel/skeleton_get_and_set_tests.cpp )
LIST ( APPEND ${TESTS_MODULE}_SRC spel/skeleton_operators_tests.cpp )
LIST ( APPEND ${TESTS_MODULE}_SRC spel/colorHistDetector_get_and_set_tests.cpp )
LIST ( APPEND ${TESTS_MODULE}_SRC spel/colorHistDetector_private_tests.cpp )
LIST ( APPEND ${TESTS_MODULE}_SRC spel/colorHistDetector_train_tests.cpp )
LIST ( APPEND ${TESTS_MODULE}_SRC spel/colorHistDetector_operators_tests.cpp )
LIST ( APPEND ${TESTS_MODULE}_SRC spel/hogDetector_get_and_set_tests.cpp )
LIST ( APPEND ${TESTS_MODULE}_SRC spel/surfDetector_get_and_set_tests.cpp )
LIST ( APPEND ${TESTS_MODULE}_SRC spel/score_get_and_set_tests.cpp )
LIST ( APPEND ${TESTS_MODULE}_SRC spel/score_constructor_tests.cpp )
LIST ( APPEND ${TESTS_MODULE}_SRC spel/score_operators_tests.cpp )
LIST ( APPEND ${TESTS_MODULE}_SRC spel/spelHelper_poserect_tests.cpp )
LIST ( APPEND ${TESTS_MODULE}_SRC spel/spelHelper_rotatepoint_tests.cpp )
LIST ( APPEND ${TESTS_MODULE}_SRC spel/spelHelper_angle_dist_tests.cpp )
LIST ( APPEND ${TESTS_MODULE}_SRC spel/detector_tests.cpp )
LIST ( APPEND ${TESTS_MODULE}_SRC spel/limbLabel_tests.cpp )
LIST ( APPEND ${TESTS_MODULE}_SRC spel/nskpsolver_tests.cpp )
LIST ( APPEND ${TESTS_MODULE}_SRC spel/frames_tests.cpp )
LIST ( APPEND ${TESTS_MODULE}_SRC spel/hogdetector_tests.cpp )
LIST ( APPEND ${TESTS_MODULE}_SRC spel/ImageSimilarityMatrix_tests.cpp )

LIST ( APPEND ${TESTS_MODULE}_HDR spel/TestsFunctions.hpp )
LIST ( APPEND ${TESTS_MODULE}_HDR ${${PROJECT_NAME}_SOURCE_DIR}/utils/general/projectLoader.hpp )

LIST ( APPEND ${TESTS_MODULE}_LNK ${SPEL_MODULE} )
LIST ( APPEND ${TESTS_MODULE}_LNK ${OpenCV_LIBS} )
LIST ( APPEND ${TESTS_MODULE}_LNK ${HDF5_LIBRARIES} )


ADD_EXECUTABLE ( ${TESTS_MODULE} ${${TESTS_MODULE}_SRC} ${${TESTS_MODULE}_HDR})

# TODO: Library names should be taken from variables
TARGET_LINK_LIBRARIES ( ${TESTS_MODULE} ${${TESTS_MODULE}_LNK} )

ADD_CUSTOM_COMMAND ( TARGET ${TESTS_MODULE} PRE_BUILD COMMAND ${CMAKE_COMMAND} -E copy_directory ${${PROJECT_NAME}_SOURCE_DIR}/tests/ImageRotationTestData $<TARGET_FILE_DIR:${TESTS_MODULE}>/${TESTS_MODULE}_TestData/ImageRotationTestData )
ADD_CUSTOM_COMMAND ( TARGET ${TESTS_MODULE} PRE_BUILD COMMAND ${CMAKE_COMMAND} -E copy_directory ${${PROJECT_NAME}_SOURCE_DIR}/tests/CHDTrainTestData $<TARGET_FILE_DIR:${TESTS_MODULE}>/${TESTS_MODULE}_TestData/CHDTrainTestData )
ADD_CUSTOM_COMMAND ( TARGET ${TESTS_MODULE} PRE_BUILD COMMAND ${CMAKE_COMMAND} -E copy_directory ${${PROJECT_NAME}_SOURCE_DIR}/tests/SimilarityMatrixTestsData $<TARGET_FILE_DIR:${TESTS_MODULE}>/${TESTS_MODULE}_TestData/SimilarityMatrixTestsData )

# Installation instructions for the test module
# Please specify the installation instructions sprcific
# To the architecture the project is being built for.

IF ( UNIX )
    INSTALL (
            TARGETS ${TESTS_MODULE}
            RUNTIME DESTINATION bin/
            LIBRARY DESTINATION lib/${PROJECT_NAME}
            ARCHIVE DESTINATION lib/static/${PROJECT_NAME}
    )
ELSEIF ( WIN32 )
    INSTALL (
            TARGETS ${TESTS_MODULE}
            RUNTIME DESTINATION bin/
            LIBRARY DESTINATION lib/
            ARCHIVE DESTINATION lib/static/
    )
ENDIF()

