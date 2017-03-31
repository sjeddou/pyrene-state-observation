# add required dependencies
# Search for dependencies.


# provide path to dependency libraries

INCLUDE_DIRECTORIES(${CMAKE_SOURCE_DIR}/include)
LINK_DIRECTORIES(${${PROJECT_NAME}_BINARY_DIR}/src)


ADD_EXECUTABLE(test-multibody-flexible-dynamical-system test-multibody-flexible-dynamical-system.cpp) # ${${PROJECT_NAME}_ABSOLUTE_HEADERS})
#ADD_EXECUTABLE(test-gaussian-white-noise test-gaussian-white-noise.cpp) # ${${PROJECT_NAME}_ABSOLUTE_HEADERS})

TARGET_LINK_LIBRARIES(test-multibody-flexible-dynamical-system ${Boost_LIBRARIES} ${PROJECT_NAME})
#TARGET_LINK_LIBRARIES(test-gaussian-white-noise ${Boost_LIBRARIES} ${PROJECT_NAME})


ADD_TEST(test-multibody-flexible-dynamical-system test-multibody-flexible-dynamical-system)
#ADD_TEST(test-gaussian-white-noise test-guassian-white-noise)
