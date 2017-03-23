
# provide path to dependency libraries

INCLUDE_DIRECTORIES(${CMAKE_SOURCE_DIR}/include)
LINK_DIRECTORIES(${${PROJECT_NAME}_BINARY_DIR}/src)

#ADD_EXECUTABLE(test-mutlibody-flexible-dynamical-system test-mutlibody-flexible-dynamical-system.cpp)  #${${PROJECT_NAME}_ABSOLUTE_HEADERS})

#TARGET_LINK_LIBRARIES(test-mutlibody-flexible-dynamical-system ${Boost_LIBRARIES} ${PROJECT_NAME})


#ADD_TEST(test-mutlibody-flexible-dynamical-system test-mutlibody-flexible-dynamical-system)



