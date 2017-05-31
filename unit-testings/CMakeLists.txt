# Unit Testing CMakeLists


ADD_DEFINITIONS(-DDEBUG=2)

macro(ADDPREFIX newlist prefix list_name)
    # create empty list - necessary?
    SET(${newlist})

     # prefix and suffix elements
    foreach(l ${${list_name}})
      list(APPEND ${newlist} ${prefix}${l} )
    endforeach()

endmacro(ADDPREFIX)

ADDPREFIX(${PROJECT_NAME}_ABSOLUTE_HEADERS  "${CMAKE_SOURCE_DIR}/"  ${PROJECT_NAME}_HEADERS)

SET(${PROJECT_NAME}_test_CXXFLAGS "-I${PROJECT_SOURCE_DIR}/include/ ${${PROJECT_NAME}_CXXFLAGS}")

include_directories(${Boost_INCLUDE_DIR} ${PROJECT_INCLUDE_DIR})

SET(tests
    test-multibody-flexible-dynamical-system
    test-gaussian-white-noise
  )

FOREACH(test ${tests})
  SET(EXECUTABLE_NAME "${test}_exe")
  ADD_EXECUTABLE(${EXECUTABLE_NAME}
    ${test}.cpp ${${PROJECT_NAME}_ABSOLUTE_HEADERS})

  
  
  PKG_CONFIG_USE_DEPENDENCY(${EXECUTABLE_NAME} state-observation)


  TARGET_LINK_LIBRARIES(${EXECUTABLE_NAME} ${Boost_LIBRARIES} ${PROJECT_NAME} state-observation)

  ADD_TEST(${EXECUTABLE_NAME} ${EXECUTABLE_NAME})

ENDFOREACH(test)

SET(execs
  )

FOREACH(exec ${execs})
  SET(EXECUTABLE_NAME "${test}_exe")
  ADD_EXECUTABLE(${EXECUTABLE_NAME}
    ${test}.cpp ${${PROJECT_NAME}_ABSOLUTE_HEADERS})

  #TARGET_LINK_LIBRARIES(${EXECUTABLE_NAME}
    #controller-pd
   # )

  PKG_CONFIG_USE_DEPENDENCY(${EXECUTABLE_NAME} state-observation)
 

  TARGET_LINK_LIBRARIES(${EXECUTABLE_NAME} ${Boost_LIBRARIES} ${PROJECT_NAME} state-observation)
ENDFOREACH(exec)

