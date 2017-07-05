# Copyright 2013, CNRS
#
# Author: Mehdi Benallegue
#
# This file is part of state-observation.
# state-observation is free software: you can redistribute it and/or
# modify it under the terms of the GNU Lesser General Public License
# as published by the Free Software Foundation, either version 3 of
# the License, or (at your option) any later version.
#
# state-observation is distributed in the hope that it will be useful, but
# WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
# General Lesser Public License for more details.  You should have
# received a copy of the GNU Lesser General Public License along with
# state-observation. If not, see <http://www.gnu.org/licenses/>.

CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

INCLUDE(cmake/base.cmake)
INCLUDE(cmake/boost.cmake)
INCLUDE(cmake/eigen.cmake)

SET(PROJECT_NAME pyrene-state-observation)
SET(PROJECT_DESCRIPTION "General implementation of observers.")
SET(PROJECT_URL "")

SET(CUSTOM_HEADER_DIR "${PROJECT_NAME}")

# Search for dependencies.



SET(CXX_DISABLE_WERROR TRUE)

SETUP_PROJECT()

SEARCH_FOR_BOOST()
SEARCH_FOR_EIGEN()

ADD_REQUIRED_DEPENDENCY("state-observation")
ADD_REQUIRED_DEPENDENCY("pinocchio")

SET(${PROJECT_NAME}_HEADERS

  include/pyrene-state-observation/pyrene-flexibility-estimation/multibody-flexible-dynamical-system.hpp
  include/pyrene-state-observation/tools/pyrene.hpp
  include/pyrene-state-observation/pyrene-flexibility-estimation/multibody-ekf-flexibility-estimator.hpp
  include/pyrene-state-observation/pyrene-flexibility-estimation/model-base-ekf-flex-estimator-imu.hpp
)

PKG_CONFIG_APPEND_LIBS(${PROJECT_NAME})

ADD_SUBDIRECTORY(src)
ADD_SUBDIRECTORY(unit-testings)
SETUP_PROJECT_FINALIZE()
