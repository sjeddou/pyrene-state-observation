// Copyright (C) 2008-2014 LAAS-CNRS, JRL AIST-CNRS.
//
// This file is part of jrl-cmakemodules.
// jrl-mathtools is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// jrl-mathtools is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
// You should have received a copy of the GNU Lesser General Public License
// along with jrl-mathtools.  If not, see <http://www.gnu.org/licenses/>.

#ifndef PYRENE_STATE_OBSERVATION_CONFIG_HH
# define PYRENE_STATE_OBSERVATION_CONFIG_HH

// Package version (header).
# define PYRENE_STATE_OBSERVATION_VERSION "0.0-5-gd8ae-dirty"

// Handle portable symbol export.
// Defining manually which symbol should be exported is required
// under Windows whether MinGW or MSVC is used.
//
// The headers then have to be able to work in two different modes:
// - dllexport when one is building the library,
// - dllimport for clients using the library.
//
// On Linux, set the visibility accordingly. If C++ symbol visibility
// is handled by the compiler, see: http://gcc.gnu.org/wiki/Visibility
# if defined _WIN32 || defined __CYGWIN__
// On Microsoft Windows, use dllimport and dllexport to tag symbols.
#  define PYRENE_STATE_OBSERVATION_DLLIMPORT __declspec(dllimport)
#  define PYRENE_STATE_OBSERVATION_DLLEXPORT __declspec(dllexport)
#  define PYRENE_STATE_OBSERVATION_DLLLOCAL
# else
// On Linux, for GCC >= 4, tag symbols using GCC extension.
#  if __GNUC__ >= 4
#   define PYRENE_STATE_OBSERVATION_DLLIMPORT __attribute__ ((visibility("default")))
#   define PYRENE_STATE_OBSERVATION_DLLEXPORT __attribute__ ((visibility("default")))
#   define PYRENE_STATE_OBSERVATION_DLLLOCAL  __attribute__ ((visibility("hidden")))
#  else
// Otherwise (GCC < 4 or another compiler is used), export everything.
#   define PYRENE_STATE_OBSERVATION_DLLIMPORT
#   define PYRENE_STATE_OBSERVATION_DLLEXPORT
#   define PYRENE_STATE_OBSERVATION_DLLLOCAL
#  endif // __GNUC__ >= 4
# endif // defined _WIN32 || defined __CYGWIN__

# ifdef PYRENE_STATE_OBSERVATION_STATIC
// If one is using the library statically, get rid of
// extra information.
#  define PYRENE_STATE_OBSERVATION_DLLAPI
#  define PYRENE_STATE_OBSERVATION_LOCAL
# else
// Depending on whether one is building or using the
// library define DLLAPI to import or export.
#  ifdef pyrene_state_observation_EXPORTS
#   define PYRENE_STATE_OBSERVATION_DLLAPI PYRENE_STATE_OBSERVATION_DLLEXPORT
#  else
#   define PYRENE_STATE_OBSERVATION_DLLAPI PYRENE_STATE_OBSERVATION_DLLIMPORT
#  endif // PYRENE_STATE_OBSERVATION_EXPORTS
#  define PYRENE_STATE_OBSERVATION_LOCAL PYRENE_STATE_OBSERVATION_DLLLOCAL
# endif // PYRENE_STATE_OBSERVATION_STATIC
#endif //! PYRENE_STATE_OBSERVATION_CONFIG_HH
