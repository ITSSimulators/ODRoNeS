# 
#  This file is part of the ODRoNeS (OpenDRIVE Road Network System) package.
#  
#  Copyright (c) 2023 Albert Solernou, University of Leeds.
# 
#  GTSmartActors is free software: you can redistribute it and/or modify
#  it under the terms of the GNU General Public License as published by
#  the Free Software Foundation, either version 3 of the License, or
#  (at your option) any later version.
# 
#  GTSmartActors is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU General Public License for more details.
# 
#  You should have received a copy of the GNU General Public License
#  along with ODRoNeS. If not, see <http://www.gnu.org/licenses/>.
# 
#  We would appreciate that if you use this software for work leading 
#  to publications you cite the package and its related publications. 
#
# Ask for C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)


# Defining RNS_DIR is convenient when you have ODRoNeS as a git submodule.
if (NOT DEFINED RNS_DIR)
   set( RNS_DIR ${PROJECT_SOURCE_DIR} )
endif ()
if (NOT DEFINED RNS_INSTALL_DEV)
   set( RNS_INSTALL_DEV ON )
endif ()
if (NOT DEFINED RNS_INSTALL_DIR)
	SET( RNS_INSTALL_DIR ${CMAKE_INSTALL_PREFIX} )
endif ()

# Our own headers:
include_directories(${RNS_DIR}/include)

# 3rd Party:
# Clothoids:
set(CLOTHOIDS_DIR ${RNS_DIR}/3rdParty/clothoids)
include(${CLOTHOIDS_DIR}/CMakeLists.txt)
# cxxOpts:
include_directories(${RNS_DIR}/3rdParty/cxxopts)
# tinyxml
include_directories(${RNS_DIR}/3rdParty/tinyxml2)


# Qt graphical interface:
option(ODRONES_USE_QT "Build ODRoNeS with a graphical interface" ON)
if (ODRONES_USE_QT)
  find_package(Qt6 COMPONENTS Core Gui Widgets) # 3DCore 3DRender 3DExtras)
  if (NOT ${Qt6_FOUND})
      message(FATAL_ERROR "You could also try compiling without Qt support using -DUSE_QT=OFF")
  endif (NOT ${Qt6_FOUND})
  set(QT_LIBRARIES Qt6::Core Qt6::Gui Qt6::Widgets)
  set(CMAKE_AUTOUIC ON)
  set(CMAKE_AUTOMOC ON)
  set(CMAKE_AUTORCC ON)
endif(ODRONES_USE_QT)

# OneVersion interface:
option(ODRONES_USE_ONEVERSION "[ Deprecated ] Add support for OneVersion maps" OFF)
if(ODRONES_USE_ONEVERSION)
  # Find the headers:
  find_path(ONEVERSION_INCLUDE_DIR NAMES sim/Car.h
          HINTS ${ONEVERSION_HOME}/Simulator3/include
          HINTS $ENV{ONEVERSION_HOME}/Simulator3/include
	  HINTS ${CMAKE_PREFIX_PATH}/Simulator3/include)
  cmake_path(GET ONEVERSION_INCLUDE_DIR PARENT_PATH ONEVERSION_ROOT)
  cmake_path(GET ONEVERSION_ROOT PARENT_PATH ONEVERSION_ROOT)
  message(STATUS "OneVersion: ${ONEVERSION_ROOT}")
  include_directories(${ONEVERSION_INCLUDE_DIR})
  message(STATUS "OneVersion include: ${ONEVERSION_INCLUDE_DIR}")
  include_directories(${ONEVERSION_ROOT}/include)
  message(STATUS "OneVersion include: ${ONEVERSION_ROOT}/include")


  # Find the library
  find_library(ONEVERSION_LIBRARY NAMES libsim sim HINTS ${ONEVERSION_ROOT}/lib)
  message(STATUS "libsim: ${ONEVERSION_LIBRARY}")

  # Definitions and dependencies:
  include(${ONEVERSION_ROOT}/share/cmake/BuildSim.cmake)
  add_compile_definitions(${ONEVERSION_DEFINITIONS})
  include_directories(${SIMDEPS_INCLUDE_DIRS})

  # Add an extra definition
  add_definitions(-DUSE_ONEVERSION)

endif(ODRONES_USE_ONEVERSION)

# Build the RNS library:
add_library(rns
    ${RNS_DIR}/include/constants.h 
    ${RNS_DIR}/src/rnsconcepts.cpp ${RNS_DIR}/include/rnsconcepts.h
    ${RNS_DIR}/src/matvec.cpp ${RNS_DIR}/include/matvec.h
    ${RNS_DIR}/src/readOdr.cpp ${RNS_DIR}/include/readOdr.h
    ${RNS_DIR}/src/readXOdr.cpp ${RNS_DIR}/include/readXOdr.h
    ${RNS_DIR}/src/readBOdr.cpp ${RNS_DIR}/include/readBOdr.h
    ${RNS_DIR}/src/readOneVersion.cpp ${RNS_DIR}/include/readOneVersion.h
    ${RNS_DIR}/src/geometry.cpp ${RNS_DIR}/include/geometry.h
    ${RNS_DIR}/src/parametric.cpp ${RNS_DIR}/include/parametric.h
    ${RNS_DIR}/src/numerical.cpp ${RNS_DIR}/include/numerical.h
    ${RNS_DIR}/src/vwNumerical.cpp ${RNS_DIR}/include/vwNumerical.h
    ${RNS_DIR}/src/straight.cpp ${RNS_DIR}/include/straight.h
    ${RNS_DIR}/src/vwStraight.cpp ${RNS_DIR}/include/vwStraight.h
    ${RNS_DIR}/src/arc.cpp ${RNS_DIR}/include/arc.h
    ${RNS_DIR}/src/vwArc.cpp ${RNS_DIR}/include/vwArc.h
    ${RNS_DIR}/src/paramPoly3.cpp ${RNS_DIR}/include/paramPoly3.h
    ${RNS_DIR}/src/vwParamPoly3.cpp ${RNS_DIR}/include/vwParamPoly3.h
    ${RNS_DIR}/src/bezier.cpp ${RNS_DIR}/src/bezier2.cpp ${RNS_DIR}/src/bezier3.cpp
    ${RNS_DIR}/include/bezier.h ${RNS_DIR}/include/bezier2.h ${RNS_DIR}/include/bezier3.h
    ${RNS_DIR}/src/vwSpiral.cpp ${RNS_DIR}/include/vwSpiral.h
    ${RNS_DIR}/src/lane.cpp ${RNS_DIR}/include/lane.h
    ${RNS_DIR}/src/section.cpp ${RNS_DIR}/include/section.h
    ${RNS_DIR}/src/rns.cpp ${RNS_DIR}/include/rns.h
    # Some tests:
    # ${RNS_DIR}/src/testArcLane.cpp ${RNS_DIR}/include/testArcLane.h
    # ${RNS_DIR}/src/testLane.cpp ${RNS_DIR}/include/testLane.h
    # ${RNS_DIR}/src/testBezier.cpp ${RNS_DIR}/include/testBezier.h
    # Qt plots:
    ${RNS_DIR}/src/graphicalrns.cpp ${RNS_DIR}/include/graphicalrns.h
    ${RNS_DIR}/src/graphicalZoom.cpp ${RNS_DIR}/include/graphicalZoom.h
    ${RNS_DIR}/3rdParty/tinyxml2/tinyxml2.cpp
)
target_link_libraries(rns clothoids ${QT_LIBRARIES})

#How about Python bindings:
option(ODRONES_PYTHON_BINDINGS "Build Python3 bindings for ODRoNeS" OFF)
if (ODRONES_PYTHON_BINDINGS) 
	find_package(Python 3.0 COMPONENTS Interpreter Development REQUIRED)
	find_package(SWIG REQUIRED)
	include(UseSWIG)
	include_directories(${Python_INCLUDE_DIRS})
	set(USE_SWIG_FLAGS "-py3")

   set_source_files_properties(${RNS_DIR}/include/odrones.i ${RNS_DIR}/include/readOdr.i
	             PROPERTIES CPLUSPLUS ON USE_SWIG_DEPENDENCIES ON)
	set_property(SOURCE ${RNS_DIR}/include/odrones.i ${RNS_DIR}/include/readOdr.i
	             PROPERTY SWIG_FLAGS ${USE_SWIG_FLAGS})
	swig_add_library(odrones TYPE SHARED LANGUAGE python
                    SOURCES ${RNS_DIR}/include/odrones.i)
	swig_link_libraries(odrones ${Python_LIBRARIES} rns clothoids)

	message(STATUS "ODRoNeS: ${RNS_INSTALL_DIR}")
	set(ODRONES_PYTHON_INSTALL_DIR 
	    ${RNS_INSTALL_DIR}/lib/python${Python_VERSION_MAJOR}.${Python_VERSION_MINOR}/site-packages/odrones 
		 CACHE INTERNAL "folder in which ODRoNeS will install the Python bindings" FORCE)
	install (TARGETS odrones DESTINATION ${ODRONES_PYTHON_INSTALL_DIR})
	install (FILES ${CMAKE_BINARY_DIR}/odrones.py DESTINATION ${ODRONES_PYTHON_INSTALL_DIR})
endif()




if (ODRONES_USE_ONEVERSION)
    target_link_libraries(rns ${ONEVERSION_LIBRARY} ${SIMDEPS_LIBRARIES})
endif (ODRONES_USE_ONEVERSION)


# Build the Graphical representations, provided you have Qt available
if (USE_QT)
  add_executable(rnscheck
      ${RNS_DIR}/src/main.cpp
      ${RNS_DIR}/src/rnswindow.cpp ${RNS_DIR}/include/rnswindow.h
  )
  target_link_libraries(rnscheck rns ${QT_LIBRARIES})

  # qt_add_executable(rns3d 
      # ${RNS_DIR}/src/mainQt3D.cpp 
      # ${RNS_DIR}/src/mainwindow.cpp ${RNS_DIR}/include/mainwindow.h)
  # target_link_libraries(rns3d PRIVATE rns Qt6::Widgets Qt6::3DCore Qt6::3DRender Qt6::3DExtras)

  install (TARGETS rnscheck RUNTIME DESTINATION ${RNS_INSTALL_DIR}/bin)
endif (USE_QT)

if (RNS_INSTALL_DEV)
   install (TARGETS rns DESTINATION ${RNS_INSTALL_DIR}/lib)
   install (DIRECTORY ${RNS_DIR}/include DESTINATION ${RNS_INSTALL_DIR}/include/odrones)
endif ()
                         

