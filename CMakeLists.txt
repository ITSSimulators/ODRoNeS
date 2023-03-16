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

cmake_minimum_required(VERSION 3.13.3)
project(ODRoNeS)
cmake_policy(VERSION 3.13.3)
if (${CMAKE_VERSION} VERSION_GREATER_EQUAL 3.17)
    cmake_policy(SET CMP0100 NEW)
endif (${CMAKE_VERSION} VERSION_GREATER_EQUAL 3.17)



# Now some more defines for it:
set(PACKAGE "ODRoNeS")
set(VERSION "0.8")
set(PACKAGE_URL "")
set(PACKAGE_NAME ${PACKAGE})
set(PACKAGE_STRING "${PACKAGE} ${VERSION}")
set(PACKAGE_TARNAME ${PACKAGE})
set(PACKAGE_VERSION ${VERSION})
set(PACKAGE_BUGREPORT "")



if (NOT DEFINED RNS_DIR)
   set( RNS_DIR ${PROJECT_SOURCE_DIR} )
endif (NOT DEFINED RNS_DIR)

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


option(USE_QT "Build SmartActors with a graphical interface" ON)
if (USE_QT)
  find_package(Qt5 COMPONENTS Core Gui Widgets)
  if (NOT ${Qt5_FOUND})
      message(FATAL_ERROR "You could also try compiling without Qt support using -DUSE_QT=OFF")
  endif (NOT ${Qt5_FOUND})
  set(QT_LIBRARIES Qt5::Core Qt5::Gui Qt5::Widgets)
  set(CMAKE_AUTOUIC ON)
  set(CMAKE_AUTOMOC ON)
endif(USE_QT)

add_library(rns
    ${RNS_DIR}/include/constants.h 
    ${RNS_DIR}/src/rnsconcepts.cpp ${RNS_DIR}/include/rnsconcepts.h
    ${RNS_DIR}/src/matvec.cpp ${RNS_DIR}/include/matvec.h
    ${RNS_DIR}/src/readOdr.cpp ${RNS_DIR}/include/readOdr.h
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
    # src/testArcLane.cpp include/testArcLane.h
    # src/testLane.cpp include/testLane.h
    # src/testBezier.cpp include/testBezier.h
    # Qt plots:
    ${RNS_DIR}/src/graphicalrns.cpp ${RNS_DIR}/include/graphicalrns.h
    ${RNS_DIR}/src/graphicalZoom.cpp ${RNS_DIR}/include/graphicalZoom.h
    3rdParty/tinyxml2/tinyxml2.cpp
)
target_link_libraries(rns clothoids ${QT_LIBRARIES})

add_executable(rnscheck
    ${RNS_DIR}/src/main.cpp
    ${RNS_DIR}/src/rnswindow.cpp ${RNS_DIR}/include/rnswindow.h
)
target_link_libraries(rnscheck rns ${QT_LIBRARIES})

install (TARGETS rnscheck RUNTIME DESTINATION bin)
install (TARGETS rns DESTINATION lib)
install (DIRECTORY ${RNS_DIR}/include DESTINATION ${CMAKE_INSTALL_PREFIX})
                         

