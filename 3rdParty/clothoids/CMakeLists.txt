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
#  along with ODRoNeS.  If not, see <http://www.gnu.org/licenses/>.
# 
#  We would appreciate that if you use this software for work leading 
#  to publications you cite the package and its related publications. 
#


add_definitions(-DG2LIB_COMPATIBILITY_MODE)
add_definitions(-DG2LIB_USE_CXX11)

include_directories(${CLOTHOIDS_DIR}/submodules/quarticRootsFlocke)
include_directories(${CLOTHOIDS_DIR}/src)

add_library( clothoids
	${CLOTHOIDS_DIR}/submodules/quarticRootsFlocke/PolynomialRoots-1-Quadratic.cc
	${CLOTHOIDS_DIR}/submodules/quarticRootsFlocke/PolynomialRoots-2-Cubic.cc
	${CLOTHOIDS_DIR}/submodules/quarticRootsFlocke/PolynomialRoots-3-Quartic.cc
	${CLOTHOIDS_DIR}/submodules/quarticRootsFlocke/PolynomialRoots-Jenkins-Traub.cc
	${CLOTHOIDS_DIR}/submodules/quarticRootsFlocke/PolynomialRoots-Utils.cc
	${CLOTHOIDS_DIR}/src/AABBtree.cc
	${CLOTHOIDS_DIR}/src/Biarc.cc
	${CLOTHOIDS_DIR}/src/BiarcList.cc
	${CLOTHOIDS_DIR}/src/Circle.cc
	${CLOTHOIDS_DIR}/src/Clothoid.cc
	${CLOTHOIDS_DIR}/src/ClothoidDistance.cc
	${CLOTHOIDS_DIR}/src/ClothoidG2.cc
	${CLOTHOIDS_DIR}/src/ClothoidList.cc
	${CLOTHOIDS_DIR}/src/Fresnel.cc
	${CLOTHOIDS_DIR}/src/G2lib.cc
	${CLOTHOIDS_DIR}/src/G2lib_intersect.cc
	${CLOTHOIDS_DIR}/src/Line.cc
	${CLOTHOIDS_DIR}/src/PolyLine.cc
	${CLOTHOIDS_DIR}/src/Triangle2D.cc )


# add_executable(cteb ${CLOTHOIDS_DIR}/main.cpp)
# target_link_libraries(cteb clothoids)
