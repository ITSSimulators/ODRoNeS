#
#   This file is part of ODRoNeS (OpenDRIVE Road Network System).
#
#   Copyright (c) 2019-2026 Albert Solernou, University of Leeds.
#
#   The ODRoNeS package is free software; you can redistribute it and/or
#   modify it under the terms of the GNU Lesser General Public
#   License as published by the Free Software Foundation; either
#   version 3 of the License, or (at your option) any later version.
#
#   The ODRoNeS package is distributed in the hope that it will be useful,
#   but WITHOUT ANY WARRANTY; without even the implied warranty of
#   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
#   Lesser General Public License for more details.
#
#   You should have received a copy of the GNU Lesser General Public
#   License along with the ODRoNeS package; if not, see
#   <https://www.gnu.org/licenses/>.
#



cmake_minimum_required(VERSION 3.20)
project(ODRoNeS)
cmake_policy(VERSION 3.20)


# Package definitions:
set(PACKAGE "ODRoNeS")
set(VERSION "0.9")
set(PACKAGE_URL "")
set(PACKAGE_NAME ${PACKAGE})
set(PACKAGE_STRING "${PACKAGE} ${VERSION}")
set(PACKAGE_TARNAME ${PACKAGE})
set(PACKAGE_VERSION ${VERSION})
set(PACKAGE_BUGREPORT "")

include(ODRoNeS.cmake)
