//
//   This file is part of ODRoNeS (OpenDRIVE Road Network System).
//
//   Copyright (c) 2019-2026 Albert Solernou, University of Leeds.
//
//   The ODRoNeS package is free software; you can redistribute it and/or
//   modify it under the terms of the GNU Lesser General Public
//   License as published by the Free Software Foundation; either
//   version 3 of the License, or (at your option) any later version.
//
//   The ODRoNeS package is distributed in the hope that it will be useful,
//   but WITHOUT ANY WARRANTY; without even the implied warranty of
//   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
//   Lesser General Public License for more details.
//
//   You should have received a copy of the GNU Lesser General Public
//   License along with the ODRoNeS package; if not, see
//   <https://www.gnu.org/licenses/>.
//



#include "readBOdr.h"
using namespace odrones;

void ReadBOdr::addRoad(Odr::smaS &s)
{
    _sections.push_back(s);
}

void ReadBOdr::addConnection(Odr::udIndexed6DPoint &c)
{
    _udConnections.push_back(c);
}

void ReadBOdr::addSimplification(bool singleArc, bool straight, bool arcSeries)
{
    _optimisations.push_back({singleArc, straight, arcSeries});
}



