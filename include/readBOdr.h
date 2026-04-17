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



#ifndef ODRONES_READBODR_H
#define ODRONES_READBODR_H


#include "readOdr.h"

namespace odrones 
{


class ReadBOdr : public ReadOdr
{
public:
    ReadBOdr() : ReadOdr(ReadOdr::kind::bodr) {}

    void addRoad(Odr::smaS &s);

    void addConnection(Odr::udIndexed6DPoint &c);

    void addSimplification(bool singleArc, bool straight, bool arcSeries);

};

} // namespace odrones;

#endif // ODRONES_READBODR_H
