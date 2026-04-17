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



#include "rnsconcepts.h"
#include "Odr.h"
using namespace odrones;

std::string concepts::actorString(actor a)
{
    switch(a)
    {
    case actor::car:
        return "car";
    case actor::meeple:
        return "meeple";
    case actor::bus:
        return "bus";
    case actor::motorbike:
        return "motorbike";
    case actor::semitrailer:
        return "semitrailer";
    case actor::trailer:
        return "trailer";
    case actor::tram:
        return "tram";
    case actor::truck:
        return "truck";
    case actor::van:
        return "van";

    default:
        return "unknown";
    }
}

std::string concepts::drivingString(drivingSide d)
{
    switch(d)
    {
    case drivingSide::leftHand:
        return "left-hand";
    case drivingSide::rightHand:
        return "right-hand";
    default:
        return "no idea";
    }
}


bool concepts::actorIsFourWheeled(actor a)
{
    if ((a == actor::car) || (a == actor::bus) || (a == actor::semitrailer) ||
        (a == actor::trailer) || (a == actor::tram) || (a == actor::truck) || (a == actor::van))
        return true;
    return false;
}
