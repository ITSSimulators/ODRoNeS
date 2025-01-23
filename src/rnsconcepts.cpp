// 
//  This file is part of the ODRoNeS (OpenDRIVE Road Network System) package.
//  
//  Copyright (c) 2023 Albert Solernou, University of Leeds.
// 
//  GTSmartActors is free software: you can redistribute it and/or modify
//  it under the terms of the GNU General Public License as published by
//  the Free Software Foundation, either version 3 of the License, or
//  (at your option) any later version.
// 
//  GTSmartActors is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU General Public License for more details.
// 
//  You should have received a copy of the GNU General Public License
//  along with ODRoNeS. If not, see <http://www.gnu.org/licenses/>.
// 
//  We would appreciate that if you use this software for work leading 
//  to publications you cite the package and its related publications. 
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

const concepts::roadType concepts::rt(const char* c)
{
    if (!strcmp(c, Odr::Kind::Bicycle))
        return roadType::bicycle;

    if (!strcmp(c, Odr::Kind::LowSpeed))
        return roadType::lowSpeed;

    if (!strcmp(c, Odr::Kind::Motorway))
        return roadType::motorway;

    if (!strcmp(c, Odr::Kind::Pedestrian))
        return roadType::pedestrian;

    if (!strcmp(c, Odr::Kind::Rural))
        return roadType::rural;

    if (!strcmp(c, Odr::Kind::TownArterial))
        return roadType::townArterial;

    if (!strcmp(c, Odr::Kind::TownCollector))
        return roadType::townCollector;

    if (!strcmp(c, Odr::Kind::TownExpressway))
        return roadType::townExpressway;

    if (!strcmp(c, Odr::Kind::TownLocal))
        return roadType::townLocal;

    if (!strcmp(c, Odr::Kind::TownPlayStreet))
        return roadType::townPlayStreet;

    if (!strcmp(c, Odr::Kind::TownPrivate))
        return roadType::townPrivate;

    if (!strcmp(c, Odr::Kind::Town))
        return roadType::town;


    return roadType::unknown;
}
