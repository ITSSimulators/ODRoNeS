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
using namespace odrones;

std::string concepts::actorString(actor a)
{
    switch(a)
    {
    case actor::car:
        return "car";
    case actor::meeple:
        return "meeple";
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


