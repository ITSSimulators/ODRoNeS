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

#ifndef RNSCONCEPTS_H
#define RNSCONCEPTS_H

#include <string>

namespace odrones
{

class rnsConcepts
{
public:
    enum class actor {car, meeple, none};
    static std::string actorString(actor a);

    /*! whether we're on a right-hand or a left-hand driving scenario */
    enum class drivingSide { rightHand, leftHand };
    static std::string drivingString(drivingSide d);
};

typedef rnsConcepts concepts;

} // namespace odrones;

#endif // RNSCONCEPTS
