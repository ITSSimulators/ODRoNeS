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

#ifndef ODRONES_RNSCONCEPTS_H
#define ODRONES_RNSCONCEPTS_H

#include <string>
#include <vector>

namespace odrones
{

class concepts
{
public:
    /*! meeple + OpenSCENARIO's vehicle category https://publications.pages.asam.net/standards/ASAM_OpenSCENARIO/ASAM_OpenSCENARIO_XML/latest/generated/content/VehicleCategory.html */
    enum class actor {car, meeple, bus, motorbike, semitrailer, trailer, tram, truck, van, none};
    static std::string actorString(actor a);

    inline static const std::vector<actor> actorV {
        actor::car, actor::meeple, actor::bus, actor::motorbike, actor::semitrailer,
        actor::trailer, actor::tram, actor::truck, actor::van };


    /*! whether we're on a right-hand or a left-hand driving scenario */
    enum class drivingSide { rightHand, leftHand };
    static std::string drivingString(drivingSide d);

};


} // namespace odrones;

#endif // ODRONES_RNSCONCEPTS_H
