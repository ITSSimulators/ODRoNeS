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

#ifndef ODRONES_READODR_H
#define ODRONES_READODR_H

#include <iostream>
#include <string>
#include <vector>
#include <utility>

#include "tinyxml2.h"
#include "xmlUtils.h"
#include "Odr.h"

namespace odrones
{


class ReadOdr
{
public:
    enum class kind { none, bodr, xodr };
    ReadOdr() : _k(kind::none) {};
    ReadOdr& operator=(const ReadOdr& r);

protected:
    ReadOdr(kind k) : _k(k) {};

public:
    void printRoads(); ///< print out the roads

    bool isReady(); ///< whether it has the sections ready or not.

    /*! return the point to a lane that has the required OpenDRIVE IDs */
    Odr::smaL* getLaneWithODRIds(uint rOdrID, int lOdrID, int lsID);

    const kind k() const { return _k; }

    const std::vector<Odr::smaS> &sections = _sections; ///< share a read-only version of _sections.

    const Odr::smaS* odrSection(uint odrID) const; ///< return the section with ID = OdrID.

    static constexpr scalar defaultSpeed = 30 * constants::mphToMs; ///< default speed is 30 mph.

protected:
    std::vector<Odr::smaS> _sections;

    kind _k;

    bool ready;

};

} // namespace odrones;

#endif // ODRONES_READODR_H
