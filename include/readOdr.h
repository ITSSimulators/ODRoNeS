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

#include "Odr.h"

namespace odrones
{


class ReadOdr
{
public:
    enum class kind { none, bodr, xodr };
    ReadOdr() : _k(kind::none) {};
    ReadOdr& operator=(const ReadOdr& r);
    ReadOdr& operator+=(const ReadOdr& r);

    void renumber(uint shift); ///< renumber the Odr ids of the sections.
    void transform(const arr2 &position, scalar angle);

protected:
    ReadOdr(kind k) : _k(k) {};

public:
    void printRoads() const; ///< print out the roads

    bool ready() const; ///< whether it has the sections ready or not.
    void ready(bool r); ///< set ready to r;

    /*! return the point to a lane that has the required OpenDRIVE IDs */
    Odr::smaL* getLaneWithODRIds(uint rOdrID, int lOdrID, int lsID);

    const kind k() const { return _k; }

    const std::vector<Odr::smaS> &sections = _sections; ///< share a read-only version of _sections.

    const Odr::smaS* odrSection(uint odrID) const; ///< return the section with ID = OdrID.

    static constexpr scalar defaultSpeed = 30 * constants::mphToMs; ///< default speed is 30 mph.

    const std::vector<Odr::udIndexed6DPoint> &udConnections = _udConnections; ///< share a read-only version

private:
    void append(const ReadOdr& r);

protected:
    std::vector<Odr::smaS> _sections;

    std::vector<Odr::udIndexed6DPoint> _udConnections;

    std::vector<Odr::speedRegulation> _defaultSpeedLimit;

    kind _k;

    bool _ready;

};

} // namespace odrones;

#endif // ODRONES_READODR_H
