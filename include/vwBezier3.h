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

#include <array>
#include <functional>
#include "vwNumerical.h"
#include "bezier3.h"

#ifndef ODRONES_VWBEZIER3_H
#define ODRONES_VWBEZIER3_H

namespace odrones
{

class vwBezier3 : public vwNumerical
{
public:

    vwBezier3(const Odr::geometry &odr, int sign, std::vector<Odr::offset> ioffset,
                             scalar so, scalar se, scalar roadSo, bool print);


    arr2 l0ControlPoint(uint i) const { return _bz0.controlPoint(i); }

private:
    arr2 curvexy_a(scalar t) const override; ///< normalised or not, here t is the distance in meters.
    arr2 l0xy_a(scalar t) const override; ///< t is the distance in meters.

private:
    bezier3 _bz0;
    int _sign;

};

} // namespace odrones;

#endif // ODRONES_VWBEZIER3_H
