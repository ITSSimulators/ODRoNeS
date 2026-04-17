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
    Odr::geometry refAsParamPoly3(bool normalised) const
    {
        return _bz0.asParamPoly3(normalised);
    }

private:
    arr2 curvexy_a(scalar t) const override; ///< normalised or not, here t is the distance in meters.
    arr2 l0xy_a(scalar t) const override; ///< t is the distance in meters.

private:
    bezier3 _bz0;
    int _sign;

};

} // namespace odrones;

#endif // ODRONES_VWBEZIER3_H
