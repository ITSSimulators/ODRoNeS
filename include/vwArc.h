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

#include "vwNumerical.h"
#include "arc.h"

#ifndef VWARC_H
#define VWARC_H

class vwArc : public vwNumerical
{
public:
    vwArc(const Odr::geometry &odr, int sign, std::vector<Odr::offset> vwWidth,
          std::vector<Odr::offset> off, scalar so, scalar se, scalar roadSo, bool print);
    void printOut() const;

private:
    arr2 curvexy_a(scalar t) const override; ///< return the value of curvexy(s) ahead


private:
    arr2 _centre;
    scalar _alpha;
    scalar _radiusOfCurvature;
    arr2 _co;
    int _sign;
    bool _print;

};


#endif // VWARC_H
