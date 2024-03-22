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

#include "bezier.h"

#ifndef ODRONES_BEZIER3_H
#define ODRONES_BEZIER3_H

namespace odrones
{

class bezier3 : public bezier
{
public:
    bezier3();
    bezier3(const bezier3& b);
    bezier3& operator=(const bezier3& b);
    ~bezier3() override;
    void assignInputLaneToThis(const bezier3& b);

    // bezier3(const arr2 &p0, const arr2 &p1, const arr2 &p2, const arr2 &p3);
    void set(const arr2 &p0, const arr2 &p1, const arr2 &p2, const arr2 &p3);
    scalar calcLength() const override;
    scalar calcLength(scalar t) const override;

    scalar curvex(scalar t) const override;
    scalar curvey(scalar t) const override;
    arr2 curvexy(scalar t) const override;

    scalar curvePx(scalar t) const override;
    scalar curvePy(scalar t) const override;
    arr2 curvePxy(scalar t) const override;

    scalar curveP2x(scalar t) const override;
    scalar curveP2y(scalar t) const override;
    arr2 curveP2xy(scalar t) const override;

    uint rootPx(scalar &t1, scalar &t2) const override; ///< returns the value of t at which Bx' == 0; false if never.
    uint rootPy(scalar &t1, scalar &t2) const override; ///< returns the value of t at which By' == 0; false if never

    bool getTGivenXY(scalar &t, const arr2 &xy) const override;

    /*! return a new bezier curve that starts in t = 0 and ends in t: */
    bezier3 getStartingPart(scalar t) const;
    /*! return a new bezier curve that starts in t and ends in 1 */
    bezier3 getEndingPart(scalar t) const;

    bool getPointAfterDistance(arr2 &p, const arr2 &o, scalar d) const override;

    void getBoundingBox(arr2 &blc, arr2 &trc) const override;

    bool getIntersectionPointFromOT(arr2 &p, const arr2 &o, const arr2 &t) const override;

};


}

#endif // ODRONES_BEZIER3_H
