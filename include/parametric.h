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

#ifndef PARAMETRIC_H
#define PARAMETRIC_H

#include "geometry.h"

namespace odrones
{

class parametric : public geometry
{
public:

    parametric();

    virtual arr2 curvexy(scalar t) const = 0;
    virtual arr2 curvePxy(scalar t) const = 0;
    virtual arr2 curveP2xy(scalar t) const = 0;

    virtual bool rootPx(scalar &t) const;  ///< Order 2 curves; returns the value of t at which Bx' == 0; false if never.
    virtual bool rootPy(scalar &t) const;  ///< Order 2 curves; returns the value of t at which By' == 0; false if never.
    virtual uint rootPx(scalar &t1, scalar &t2) const; ///< Order 3 curves; returns the values of t at which Px'(t) = 0, and the number of valid answers, so that 1 means that t1 is valid and t2 is not.
    virtual uint rootPy(scalar &t1, scalar &t2) const; ///< Order 3 curves; returns the values of t at which Py'(t) = 0, and the nubmer of valid answeres, so that 1 it means that t1 is valid and t2 is not.

    scalar calcLengthGQ20() const; ///< calculate the total length using a Gaussian Quadrature with 20 points.
    scalar calcLengthGQ30() const; ///< calculate the total length using a Gaussian Quadrature with 30 points.

    /*! radius of curvature at t */
    scalar pGetCurvature(scalar t) const;
    scalar pGetCurvature(const arr2& p) const;

    /*! check whether 0 <= t <= _maxt */
    bool isTinRange(scalar t) const;
    /*! check whether 0 <= t <= _maxt AND if t is almost 0/_maxt, set it to 0/_maxt. */
    bool isTinRangeFix(scalar &t) const;

    /*! get t at a certain point on the curve */
    virtual bool getTGivenXY(scalar &t, const arr2 &xy) const = 0;

    //! return the top left corner, length and height of the rectangle that aligned to the x,y axis bounds the curve.
    void calcBoundingBoxO2(arr2 &blc, arr2 &trc) const;
    void calcBoundingBoxO3(arr2 &blc, arr2 &trc) const;

    //! places in p the projection of o onto the bezier line, and returns the corresponding t.
    scalar projectPointHereT(arr2 &p, const arr2 &o) const;


protected:
    scalar _mint; ///< minimum value for parameter t.
    scalar _maxt; ///< maximum value for parameter t.

    /*! return how many of the ti are in range,
     *  and reorder the ti's so that they're valid up to the n
     *  Beware that t1 may become greater or smaller than t2, the order does not have anything to do with that */
    uint reorderTinRange(scalar &t1, scalar &t2) const;
    uint reorderTinRange(scalar &t1, scalar &t2, scalar &t3) const;
    uint reorderTinRangeFix(scalar &t1, scalar &t2) const;
    uint reorderTinRangeFix(scalar &t1, scalar &t2, scalar &t3) const;

};


} // namespace odrones;



#endif // PARAMETRIC_H
