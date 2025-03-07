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

#ifndef ODRONES_NUMERIC_H
#define ODRONES_NUMERIC_H

#include <iostream>
#include "matvec.h"

#ifdef QT_CORE_LIB
#include <QPainterPath>
#endif // QT_CORE_LIB

namespace odrones
{

class numerical
{
public:
    numerical();
    numerical(const numerical& n);

    /*! By defining ds and size, initialise stores these values into
     *   _pointsDs and _pointsSize and allocates the arrays. */
    void initialise(scalar ds, uint size);

    /*! Every class needs to implement it on its own,
     *   filling in the arrays, and defining _lastDs and _maxS. */
    virtual void nSetupPointsXYUniformly(scalar ds) = 0;

    /*! Setup() calls nSetupPointsXYUniformly and then
     *   it defines _lastDs and _maxS. */
    uint setup();

    numerical& operator=(const numerical& n);
    void assignInputToThis(const numerical& n);
    virtual ~numerical();

    void allocateMemory(uint pSize);
    void zeroPoints(); ///< set the value of all the points to zero
    void clearMemory();

    void base(); ///< initialise all the variables.
    void nInvert(); ///< invert recalculating the points using interpolate().
    bool isSet() const; ///< check if _pointsSize > 0;

    arr2 interpolate(scalar d) const; ///< interpolate the value of the curve at 0 <= d <= n.
    bool interpolate(arr2 &p, scalar d) const; ///< interpolate safely;
    scalar interpolateSo(scalar d) const;
    scalar nProjectPointHere(arr2 &p, const arr2 &o) const;
    scalar nDistanceToTheEoL(const arr2 &p) const;
    bool nGetPointAfterDistance(arr2 &p, const arr2 &o, scalar d) const;
    bool nGetPointAfterDistance(arr2 &p, scalar s, scalar d) const;
    bool nGetPointAtDistance(arr2 &p, scalar d) const;
    void nCalcBoundingBox(arr2 &blc, arr2 &trc); ///< calculate the bounding box given _pointsX and _pointsY.
    bool nGetIntersectionPointFromOT(arr2 &p, const arr2 &o, const arr2 &t) const;
    scalar nGetCurvature(const arr2 &p) const; ///< calculate the curvture at p using the _pointsX,Y arrays.
    arr2 nGetTangentInPoint(const arr2 &p) const; ///< numerical differentiation.
#ifdef QT_CORE_LIB
    QPainterPath nGetQPainterPath(uint n) const;
#endif


    static scalar defaultDs(scalar length); ///< return min(0.05, length / 3.);

    scalar maxS() const;
    std::vector<scalar> S() const;
    std::vector<scalar> So() const;
    std::vector<arr2> points() const;
    uint pointsSize() const;

private:
    void interpolateSCore(uint &ndx, scalar &frac, scalar s) const; ///< figure out the ndx and fraction so that you can interpolate _pointsX,Y and _pointsSo;



protected:
    /*! Two arrays of points along the curve at a certain resolution, so that we can do some numerical geometry
     *   This seems rather horrible, but it works very well because we do bounding boxes all the time,
     *  which means finding min and max all the time */
    scalar *_pointsX, *_pointsY, *_pointsS, *_pointsSo;
    uint _pointsSize; ///< length of the arrays
    scalar _approxDs; ///< approximated distance between points.

    static constexpr uint minPointsSize = 5;


};

} // namespace odrones;


#endif // ODRONES_NUMERIC_H
