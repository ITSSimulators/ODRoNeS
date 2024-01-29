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

#include "paramPoly3.h"


paramPoly3::paramPoly3(const paramPoly3 &p)
{
    assignInputGeomToThis(p);
}

void paramPoly3::base()
{
    _u = {0., 0., 0., 0.};
    _v = {0., 0., 0., 0.};
    _normalised = false;
    _mint = 0;
    _maxt = 0;

    _sign = 0;
    _offset = 0;
    _odrLength = 0;

    _po = {0., 0.};
    _pto = {0., 0.};
}

paramPoly3::paramPoly3(const Odr::geometry &odg, int sign, scalar offsetA, scalar so, scalar se)
{
    _shape = mvf::shape::paramPoly3;
    _sign = sign;
    _offset = offsetA;
    _odrLength = odg.length;

    _u[0] = odg.aU;
    _u[1] = odg.bU;
    _u[2] = odg.cU;
    _u[3] = odg.dU;

    _v[0] = odg.aV;
    _v[1] = odg.bV;
    _v[2] = odg.cV;
    _v[3] = odg.dV;

    _o = {odg.x, odg.y};
    _to = {std::cos(odg.hdg), std::sin(odg.hdg)};

    _po = _o;
    _pto = _to;

    // Check that so and se are within bounds:
    if (!mvf::isInRangeLR(so, 0, odg.length))
        std::cerr << "so is out of bonds!!" << std::endl;

    if (!mvf::isInRangeLR(se, 0, odg.length))
        std::cerr << "se is out of bonds!!" << std::endl;

    _mint = so;
    _maxt = se;
    if (odg.pRange == Odr::Attr::ParamPoly3Range::normalized)
    {
        _normalised = true;
        _mint = _mint / _odrLength;
        _maxt = _maxt / _odrLength;
    }
    else if (odg.pRange == Odr::Attr::ParamPoly3Range::arcLength)
        _normalised = false;
    else
        std::cerr << "[ Error ] paramPoly3 pRange is neither normalized nor arcLength" << std::endl;

    _origin = paramPoly3::curvexy(_mint);
    _dest = paramPoly3::curvexy(_maxt);

    // setup the numerical side:
    _length = calcLength(); // WARNING: we'll change it in a minute.
    scalar ds = numerical::defaultDs(_length);
    uint size = 1 + static_cast<uint>(std::round(_length / ds));
    numerical::initialise(ds, size);
    numerical::setup(); // this calls nSetupPointsXYUniformly, and needs _length.
    _length = numerical::maxS(); // but we're using the numerical length for all the calculations,
                                // since the distanceToEoL calculations use numerical.

    // and now use it to calculate the bounding box:
    nCalcBoundingBox(_blc, _trc);

    /*
    std::cout << "_origin: (" << _origin[0] << ", " << _origin[1] << ") "; // << std::endl;
    std::cout << "_to: (" << _to[0] << ", " << _to[1] << "), "; // << std::endl;
    std::cout << "dest: (" << _dest[0] << ", " << _dest[1] << ") "; // << std::endl;
    std::cout << "normalised: " << std::to_string(_normalised) << " ";
    std::cout << "length: " << _length << ", odrLength: " << _odrLength << " " << std::endl;
    */
}

paramPoly3& paramPoly3::operator=(const paramPoly3 &p3)
{
    numerical::clearMemory();
    assignInputGeomToThis(p3);
    return *this;
}

void paramPoly3::assignInputGeomToThis(const paramPoly3 &p3)
{
    for (uint i = 0; i < 4; ++i)
    {
        _u[i] = p3._u[i];
        _v[i] = p3._v[i];
    }
    _normalised = p3._normalised;

    _sign = p3._sign;
    _offset = p3._offset;
    _odrLength = p3._odrLength;
    _maxt = p3._maxt;

    _po = p3._po;
    _pto = p3._pto;

    geometry::assignInputGeomToThis(p3);
    numerical::assignInputToThis(p3);
}


void paramPoly3::invert()
{

    arr2 te = curvePxy(_maxt);
    _to = {-te[0], -te[1]};
    _sign = -_sign;

    // swap origin and destination
    arr2 tmp = _dest;
    _dest = _origin;
    _origin = tmp;

    // swap o and d:
    tmp = _d;
    _d = _o;
    _o = tmp;


    // get a new set of parameters:
    scalar ol2 = _odrLength * _odrLength;
    scalar ol3 = _odrLength * ol2;
    _u[0] = _u[0] + _u[1]*_odrLength + _u[2]*ol2 + _u[3]*ol3;
    _u[1] = -_u[1] -2*_u[2]*_odrLength -3*_u[3]*ol2;
    _u[2] = _u[2] + 3*_u[3]*_odrLength;
    _u[3] = -_u[3];

    _v[0] = _v[0] + _v[1]*_odrLength + _v[2]*ol2 + _v[3]*ol3;
    _v[1] = -_v[1] -2*_v[2]*_odrLength -3*_v[3]*ol2;
    _v[2] = _v[2] + 3*_v[3]*_odrLength;
    _v[3] = -_v[3];

    scalar tmq = _mint;
    _mint = _odrLength - _maxt;
    _maxt = _odrLength - tmq;

    numerical::zeroPoints();
    numerical::setup();
    _length = numerical::maxS();


    nCalcBoundingBox(_blc, _trc);

}


arr2 paramPoly3::projectPointHere(const arr2 &p) const
{
    arr2 q;
    parametric::projectPointHereT(q, p);
    return q;
}


bool paramPoly3::isPointHere(const arr2 &p) const
{
    if (!mvf::isPointInBoxBLcTRcTol(p, _blc, _trc, mvf::absolutePrecision))
        return false;

    arr2 q;
    parametric::projectPointHereT(q, p);
    if (mvf::sqrDistance(q, p) < mvf::distPrecision2) return true;

    numerical::nProjectPointHere(q, p); /// And here we see that deriving both from numerical and parametric was not a great idea.
    if (mvf::sqrDistance(q, p) < mvf::distPrecision2) return true;
    return false;
}

arr2 paramPoly3::getTangentInPoint(const arr2 &p) const
{
    arr2 q;
    scalar t = parametric::projectPointHereT(q, p);
    arr2 tg = curvePxy(t);
    mvf::normalise(tg);
    return tg;
}

scalar paramPoly3::distanceToTheEoL(const arr2 &p) const
{
    return nDistanceToTheEoL(p);
}

bool paramPoly3::getPointAfterDistance(arr2 &p, const arr2 &o, scalar d) const
{
    return nGetPointAfterDistance(p, o, d);
}

bool paramPoly3::getIntersectionPointFromOT(arr2 &p, const arr2 &o, const arr2 &t) const
{
    return nGetIntersectionPointFromOT(p, o, t);
}

scalar paramPoly3::getCurvature(const arr2 &p) const
{
    // It would be nice to have the 2nd derivative working...
    return nGetCurvature(p);
}

#ifdef QT_CORE_LIB
QPainterPath paramPoly3::getQPainterPath(uint n) const
{
    QPainterPath qpp;
    arr2 p = _origin;
    qpp.moveTo(QPointF(ct::mToPix * p[0], - ct::mToPix * p[1]));
    for (uint i = 0; i <= n; ++i)
    {
        scalar t = _mint + i * (_maxt - _mint) / scalar(n);
        p = curvexy(t);
        qpp.lineTo(QPointF(ct::mToPix * p[0], - ct::mToPix * p[1]));
    }
    qpp.lineTo(QPointF(ct::mToPix * _dest[0], - ct::mToPix * _dest[1]));
    return qpp;
}
#endif // QT_CORE_LIB

arr2 paramPoly3::clxy(scalar t) const
{
    scalar u = _u[0] + _u[1] * t + _u[2] * t*t + _u[3] * t * t * t;
    scalar v = _v[0] + _v[1] * t + _v[2] * t*t + _v[3] * t * t * t;

    return { _po[0] + u * _pto[0] - v * _pto[1],
             _po[1] + u * _pto[1] + v * _pto[0]};
}


arr2 paramPoly3::clPxy(scalar t) const
{
    scalar up = _u[1] + 2 * _u[2] * t + 3 * _u[3] * t * t;
    scalar vp = _v[1] + 2 * _v[2] * t + 3 * _v[3] * t * t;

    return { up * _pto[0] - vp * _pto[1],
             up * _pto[1] + vp * _pto[0]};

}

arr2 paramPoly3::clP2xy(scalar t) const
{
    scalar up2 = 2 * _u[2] + 6 * _u[3] * t;
    scalar vp2 = 2 * _v[2] + 6 * _v[3] * t;

    return { up2 * _pto[0] - vp2 * _pto[1],
             up2 * _pto[1] + vp2 * _pto[0]};
}


arr2 paramPoly3::curvexy(scalar t) const
{
    arr2 cl = clxy(t);
    arr2 ni = clNormal(t);

    return {cl[0] + _offset * ni[0],
            cl[1] + _offset * ni[1]};
}

arr2 paramPoly3::curvePxy(scalar t) const
{
    arr2 clP = clPxy(t);
    arr2 niP = clNormalP(t);
    return {clP[0] + _offset * niP[0],
            clP[1] + _offset * niP[1]};
}


arr2 paramPoly3::curveP2xy(scalar t) const
{
    std::cerr << "[ Error ] paramPoly3::curveP2xy is just the 2nd derivative of the central lane, and should be used as a first approximation"
              << std::endl;
    return clP2xy(t);
}

arr2 paramPoly3::clNormal(scalar t) const
{
    arr2 ti = clPxy(t);
    mvf::normalise(ti);
    return {-_sign * ti[1],
             _sign * ti[0]};
}

arr2 paramPoly3::clNormalP(scalar t) const
{
    // That's the first derivative of the normal of the centre lane, which if we write clxy(t) = f(t) is:
    //           f'(t)    (  0  1 )
    //  n(t) = ---------- |       | * _sign
    //         ||f'(t)||  ( -1  0 )
    // and thus
    //              f''(t) * ||f'(t)|| - f'(t) * ( ||f'(t) ) '      / 0  1 \
    //  n'(t) = -------------------------------------------------   |      | * _sign
    //                        ||f'(t)||^2                           \ -1 0 /
    arr2 ti = clPxy(t);
    scalar tiLength2 = mvf::sqrMagnitude(ti);
    scalar tiLength = std::sqrt(tiLength2);
    arr2 tiP = clP2xy(t);
    scalar tiLengthP = (tiP[0] * ti[0] + tiP[1] * ti[1]) / tiLength;

    arr2 npi = { (tiP[0] * tiLength - ti[0] * tiLengthP) / tiLength2,
                 (tiP[1] * tiLength - ti[1] * tiLengthP) / tiLength2 };

    return {- _sign * npi[1], _sign * npi[0]};

}

/* That is just for cl(t): Wrong.
uint paramPoly3::rootPx(scalar &t1, scalar &t2) const
{
    if (!mvf::solve2ndOrderEq(t1, t2, 3 * _u[3], 2 * _u[2], _u[1] )) return false;
    return reorderTinRangeFix(t1, t2);
}

uint paramPoly3::rootPy(scalar &t1, scalar &t2) const
{
    if (!mvf::solve2ndOrderEq(t1, t2, 3 * _v[3], 2 * _v[2], _v[1] )) return false;
    return reorderTinRangeFix(t1, t2);
}
*/


void paramPoly3::nSetupPointsXYUniformly(scalar dl)
{
   // Memory has already been allocated!
   _pointsX[0] = _origin[0];
   _pointsY[0] = _origin[1];
   _pointsSo[0] = 0;
   arr2 pl0o = clxy(_mint);
   std::vector<scalar> vt(_pointsSize);

   scalar dt = (_maxt - _mint) / _pointsSize;
   scalar t = _mint + dt;
   arr2 io = _origin;
   uint ndx = 1;
   while((t < _maxt) && (ndx < _pointsSize))
   {
       arr2 ie = paramPoly3::curvexy(t);
       _pointsX[ndx] = ie[0];
       _pointsY[ndx] = ie[1];
       arr2 pl0i = clxy(t);
       _pointsSo[ndx] = _pointsSo[ndx-1] + mvf::distance(pl0i, pl0o);
       pl0o = pl0i;

       t += dt;
       ndx += 1;
   }

   while (ndx < _pointsSize) // If we missed points we'll fill in with dest.
   {
       _pointsX[ndx] = _dest[0];
       _pointsY[ndx] = _dest[1];
       arr2 pl0i = clxy(_maxt);
       _pointsSo[ndx] = _pointsSo[ndx-1] + mvf::distance(pl0i, pl0o);
       pl0o = pl0i;

       ndx += 1;
   }

   // and make sure that the last point is dest:
   _pointsX[_pointsSize -1] = _dest[0];
   _pointsY[_pointsSize -1] = _dest[1];
   arr2 pl0i = clxy(_maxt);
   _pointsSo[_pointsSize -1] = _pointsSo[_pointsSize -2] + mvf::distance(pl0i, pl0o);


   /*

   scalar l = dl;
   scalar integral = 0;
   for (uint i = 1; i < _pointsSize; ++i)
   {
       if (l > _length) // we rounded a bit too far.
       {
           if (i != _pointsSize -1)
               std::cout << "[ lane ] _pointsXY will likely go wrong... " << std::endl;
           break;
       }

       while (integral < l)
       {
           t += dt;
           arr2 ie = curvexy(t);
           integral += mvf::distance(ie, io);
           io = ie;
           if (mvf::areCloseEnough(integral, l, mvf::absolutePrecision)) break;
       }
       _pointsX[i] = io[0];
       _pointsY[i] = io[1];
       vt[i] = t;
       arr2 pl0i = clxy(t);
       _pointsSo[i] = _pointsSo[i-1] + mvf::distance(pl0i, pl0o);
       pl0o = pl0i;
       l += dl;
   }

   _pointsX[_pointsSize -1] = _dest[0];
   _pointsY[_pointsSize -1] = _dest[1];
   _pointsSo[_pointsSize -1] = _pointsSo[_pointsSize -2] +
           mvf::distance(clxy(vt[_pointsSize -2]), clxy(_maxt));

   */

   return;
}

scalar paramPoly3::calcLength() const
{
    return calcLengthGQ30();
}


bool paramPoly3::getTGivenXY([[maybe_unused]] scalar &t, [[maybe_unused]] const arr2 &xy) const
{
    // Solving the equations here for a constant offset (width) would mean finding t given x,
    //   x = curvexy(t)
    // and that would be a series of roots for t, and then do the same given a value for y:
    //   y = curvexy(t)
    // and thus there'd need to be a shared root for t that is on both sides, and that is within the [0, _maxt] range.
    // Now, solving the equations is difficult, as they are order 8.
    // If we write clxy as f, and then f_x is the x component, the first equation is:
    //  x = f_x(t) - w * f'_y(t) / ||f'(t)||
    // but now ||f'(t)|| has a square root so if we write m(t) = ||f'(t)||
    // m(t) * x = m(t) * f_x(t) - w * f'_y(t)
    // m2(t) * (x - f_x(t))^2 = (w * f'_y(t))^2 , and in the future we can have w(t)
    // which is order 8 because:
    // O(2) * (O(0) - O(3))^2 = O(4), i e, O(2) * O(6) = O(4)
    // So, we'll need to use Newton-Raphson or some other numerical method:
    // https://en.wikipedia.org/wiki/Root-finding_algorithms

    // Alternatively we could...
    // 1 - Numerically project xy onto cl, then solve the degree-3 polynomial,
    //     and we may not need any further refinement.
    // 2 - Find the parameters of a NEW degree-3 polynomial that goes through curvexy(t).
    //     curvexy(t) = a0 + a1*t + a2*t^2 + a3*t^3
    //     and that could be possible solving a 4x4 linear equations system.
    //     However, what will happen when we have variable widths?
    std::cerr << "Not implemented!" << std::endl;
    return false;
}

scalar paramPoly3::sl0(scalar s) const
{
    return interpolateSo(s) + _roadSo;
}
