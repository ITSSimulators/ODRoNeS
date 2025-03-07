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

#ifndef ODRONES_CONSTANTS_H
#define ODRONES_CONSTANTS_H

#include <limits>

namespace odrones
{

typedef double scalar;
typedef unsigned int uint;

namespace constants
{
    constexpr double pixToMD = 0.15;
    constexpr double mToPixD = 1.0/pixToMD;
    constexpr float pixToMF = static_cast<float>(pixToMD);
    constexpr float mToPixF = static_cast<float>(mToPixD);
    constexpr scalar pixToM = pixToMD;
    constexpr scalar mToPix = mToPixD;


    constexpr scalar infinityScalar = std::numeric_limits<scalar>::infinity();
    constexpr scalar largestScalar = std::numeric_limits<scalar>::max();
    constexpr scalar pi = 3.14159265358979323846;
    constexpr scalar sqrt2 = 1.4142135623730951;
    constexpr scalar rad2deg = 180 / pi;
    constexpr scalar deg2rad = pi / 180;
    constexpr scalar oneThird = 0.33333333333333333333;
    constexpr scalar oneSixth = 0.16666666666666666666;

    constexpr scalar g = 9.80665; ///< standard gravity

    constexpr scalar milesToMetres = 1609.44;
    constexpr scalar mphToMs = 0.44704;
    constexpr scalar kmhToMs = 0.27777778;


    // imperial to metric
    constexpr scalar ftToM = 0.3048;

    namespace gc
    {
    constexpr scalar W2[2] = {1, 1};
    constexpr scalar A2[2] = {-0.5773502691896257, 0.5773502691896257};
    constexpr scalar A10[10] = {
        -0.1488743389816312,
        0.1488743389816312,
        -0.4333953941292472,
        0.4333953941292472,
        -0.6794095682990244,
        0.6794095682990244,
        -0.8650633666889845,
        0.8650633666889845,
        -0.9739065285171717,
        0.9739065285171717};

    constexpr scalar W10[10] = {
        0.2955242247147529,
        0.2955242247147529,
        0.2692667193099963,
        0.2692667193099963,
        0.2190863625159820,
        0.2190863625159820,
        0.1494513491505806,
        0.1494513491505806,
        0.0666713443086881,
        0.0666713443086881};

    constexpr scalar A15[15] = {
        0.0000000000000000,
        -0.2011940939974345,
        0.2011940939974345,
        -0.3941513470775634,
        0.3941513470775634,
        -0.5709721726085388,
        0.5709721726085388,
        -0.7244177313601701,
        0.7244177313601701,
        -0.8482065834104272,
        0.8482065834104272,
        -0.9372733924007060,
        0.9372733924007060,
        -0.9879925180204854,
        0.9879925180204854};

    constexpr scalar W15[15] = {
        0.2025782419255613,
        0.1984314853271116,
        0.1984314853271116,
        0.1861610000155622,
        0.1861610000155622,
        0.1662692058169939,
        0.1662692058169939,
        0.1395706779261543,
        0.1395706779261543,
        0.1071592204671719,
        0.1071592204671719,
        0.0703660474881081,
        0.0703660474881081,
        0.0307532419961173,
        0.0307532419961173};

    constexpr scalar A20[20] = {
        -0.0765265211334973,
        0.0765265211334973,
        -0.2277858511416451,
        0.2277858511416451,
        -0.3737060887154195,
        0.3737060887154195,
        -0.5108670019508271,
        0.5108670019508271,
        -0.6360536807265150,
        0.6360536807265150,
        -0.7463319064601508,
        0.7463319064601508,
        -0.8391169718222188,
        0.8391169718222188,
        -0.9122344282513259,
        0.9122344282513259,
        -0.9639719272779138,
        0.9639719272779138,
        -0.9931285991850949,
        0.9931285991850949};

    constexpr scalar W20[20] = {
        0.1527533871307258,
        0.1527533871307258,
        0.1491729864726037,
        0.1491729864726037,
        0.1420961093183820,
        0.1420961093183820,
        0.1316886384491766,
        0.1316886384491766,
        0.1181945319615184,
        0.1181945319615184,
        0.1019301198172404,
        0.1019301198172404,
        0.0832767415767048,
        0.0832767415767048,
        0.0626720483341091,
        0.0626720483341091,
        0.0406014298003869,
        0.0406014298003869,
        0.0176140071391521,
        0.0176140071391521};

    constexpr scalar A30[30] = {
        -0.0514718425553177,
        0.0514718425553177,
        -0.1538699136085835,
        0.1538699136085835,
        -0.2546369261678899,
        0.2546369261678899,
        -0.3527047255308781,
        0.3527047255308781,
        -0.4470337695380892,
        0.4470337695380892,
        -0.5366241481420199,
        0.5366241481420199,
        -0.6205261829892429,
        0.6205261829892429,
        -0.6978504947933158,
        0.6978504947933158,
        -0.7677774321048262,
        0.7677774321048262,
        -0.8295657623827684,
        0.8295657623827684,
        -0.8825605357920527,
        0.8825605357920527,
        -0.9262000474292743,
        0.9262000474292743,
        -0.9600218649683075,
        0.9600218649683075,
        -0.9836681232797472,
        0.9836681232797472,
        -0.9968934840746495,
        0.9968934840746495};

    constexpr scalar W30[30] = {
        0.1028526528935588,
        0.1028526528935588,
        0.1017623897484055,
        0.1017623897484055,
        0.0995934205867953,
        0.0995934205867953,
        0.0963687371746443,
        0.0963687371746443,
        0.0921225222377861,
        0.0921225222377861,
        0.0868997872010830,
        0.0868997872010830,
        0.0807558952294202,
        0.0807558952294202,
        0.0737559747377052,
        0.0737559747377052,
        0.0659742298821805,
        0.0659742298821805,
        0.0574931562176191,
        0.0574931562176191,
        0.0484026728305941,
        0.0484026728305941,
        0.0387991925696271,
        0.0387991925696271,
        0.0287847078833234,
        0.0287847078833234,
        0.0184664683110910,
        0.0184664683110910,
        0.0079681924961666,
        0.0079681924961666};

    }

}

namespace ct = constants;

} // namespace odrones

#endif // ODRONES_CONSTANTS_H
