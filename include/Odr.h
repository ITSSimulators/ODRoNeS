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

#ifndef ODRONES_ODR_H
#define ODRONES_ODR_H

#include <vector>
#include <string>
#include <iostream>
#include <algorithm>
#include "tinyxml2.h"

#include "constants.h"
#include "matvec.h"

namespace odrones 
{

namespace Odr
{

    class Elem
    {
    public:
        static const char* OpenDrive;
        static const char* Header;
        static const char* Road;
        static const char* Type;
        static const char* PlanView;
        static const char* Lane;
        static const char* Lanes;
        static const char* LaneOffset;
        static const char* LaneSection;

        static const char* DefaultRegulations;
        static const char* RoadRegulations;
        static const char* Semantics;

        static const char* Junction;
        static const char* Connection;
        static const char* LaneLink;

        static const char* Link;
        static const char* Predecessor;
        static const char* Successor;

        static const char* Left;
        static const char* Center;
        static const char* Right;

        static const char* Geometry;
        static const char* Line;
        static const char* Arc;
        static const char* Spiral;
        static const char* ParamPoly3;
        static const char* Bezier3;

        static const char* Width;
        static const char* Speed;
        static const char* Border;

        static const char* Signals;
        static const char* Signal;
        static const char* Validity;
        static const char* UserData;
        static const char* Dependency;

        static const char* UDConnectionPoints;
        static const char* UDConnectionPoint;
    };

    class Attr
    {
    public:
        static const char* Type;
        static const char* Name;
        static const char* Id;
        static const char* Junction;
        static const char* Length;
        static const char* Rule;

        static const char* IncomingRoad;
        static const char* ConnectingRoad;
        static const char* ContactPoint;
        static const char* From;
        static const char* To;

        static const char* SingleSide;

        static const char* ElementType;
        static const char* ElementId;

        static const char* Level;

        static const char* Curvature;
        static const char* CurvStart;
        static const char* CurvEnd;
        static const char* S;
        static const char* T;
        static const char* X;
        static const char* Y;
        static const char* Hdg;

        static const char* sOffset;
        static const char* A;
        static const char* B;
        static const char* C;
        static const char* D;

        static const char* Max;

        enum class Geometry {line, spiral, arc, paramPoly3, bezier3, none};
        enum class ParamPoly3Range {arcLength, normalized, none};

        static const char* aU;
        static const char* bU;
        static const char* cU;
        static const char* dU;
        static const char* aV;
        static const char* bV;
        static const char* cV;
        static const char* dV;
        static const char* pRange;

        static const char* bz0x;
        static const char* bz0y;
        static const char* bz1x;
        static const char* bz1y;
        static const char* bz2x;
        static const char* bz2y;
        static const char* bz3x;
        static const char* bz3y;

        static const char* rZ;

        static const char* Unit;

        static const char* Dynamic;
        static const char* Orientation;
        static const char* ZOffset;
        static const char* Value;
        static const char* Height;
        static const char* Width;
        static const char* Text;
        static const char* HOffset;
        static const char* Pitch;
        static const char* Roll;
        static const char* FromLane;
        static const char* ToLane;


    };

    class Kind
    {
    public:
        static const char* Driving;
        static const char* Sidewalk;
        static const char* Walking;
        static const char* Unknown;

        static const char* Start;
        static const char* End;

        static const char* Road;
        static const char* Junction;

        static const char* arcLength;
        static const char* normalized;

        static const char* mph;
        static const char* ms;
        static const char* kmh;

        static const char* Plus;
        static const char* Minus;
        static const char* None;

        static const char* False;
        static const char* True;

        static const char* LHT;
        static const char* RHT;

        static const char* Bicycle;
        static const char* LowSpeed;
        static const char* Motorway;
        static const char* Pedestrian;
        static const char* Rural;
        static const char* TownArterial;
        static const char* TownCollector;
        static const char* TownExpressway;
        static const char* TownLocal;
        static const char* TownPlayStreet;
        static const char* TownPrivate;
        static const char* Town;

        static const char* Map;

        static const char* Maximum;
    };


    class geometry
    {
    public:
        geometry()
        {
            g = Attr::Geometry::none;
            s = 0; x = 0; y = 0; hdg = 0; length = 0;
            curvature = 0; curvStart = 0; curvEnd = 0;
            aU = 0; bU = 0; cU = 0; dU = 0;
            aV = 0; bV = 0; cV = 0; dV = 0;
            bz0x = 0; bz0y = 0; bz1x = 0; bz1y = 0;
            bz2x = 0; bz2y = 0; bz3x = 0; bz3y = 0;
            pRange = Attr::ParamPoly3Range::none;
        }

        void print()
        {
            if (g == Attr::Geometry::line)
                std::cout << "line: " << std::endl;
            if (g == Attr::Geometry::arc)
                std::cout << "arc: " << std::endl;
            if (g == Attr::Geometry::spiral)
                std::cout << "spial: " << std::endl;
            if (g == Attr::Geometry::paramPoly3)
                std::cout << "paramPoly3: " << std::endl;
            if (g == Attr::Geometry::bezier3)
                std::cout << "bezier3:" << std::endl;
            std::cout << "s: " << s << ", x: " << x << ", y: " << y << std::endl;
            std::cout << "hdg: " << hdg << ", length: " << length << std::endl;
            if (g == Attr::Geometry::arc)
                std::cout << "curvature: " << curvature << std::endl;
            else if (g == Attr::Geometry::spiral)
                std::cout << "curvStart: " << curvStart	 << ", curvEnd: " << curvEnd << std::endl;
            else if (g == Attr::Geometry::paramPoly3)
            {
                std::cout << "aU: " << aU << ", bU: " << bU << ", cU: " << cU << ", dU: " << dU << std::endl;
                std::cout << "aV: " << aV << ", bV: " << bV << ", cV: " << cV << ", dV: " << dV << std::endl;
                if (pRange == Attr::ParamPoly3Range::arcLength)
                    std::cout << "pRange: arcLength" << std::endl;
                if (pRange == Attr::ParamPoly3Range::normalized)
                    std::cout << "pRange: normalised" << std::endl;
            }
            else if (g == Attr::Geometry::bezier3)
            {
                std::cout << "bz0x: " << bz0x << ", bz0y: " << bz0y << ", bz1x: " << bz1x << ", bz1y: " << bz1y << std::endl;
                std::cout << "bz2x: " << bz2x << ", bz2y: " << bz2y << ", bz3x: " << bz3x << ", bz3y: " << bz3y << std::endl;
            }

        }
    public:
        Attr::Geometry g; ///< shape
        double s;      ///< initial s
        double x;      ///< initial x
        double y;      ///< initial y
        double hdg;    ///< rad ]-inf:inf[ ; initial heading
        double length; ///< length!

        double curvature; ///< arc parameter; constant curvature; 1/m; -inf:inf; positive for counter-clockwise curves.

        double curvStart; ///< spiral parameter; curvature at the start of the element; 1/m
        double curvEnd;   ///< spiral parameter; curvature at the end of the element; 1/m

        double aU, bU, cU, dU; ///< m, 1/m, 1/m^2, 1/m^3
        double aV, bV, cV, dV; ///< m, 1/m, 1/m^2, 1/m^3

        double bz0x, bz0y, bz1x, bz1y; ///< metres for our extended version using Beizer3
        double bz2x, bz2y, bz3x, bz3y; ///< metres for our extended version using Bezier3.
        Odr::Attr::ParamPoly3Range pRange; ///< range for the parameter of the parametric curve.
    };


    struct laneLink ///< lane link for junctions
    {
        int from;
        int to;
    };

    struct connection /// road link for junctions
    {
        int id;
        int incomingRoad;
        int connectingRoad;
        int contactPoint; ///< 0 for start, 1 for end;
        std::vector<laneLink> llink;
    };


    class offset
    {
    public:
        enum class LR {R, L, RL};

        offset()
        {
            a = 0; b = 0; c = 0; d = 0; s = 0; se = 0; seSet = false; lr = LR::RL;
        };
        offset(double ai, double bi, double ci, double di, double si)
        {
            a = ai; b = bi; c = ci; d = di; s = si; se = 0; seSet = false; lr = LR::RL;
        }
        offset(double ai, double bi, double ci, double di, double si, double sei)
        {
            a = ai; b = bi; c = ci; d = di; s = si; se = sei; seSet = true; lr = LR::RL;
        }
        offset(double ai, double bi, double ci, double di, double si, double sei, LR lri)
        {
            a = ai; b = bi; c = ci; d = di; s = si; se = sei; seSet = true; lr = lri;
        }

        offset(const offset &o)
        {
            a = o.a; b = o.b; c = o.c; d = o.d; s = o.s; se = o.se; seSet = o.seSet; lr = o.lr;
        }

        offset operator= (const offset &o)
        {
            a = o.a; b = o.b; c = o.c; d = o.d; s = o.s; se = o.se; seSet = o.seSet; lr = o.lr;
            return *this;
        }

        offset operator+ (const offset &o) const
        {
            return offset(a + o.a, b + o.b, c + o.c, d + o.d, s, se, lr);
        }
        void operator+= (const offset &o)
        {
            a += o.a; b += o.b; c += o.c; d += o.d;
        }
        bool operator== (const offset &o) const
        {
            if (!mvf::areSameValues(a, o.a)) return false;
            if (!mvf::areSameValues(b, o.b)) return false;
            if (!mvf::areSameValues(c, o.c)) return false;
            if (!mvf::areSameValues(d, o.d)) return false;
            if (!mvf::areSameValues(s, o.s)) return false;
            if (!mvf::areSameValues(se, o.se)) return false;
            if (seSet != o.seSet) return false;
            if (lr != o.lr) return false;
            return true;
        }
        bool operator<= (const offset &o) const
        {
            if (s <= o.s) return true;
            return false;
        }
        bool operator< (const offset &o) const
        {
            if (s < o.s) return true;
            return false;
        }
        bool operator> (const offset &o) const
        {
            if (s > o.s) return true;
            return false;
        }
        bool operator>= (const offset &o) const
        {
            if (s >= o.s) return true;
            return false;
        }

        offset operator* (scalar f) const
        {
            return offset(a * f, b * f, c * f, d * f, s, se, lr);
        }
        friend offset operator* (scalar f, offset &o)
        {
            return o * f;
        }
        void print() const
        {
            std::cout << "a: " << a
                      << ", b: " << b
                      << ", c: " << c
                      << ", d: " << d
                      << ", s: " << s
                      << ", se: " << se << ", seSet: " << std::to_string(seSet)
                      << ", lr: " << static_cast<int>(lr) << std::endl;
        }
        bool isConstant() const
        {
            if (!mvf::areSameValues(b,0)) return false;
            if (!mvf::areSameValues(c,0)) return false;
            if (!mvf::areSameValues(d,0)) return false;
            return true;
        }
        bool isNull() const
        {
            if (!mvf::areSameValues(a,0)) return false;
            if (!mvf::areSameValues(b,0)) return false;
            if (!mvf::areSameValues(c,0)) return false;
            if (!mvf::areSameValues(d,0)) return false;
            return true;
        }
        bool inRange(scalar t, scalar la = 0, scalar ra = 0) const ///< with left and right accuracies.
        {
            if (!seSet) return false;
            if (lr == LR::RL)
            {
                if (mvf::isInRangeLR(t, s, se, 0))
                    return true;
                else if (mvf::areCloseEnough(t, se, ra))
                    return true;
                else
                    return mvf::areCloseEnough(t, s, la);
            }
            else if (lr == LR::L)
                return mvf::isInRangeL(t, s, se, la);
            else if (lr == LR::R)
                return mvf::isInRangeR(t, s, se, ra);
            else
                return mvf::isInRange0(t, s, se);
        }
        static std::vector<offset> simplify(const std::vector<offset> &v);
        double a, b, c, d, s, se;
        LR lr; ///< whether to use L: [s, se), R: (s, se] or LR: [s, se]
        bool seSet;
    };

    class tsign
    {
    public:
        tsign()
        {
            s = 0; t = 0;
            zOffset = 0; value = 0;
            height = 0; width = 0;
            hOffset = 0; pitch = 0; roll = 0;
            orientation = 4;
            fromLane = -100;
            toLane = 100;
            dynamic = false;
            id = ""; name = ""; unit = ""; text = "";
        }
    public:
        double s; ///< s coordinate
        double t; ///< t coordinate
        double zOffset; ///< vertical clearance of the object
        double value; ///< [opt] value of the signal;
        double height; ///< height of the signal, measured from bottom edge
        double width; ///< width of the signal
        double hOffset; ///< heading of the signal relative to orientation if it's + or -, or to the reference line otherwise.
        double pitch, roll; ///< opt.
        int orientation; ///< +1 for positive s-direction, -1 for negative s-direction, and 0 for valid in both directions.
        int fromLane, toLane; ///< validity
        bool dynamic; ///< whether it is static or dynamic (e g, traffic light)
        std::string id; ///< Unique ID within the OpenDRIVE file
        std::string name; ///< Name of the signal. May be chosen freely. 
        std::string unit; ///< [opt] but mandatory if value is given (e_unit)
        std::string text; ///< more text.
    };



    class smaL  ///< smartActors Lane
    {
    public:
        smaL()
        {
            id = 0; sID = 0; ndxLS = 0;  odrID = 0; sign = 0;
            length = 0; speed = 0; startingS = 0;
            kind = "";
        }

        void writeXML(tinyxml2::XMLElement *elem, tinyxml2::XMLDocument &doc) const;
        void writeXMLWidth(tinyxml2::XMLElement *elem, tinyxml2::XMLDocument &doc) const;
        void writeXMLBorder(tinyxml2::XMLElement *elem, tinyxml2::XMLDocument &doc) const;


    public:
        uint id;
        uint sID; ///< section ID; I know.
        uint ndxLS; ///< lane section index.
        int odrID; ///< positive and ascending on the left
        int sign; ///< -1, 0, or 1;
        double length;
        double speed;
        double startingS; ///< given by the laneSection.
        std::vector<smaL*> prevLane;
        std::vector<smaL*> nextLane;
        std::vector<Odr::offset> w; ///< width of the lane; -- that should be a vector.
        std::vector<Odr::offset> border; ///< sometimes there's a border.
        std::string kind;
    };

    class smaS ///< smartActors Section
    {
    public:
        smaS()
        {
            id = 0; odrID = 0; lsSize = 0; name = "";
        }
    public:
        std::vector<smaL> lanes;
        uint id;
        uint odrID;
        uint lsSize; ///< number of lane sections
        std::string name;
        std::string type; ///< to be turned into a vector of types, which have the starting s, and the type itself.
        std::string rule; ///< 1.8 lht or rht!
        std::vector<Odr::geometry> geom;
        std::vector<Odr::offset> loffset;
        std::vector<Odr::tsign> tsigns;
    };

    class udIndexed6DPoint
    {
    public:
        udIndexed6DPoint()
        {
            px = 0; py = 0; pz = 0;
            rx = 0; ry = 0; rz = 0;
            id = 0;
        }
    public:
        double px, py, pz;
        double rx, ry, rz;
        int id;
    };

    class speedRegulation
    {
    public:
        std::string roadType; ///< the road type that this regulation applies to
        std::string type; ///< maximum, recommended...
        double value; ///< value of the speed;
        std::string unit; ///<  (e_unit)
    };


// public:
    extern std::string geomString(const Odr::Attr::Geometry &g);
    extern bool isRoadTypeValid(const std::string &rt);

};

}

#endif // ODRONES_ODR_H

