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

#include "constants.h"
#include "matvec.h"

#include <vector>
#include <string>
#include <iostream>

namespace tinyxml2
{
    class XMLDocument;
    class XMLElement;
}

namespace odrones 
{

namespace Odr
{

    class Elem
    {
    public:
        constexpr static const char* OpenDrive {"OpenDRIVE"} ;
        constexpr static const char* Header {"header"};
        constexpr static const char* Road {"road"};
        constexpr static const char* Type {"type"};
        constexpr static const char* PlanView {"planView"};
        constexpr static const char* Lane {"lane"};
        constexpr static const char* Lanes {"lanes"};
        constexpr static const char* LaneOffset {"laneOffset"};
        constexpr static const char* LaneSection {"laneSection"};

        constexpr static const char* DefaultRegulations {"defaultRegulations"};
        constexpr static const char* RoadRegulations {"roadRegulations"};
        constexpr static const char* Semantics {"semantics"};

        constexpr static const char* Junction {"junction"};
        constexpr static const char* Connection {"connection"};
        constexpr static const char* LaneLink {"laneLink"};

        constexpr static const char* Link {"link"};
        constexpr static const char* Predecessor {"predecessor"};
        constexpr static const char* Successor {"successor"};

        constexpr static const char* Left {"left"};
        constexpr static const char* Center {"center"};
        constexpr static const char* Right {"right"};

        constexpr static const char* Geometry {"geometry"};
        constexpr static const char* Line {"line"};
        constexpr static const char* Arc {"arc"};
        constexpr static const char* Spiral {"spiral"};
        constexpr static const char* ParamPoly3 {"paramPoly3"};
        constexpr static const char* Bezier3 {"bezier3"};

        constexpr static const char* Width {"width"};
        constexpr static const char* Speed {"speed"};
        constexpr static const char* Border {"border"};

        constexpr static const char* Signals {"signals"};
        constexpr static const char* Signal {"signal"};
        constexpr static const char* Validity {"validity"};
        constexpr static const char* UserData {"userData"};
        constexpr static const char* Dependency {"dependency"};

        constexpr static const char* UDConnectionPoints {"udConnectionPoints"};
        constexpr static const char* UDConnectionPoint {"udConnectionPoint"};
    };

    class Attr
    {
    public:
        constexpr static const char* Type {"type"};
        constexpr static const char* Name {"name"};
        constexpr static const char* Id {"id"};
        constexpr static const char* Junction {"junction"};
        constexpr static const char* Length {"length"};
        constexpr static const char* Rule {"rule"};

        constexpr static const char* IncomingRoad {"incomingRoad"};
        constexpr static const char* ConnectingRoad {"connectingRoad"};
        constexpr static const char* ContactPoint {"contactPoint"};
        constexpr static const char* From {"from"};
        constexpr static const char* To {"to"};

        constexpr static const char* SingleSide {"singleSide"};

        constexpr static const char* ElementType {"elementType"};
        constexpr static const char* ElementId {"elementId"};

        constexpr static const char* Level {"level"};

        constexpr static const char* Curvature {"curvature"};
        constexpr static const char* CurvStart {"curvStart"};
        constexpr static const char* CurvEnd {"curvEnd"};
        constexpr static const char* S {"s"};
        constexpr static const char* T {"t"};
        constexpr static const char* X {"x"};
        constexpr static const char* Y {"y"};
        constexpr static const char* Hdg {"hdg"}; 

        constexpr static const char* sOffset {"sOffset"};
        constexpr static const char* A {"a"};
        constexpr static const char* B {"b"};
        constexpr static const char* C {"c"};
        constexpr static const char* D {"d"};

        constexpr static const char* Max {"max"};

        enum class Geometry {line, spiral, arc, paramPoly3, bezier3, none};
        enum class ParamPoly3Range {arcLength, normalized, none};

        constexpr static const char* aU {"aU"};
        constexpr static const char* bU {"bU"};
        constexpr static const char* cU {"cU"};
        constexpr static const char* dU {"dU"};
        constexpr static const char* aV {"aV"};
        constexpr static const char* bV {"bV"};
        constexpr static const char* cV {"cV"};
        constexpr static const char* dV {"dV"};
        constexpr static const char* pRange {"pRange"};

        constexpr static const char* bz0x {"bz0x"};
        constexpr static const char* bz0y {"bz0y"};
        constexpr static const char* bz1x {"bz1x"};
        constexpr static const char* bz1y {"bz1y"};
        constexpr static const char* bz2x {"bz2x"};
        constexpr static const char* bz2y {"bz2y"};
        constexpr static const char* bz3x {"bz3x"};
        constexpr static const char* bz3y {"bz3y"};

        constexpr static const char* rZ {"rz"};

        constexpr static const char* Unit {"unit"};

        constexpr static const char* Dynamic {"dynamic"};
        constexpr static const char* Orientation {"orientation"};
        constexpr static const char* ZOffset {"zOffset"};
        constexpr static const char* Value {"value"};
        constexpr static const char* Height {"height"};
        constexpr static const char* Width {"width"};
        constexpr static const char* Text {"text"};
        constexpr static const char* HOffset {"hOffset"};
        constexpr static const char* Pitch {"pitch"};
        constexpr static const char* Roll {"roll"};
        constexpr static const char* FromLane {"fromLane"};
        constexpr static const char* ToLane {"toLane"};


    };

    class Kind
    {
    public:
        constexpr static const char* Driving {"driving"};
        constexpr static const char* Sidewalk {"sidewalk"};
        constexpr static const char* Walking {"walking"};

        constexpr static const char* Shoulder {"shoulder"};
        constexpr static const char* Border {"border"};
        constexpr static const char* Stop {"stop"};
        constexpr static const char* Restricted {"restricted"};
        constexpr static const char* Parking {"parking"};
        constexpr static const char* Median {"median"};
        constexpr static const char* Biking {"biking"};
        constexpr static const char* Curb {"curb"};
        constexpr static const char* Entry {"entry"};
        constexpr static const char* Exit {"exit"};
        constexpr static const char* OnRamp {"onRamp"};
        constexpr static const char* OffRamp {"offRamp"};
        constexpr static const char* ConnectingRamp {"connectingRamp"};
        constexpr static const char* SlipLane {"slipLane"};
        constexpr static const char* Unknown {"unknown"};

        enum class LaneType {driving, sidewalk, walking, shoulder, border, stop,
                              restricted, parking, median, biking, curb, entry,
                              exit, onRamp, offRamp, connectingRamp, slipLane, unknown };

        inline static const std::vector<LaneType> laneTypeV { LaneType::driving,
                LaneType::sidewalk, LaneType::walking, LaneType::shoulder,
                LaneType::border, LaneType::stop, LaneType::restricted,
                LaneType::parking, LaneType::median, LaneType::biking, LaneType::curb,
                LaneType::entry, LaneType::exit, LaneType::onRamp, LaneType::offRamp,
                LaneType::connectingRamp, LaneType::slipLane };

        constexpr static const char* Start {"start"};
        constexpr static const char* End {"end"};

        constexpr static const char* Road {"road"};
        constexpr static const char* Junction {"junction"};

        constexpr static const char* arcLength {"arcLength"};
        constexpr static const char* normalized {"normalized"};

        constexpr static const char* mph {"mph"};
        constexpr static const char* ms {"m/s"};
        constexpr static const char* kmh {"km/h"};

        constexpr static const char* Plus {"+"};
        constexpr static const char* Minus {"-"};
        constexpr static const char* None {"none"};

        constexpr static const char* False {"false"};
        constexpr static const char* True {"true"};

        constexpr static const char* LHT {"LHT"};
        constexpr static const char* RHT {"RHT"};

        constexpr static const char* Bicycle {"bicycle"};
        constexpr static const char* LowSpeed {"lowSpeed"};
        constexpr static const char* Motorway {"motorway"};
        constexpr static const char* Pedestrian {"pedestrian"};
        constexpr static const char* Rural {"rural"};
        constexpr static const char* TownArterial {"townArterial"};
        constexpr static const char* TownCollector {"townCollector"};
        constexpr static const char* TownExpressway {"townExpressway"};
        constexpr static const char* TownLocal {"townLocal"};
        constexpr static const char* TownPlayStreet {"townPlayStreet"};
        constexpr static const char* TownPrivate {"townPrivate"};
        constexpr static const char* Town {"town"};

        enum class RoadType { bicycle, lowSpeed, motorway, pedestrian, rural,
                              townArterial, townCollector, townExpressway, townLocal,
                              townPlayStreet, townPrivate, town, unknown };

        inline static const std::vector<RoadType> roadTypeV { RoadType::bicycle,
                              RoadType::lowSpeed, RoadType::motorway,
                              RoadType::pedestrian, RoadType::rural,
                              RoadType::townArterial, RoadType::townCollector,
                              RoadType::townExpressway, RoadType::townLocal,
                              RoadType::townPlayStreet, RoadType::townPrivate, RoadType::town};

        constexpr static const char* Map {"map"};

        constexpr static const char* Maximum {"maximum"};
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

        void zeroBezier()
        {
            bz0x = 0; bz0y = 0; bz1x = 0; bz1y = 0;
            bz2x = 0; bz2y = 0; bz3x = 0; bz3y = 0;
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

    class speedLimit
    {
    public:
        speedLimit()
        {
            value = 0; s = 0;
        }
    public:
        double value; ///< stored in m/s;
        double s; ///< starting s coordinate;
    };

    extern std::string geomString(const Odr::Attr::Geometry &g);
    extern bool isRoadTypeValid(const std::string &rt);
    extern Odr::Kind::RoadType roadTypeFromCString(const char* c);
    extern std::string roadTypeString(Odr::Kind::RoadType rt);
    extern Odr::Kind::LaneType laneTypeFromCString(const char* c);
    extern std::string laneTypeString(Odr::Kind::LaneType lt);


    class smaL  ///< smartActors Lane
    {
    public:
        smaL()
        {
            id = 0; sID = 0; ndxLS = 0;  odrID = 0; sign = 0;
            length = 0; startingS = 0;
            kind = "";
        }

        void writeXML(tinyxml2::XMLElement *elem, tinyxml2::XMLDocument &doc) const;
        void writeXMLWidth(tinyxml2::XMLElement *elem, tinyxml2::XMLDocument &doc) const;
        void writeXMLBorder(tinyxml2::XMLElement *elem, tinyxml2::XMLDocument &doc) const;
        std::string print() const
        {
            return "laneSection: " + std::to_string(ndxLS) + ", lane: " + std::to_string(odrID);
        };


    public:
        uint id;
        uint sID; ///< section ID; I know.
        uint ndxLS; ///< lane section index.
        int odrID; ///< positive and ascending on the left
        int sign; ///< -1, 0, or 1;
        double length;
        double startingS; ///< given by the laneSection.
        std::vector<speedLimit> speed;
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

        void setRoadType(const std::string &str)
        {
            type = roadTypeFromCString(str.c_str());
        }

        std::string print() const
        {
            return "road: " + std::to_string(odrID);
        }

        scalar length() const
        {
            scalar l = 0;
            for (uint i = 0; i < geom.size(); ++i)
                l += geom[i].length;
            return l;
        }

    public:
        std::vector<smaL> lanes;
        uint id;
        uint odrID;
        uint lsSize; ///< number of lane sections
        std::string name;
        Odr::Kind::RoadType type; ///< to be turned into a vector of types, which have the starting s, and the type itself.
        std::string rule; ///< 1.8 lht or rht!
        std::vector<Odr::geometry> geom;
        std::vector<Odr::offset> loffset;
        std::vector<Odr::tsign> tsigns;
    };

    class speedRegulation
    {
    public:
        Odr::Kind::RoadType roadType;  ///< the road type that this regulation applies to
        std::string type; ///< maximum, recommended...
        double value; ///< value of the speed;
    };


    class udIndexed6DPoint  ///< User Data (UD) indexed point.
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


};

}

#endif // ODRONES_ODR_H

