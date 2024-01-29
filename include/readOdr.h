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

#ifndef READODR_H
#define READODR_H

#include <iostream>
#include <string>
#include <vector>
#include <utility>
#include <algorithm>

#include "tinyxml2.h"
#include "matvec.h"

class Odr
{
public:
    class Elem
    {
    public:
        static const char* OpenDrive;
        static const char* Road;
        static const char* PlanView;
        static const char* Lane;
        static const char* Lanes;
        static const char* LaneOffset;
        static const char* LaneSection;

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

        static const char* Width;
        static const char* Speed;
        static const char* Border;

        static const char* Signals;
        static const char* Signal;
        static const char* Validity;
        static const char* UserData;
        static const char* Dependency;
    };

    class Attr
    {
    public:
        static const char* Type;
        static const char* Name;
        static const char* Id;
        static const char* Length;

        static const char* IncomingRoad;
        static const char* ConnectingRoad;
        static const char* ContactPoint;
        static const char* From;
        static const char* To;

        static const char* SingleSide;

        static const char* ElementType;
        static const char* ElementId;

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

        enum class Geometry {line, spiral, arc, paramPoly3, none};
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
            pRange = Attr::ParamPoly3Range::none;
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
            id = 0; odrID = 0; lsSize = 0;
        }
    public:
        std::vector<smaL> lanes;
        uint id;
        uint odrID;
        uint lsSize; ///< number of lane sections
        std::vector<Odr::geometry> geom;
        std::vector<Odr::offset> loffset;
        std::vector<Odr::tsign> tsigns;
    };


public:
    static std::string geomString(const Odr::Attr::Geometry &g);


};

class ReadOdr
{
public:
    /*! Constructor where iFile is either the input file name (isOdrFile == true) or
     *                                    the text of the file itself (isOdrFile == false) */
    ReadOdr(std::string iFile, bool isOdrFile);

    void printRoads(); ///< print out the roads

    bool isReady(); ///< whether it has the sections ready or not.

    const std::vector<Odr::smaS> &sections = _sections; ///< share a read-only version of _sections.

public:
    static constexpr scalar defaultSpeed = 30 * constants::mphToMs; ///< default speed is 30 mph.

private:
    int loadXodr(std::string iFile, bool isOdrFile); ///< return non-zero in case of error.

    void XMLCheckResult(int a_eResult); ///< print out whether there was an XML error.

    /*! return the point to a lane that has the required OpenDRIVE IDs */
    Odr::smaL* getLaneWithODRIds(uint rOdrID, int lOdrID, int lsID);

    /*! Read and return a Junction, given the Junction XMLElement */
    std::vector<Odr::connection> readJunction(tinyxml2::XMLElement *c);

    /*! Read all the geometry of an XML Road into a vector */
    std::vector<Odr::geometry> readGeometry(tinyxml2::XMLElement *pv);

    /*! Read all the laneOffset the lanes have given a Lanes XMLElement */
    std::vector<Odr::offset> readLaneOffset(tinyxml2::XMLElement *lanes);

    /*! Read all the Signals gien a Signals XMLElement */
    std::vector<Odr::tsign> readTrafficSigns(tinyxml2::XMLElement *xmlsgns);

    /*! return true if there's some laneOffset in the Lanes XMLElement */
    bool hasLaneOffset(tinyxml2::XMLElement *lanes);

    /*! return true if anything apart from a is non-zero in the offset */
    bool hasComplicatedOffset(Odr::offset &o);

    /*! Get the LinkID and Link Connection Point out of a Road Link XMLElement */
    int getRoadLinkData(uint &rLinkID, uint &rLinkCP, tinyxml2::XMLElement *fbLinkXML);

    /*! Given the lane XMLElement, consider adding a lane to the section, in ndxL
     *   with laneSection index ndxLS and starting at startingS.
     *  Increase +1 ndxL if so, return negative in case of error */
    int addLane(tinyxml2::XMLElement *road, tinyxml2::XMLElement *lane, uint ndxS, uint ndxL, uint ndxLS, double startingS);

    /*! Add previous and next lanes */
    uint linkLanes(tinyxml2::XMLElement *lXML, uint ndxS, uint ndxL, uint rPrevID, uint rNextID); //, int rPrevCP, int rNextCP);

private:
    std::vector<Odr::smaS> _sections;
    bool ready;



};


#endif // READODR_H
