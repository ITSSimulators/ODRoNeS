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

#ifndef READONEVERSION_H
#define READONEVERSION_H

// Only System and ODRoNeS headers should be here.
#include <iostream>
#include <string>
#include <vector>
#include <array>

typedef unsigned int uint;



/*! The classes OneVersion and readOneVersion are meant
 *   to read OneVersion aka Simulator* roads from binary files and
 *   to store this into some data structure that can be accessed later.
 *  They need access to the headers of OneVersion and in order
 *  to avoid any definition conflict,
 *  it should not load any of the ODRoNeS headers */
class OneVersion
{
public:
    class OVID
    {
    public:
        OVID()
        {
          laneID = -1; lgIndex = -1;
          roadIDM = -1; roadIDm = -1;
          nnodeID = -1;
        };


        void assignInputToThis(const OVID& input)
        {
            laneID = input.laneID;
            lgIndex = input.lgIndex;
            roadIDM = input.roadIDM;
            roadIDm = input.roadIDm;
            nnodeID = input.nnodeID;
        };

        OVID(const OVID& input)
        {
            assignInputToThis(input);
        };

        std::string to_string() const
        {
            std::string id = std::to_string(nnodeID) + ":"
                    + std::to_string(roadIDM) + "."
                    + std::to_string(roadIDm) + ":"
                    + std::to_string(lgIndex) + ":"
                    + std::to_string(laneID);
            return id;
        };


        OVID& operator= (const OVID& input)
        {
            assignInputToThis(input);
            return *this;
        };


        bool operator== (const OVID& input) const
        {
            if (laneID != input.laneID) return false;
            if (lgIndex != input.lgIndex) return false;
            if (roadIDM != input.roadIDM) return false;
            if (roadIDm != input.roadIDm) return false;
            if (nnodeID != input.nnodeID) return false;
            return true;
        };

        bool operator!= (const OVID& input) const
        {
            return (!operator==(input));
        };

        bool sameRoadIDs(const OVID& input) const
        {
            if (roadIDM != input.roadIDM) return false;
            if (roadIDm != input.roadIDm) return false;
            return true;
        };

        bool validRoad() const
        {
            if (roadIDM == -1) return false;
            if (roadIDm == -1) return false;
            if (nnodeID == -1) return false;
            return true;
        }

        bool validLane()
        {
            if (laneID == -1) return false;
            if (lgIndex == -1) return false;
            if (roadIDM == -1) return false;
            if (roadIDm == -1) return false;
            if (nnodeID == -1) return false;
            return true;
        }


    public:
        int laneID;
        int lgIndex;
        int roadIDM;
        int roadIDm;
        int nnodeID;
    };

    class cartCoordType
    {
    public:
        cartCoordType()
        {
            x = 0, y = 0; z = 0;
            tx = 0; ty = 0; tz = 0;
        }

    public:
        double x, y, z;
        double tx, ty, tz;
    };


    enum class SegmentType // aka SegmentEnum
    { straight, circular, piecewiseLinear };

    class segment
    {
    public:
        segment()
        {
            start.x = 0; start.y = 0; start.z = 0;
            start.tx = 0; start.ty = 0; start.tz = 0;
            end.x = 0; end.y = 0; end.z = 0;
            end.tx =0; end.ty = 0; end.tz = 0;
            length = 0; cummulativeLength = 0;
            radius = 0;
            centreX = 0; centreY = 0; centreZ = 0;
            rightHand = 0;
        }
    public:
        cartCoordType start;
        cartCoordType end;
        double length;
        double cummulativeLength;
        double radius;                    ///< Circle: radius of curvature.
        double centreX, centreY, centreZ; ///< Circle: centre of the arc.
        int rightHand; ///< Circle: +1 for rightHand (clockwise?), -1 for leftHand
        SegmentType type;
    };

    class linearFunc
    {
    public:
        linearFunc() { a = 0; b = 0; };
        double evaluate( double t ) { return a * t + b; };

        double a; ///< ax + b
        double b; ///< ax + b
    };

    class curve3d
    {
    public:
        curve3d() {};

    public:
        std::vector<segment> segments;
        linearFunc centreFunction;
        linearFunc leftEdgeFunction;
        linearFunc rightEdgeFunction;

    };

    class smaL
    {
    public:
        smaL()
        {
            id = 0;
            nearsideLane = nullptr; offsideLane = nullptr;
            length = 0; width0 = 0; dir = 0;
            lgIndex = 0; lgLength = 0; lgStartDistance = 0;
        };
    public:
        uint id;
        OVID ovID, nearsideLaneOVID, offsideLaneOVID;
        smaL* offsideLane;
        smaL* nearsideLane;
        float length;
        float width0; ///< width at the origin
        uint lgIndex; ///< lane group index.
        float lgLength; ///< lane group length
        float lgStartDistance; ///< lane group start distance
        int dir; ///< +1 / -1 depending on wether the direction is the same to the curve
        OneVersion::curve3d curve;
        std::vector<smaL*> prevLane;
        std::vector<smaL*> nextLane;
    };

    class smaS
    {
    public:
        smaS()
        {
            id = 0; lgSize = 0;
            friction = 0; speedLimit = 0;
            forwardsNode = -1; backwardsNode = -1;
            startJunction = -1; endJunction = -1;
            junction = -1;
            // firstInNN = false; lastInNN = false;
        };

        std::string idString() const
        {
            return std::to_string(ovID.roadIDM) + "." + std::to_string(ovID.roadIDm);
        };

        bool hasForwardsNode() const
        {
            if (forwardsNode == -1) return false;
            return true;
        };

        bool hasBackwardsNode() const
        {
            if (backwardsNode == -1) return false;
            return true;
        };

        bool isJunction() const
        {
            if (junction == -1) return false;
            return true;
        };

    public:
        uint id;
        OVID ovID, forwardsRoadOVID, backwardsRoadOVID;
        int forwardsNode, backwardsNode, startJunction, endJunction;
        int junction; ///< -1 unless it belongs to a junction / junction.number otherwise.
        uint lgSize; ///< number of lane groups.
        float friction;
        float speedLimit;
        // bool lastInNN, firstInNN; ///< whether this section is the first (or not) or the last (or not) in the Network Node.
        std::vector<smaL> lanes;
    };
};

class readOneVersion
{

public:
    readOneVersion(std::string iFile);

    bool ready() { return _ready; };

public:
    std::vector<OneVersion::smaS> sections;

private:
    bool _ready;
};

#endif // READONEVERSION_H
