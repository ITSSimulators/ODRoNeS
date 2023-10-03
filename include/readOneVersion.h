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

#ifdef USE_ONEVERSION

// Only System and ODRoNeS headers should be here.
#include <iostream>
#include <string>
#include <vector>



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

        OVID& operator= (const OVID& input)
        {
            laneID = input.laneID;
            lgIndex = input.lgIndex;
            roadIDM = input.roadIDM;
            roadIDm = input.roadIDm;
            nnodeID = input.nnodeID;
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
            xp = 0; yp = 0; zp = 0;
        }

    public:
        double x, y, z;
        double xp, yp, zp;
    };


    enum class SegmentType // aka SegmentEnum
    { straight, circular, piecewiseLinear };

    class segment
    {
    public:
        segment()
        {
            start.x = 0; start.y = 0; start.z = 0;
            start.xp = 0; start.yp = 0; start.zp = 0;
            end.x = 0; end.y = 0; end.z = 0;
            end.xp =0; end.yp = 0; end.zp = 0;
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

    };

    class smaL
    {
    public:
        smaL()
        {
            id = 0;
            nearsideLane = nullptr; offsideLane = nullptr;
            length = 0; dir = 0;
            lgIndex = 0; lgLength = 0; lgStartDistance = 0;
        };
    public:
        uint id;
        OVID ovID, nearsideLaneOVID, offsideLaneOVID;
        smaL* offsideLane;
        smaL* nearsideLane;
        float length;
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
        };
        std::string idString() const
        {
            return std::to_string(ovID.roadIDM) + "." + std::to_string(ovID.roadIDm);
        };
    public:
        uint id;
        OVID ovID, forwardsRoadOVID, backwardsRoadOVID;
        int forwardsNode, backwardsNode, startJunction, endJunction;
        int junction; ///< -1 unless it belongs to a junction / junction.number otherwise.
        uint lgSize; ///< number of lane groups.
        float friction;
        float speedLimit;
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

#endif // USE_ONEVERSION

#endif // READONEVERSION_H
