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

#include <iostream>
#include <string>
#include <vector>

// OneVersion headers:
#include "config/config.h"
#include "lrn/orig/LogicalRoadNetwork.h"
#include "lrn/orig/NetworkNode.h"


/*! The classes OneVersion and readOneVersion are meant
 *   to read OneVersion aka Simulator* roads from binary files and
 *   to store this into some data structure that can be accessed later.
 *  They need access to the headers of OneVersion and in order
 *  to avoid any definition conflict,
 *  it should not load any of the ODRoNeS headers */
class OneVersion
{
public:
    class curve3d
    {
    public:
        enum class SegmentType // aka SegmentEnum
        { straight, circular, picewiseLinear };

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

        class segment
        {
            segment()
            {
                startX = 0; startY = 0; startZ = 0;
                startXp = 0; startYp = 0; startZp = 0;
                endX = 0; endY = 0; endZ = 0;
                endXp = 0; endYp = 0; endZp = 0;
                length = 0; cummulativeLength = 0;
                radius = 0;
                centreX = 0; centreY = 0; centreZ = 0;
                rightHand = 0;
            }
        public:
            double startX, startY, startZ;
            double startXp, startYp, startZp;
            double endX, endY, endZ;
            double endXp, endYp, endZp;
            double length;
            double cummulativeLength;
            double radius;                    ///< Circle: radius of curvature.
            double centreX, centreY, centreZ; ///< Circle: centre of the arc.
            int rightHand; ///< Circle: +1 for rightHand (clockwise?), -1 for leftHand
        };

    public:
        curve3d() {};

    public:
        std::vector<SegmentType> segmentTypes;


    };

    class smaL
    {
    public:
        smaL()
        {
            id = 0; ovID = 0; ndxLG = 0;
        }
    public:
        uint id;
        uint ovID;
        uint ndxLG; ///< lane group index.
        std::vector<smaL*> prevLane;
        std::vector<smaL*> nextLane;


    };

    class smaS
    {
    public:
        smaS()
        {
            id = 0; ovID = 0; lgSize = 0;
        }
    public:
        uint id;
        uint ovID;
        uint lgSize; ///< number of lane groups.
        std::vector<smaL> lanes;
    };
};

class readOneVersion
{

public:
    readOneVersion(std::string iFile);

    bool ready() { return _ready; };

private:
    LogicalRoadNetwork *_ovn;
    bool _ready;
};

#endif // USE_ONEVERSION

#endif // READONEVERSION_H
