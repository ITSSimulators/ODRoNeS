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


#ifdef USE_ONEVERSION

#include "readOneVersion.h"

readOneVersion::readOneVersion(std::string iFile) :
    _ready(false)
{
    _ovn = new LogicalRoadNetwork(iFile);
    if (!_ovn)
    {
        std::cout << "[ Error ] OneVersion's LogicalRoadNetwork failed to load!" << std::endl;
        return;
    }

    std::cout << "-- There are " << _ovn->getNetNodeCount() << " network nodes and "
              << _ovn->getJunctionCount() << " junctions in " << iFile << std::endl;

    // Loop over every possible lane, and add its details to the right place.
    //  For every network node
    //    For every road node
    //      For every LaneGroup in RN.laneGroup.array
    //        create a section, and put in there the lanes within the laneGroup.
    uint secID = 0;
    // For Every Network Node:
    for (uint i = 1; i <= _ovn->getNetNodeCount(); ++i) // who did that 1-based?!
    {
        NetworkNode *nn = _ovn->getNetNode(i);
        nn->getRoadList();
        const RoadNodeArray rna = nn->getRoadList();
        // For Every Road Node:
        for (uint j = 0; j < rna.size(); j++)
        {
            secID += 1;
            // std::cout << "here's a new section: " << secID << std::endl; // new section.
            // For Every Lane Group:
            for (uint k = 0; k < rna[j]->numLaneGroups(); ++k)
            {
                // std::cout << "here's a new lane group: " << k << std::endl; // mark it.
                const LaneGroup *lg = rna[j]->laneGroupForIndex(k);
                // For Every Lane:
                for (uint l = 0; l < lg->numLanes(); ++l)
                {
                    std::cout << "here's lane: " << secID << ":" << l << std::endl;
                    const Lane* lane = lg->laneForIndex(l);
                    if (lane->laneType() == Lane::Patch)
                    {
                        std::cout << "this lane is a patch (standard)" << std::endl;
                        // use centreRefCurve:
                        const geom::Curve* curve = lane->centreRefCurve();
                        for (uint m = 0; m < curve->numSegments(); ++m)
                        {
                            std::cout << "this is segment " << m << std::endl;
                            if (curve->segmentType(m) == geom::Curve::StraightSegment)
                            {
                                std::cout << "this segment is straight" << std::endl;
                                curve->segmentAttribute(m, geom::Curve::Length);
                                curve->segmentAttribute(m, geom::Curve::CumulativeLength);

                                geom::Curve::CartCoordType start;
                                curve->getPointAtIndex(m, start);

                                geom::Curve::CartCoordType end;
                                curve->getPointAtIndex(m+1, end);
                            }
                            else if (curve->segmentType(m) == geom::Curve::CircularSegment)
                            {
                                std::cout << "this segment is an arc" << std::endl;
                                curve->segmentAttribute(m, geom::Curve::Length);
                                curve->segmentAttribute(m, geom::Curve::CumulativeLength);
                                curve->segmentAttribute(m, geom::Curve::Radius);
                                curve->segmentAttribute(m, geom::Curve::RightHand);
                                curve->segmentAttribute(m, geom::Curve::CentreX);
                                curve->segmentAttribute(m, geom::Curve::CentreY);
                                curve->segmentAttribute(m, geom::Curve::CentreZ);

                                geom::Curve::CartCoordType start;
                                curve->getPointAtIndex(m, start);

                                geom::Curve::CartCoordType end;
                                curve->getPointAtIndex(m+1, end);
                            }
                            else if (curve->segmentType(m) == geom::Curve::PiecewiseLinearSegment)
                            {
                                // it looks like it's just a straight,
                                //    and you have lots of straights...
                                std::cout << "this segment is piecewise linear" << std::endl;
                                curve->segmentAttribute(m, geom::Curve::Length);
                                curve->segmentAttribute(m, geom::Curve::CumulativeLength);

                                // Get all the points from "m" to the end.
                                curve->numPoints();
                                geom::Curve::CartCoordType start;
                                curve->getPointAtIndex(m, start);

                                geom::Curve::CartCoordType end;
                                curve->getPointAtIndex(m, end);

                            }
                            else
                            {
                                std::cout << "This segment is not yet supported!" << std::endl;
                            }
                        }
                    }
                    else if (lane->laneType() == Lane::ContraFlow)
                    {
                        std::cout << "ContraFlow lanes are not supported yet!" << std::endl;
                    }
                    else if (lane->laneType() == Lane::VaryingWidth)
                    {
                        std::cout << "VaryingWidth lanes are not supported yet!" << std::endl;
                    }
                    else
                    {
                        std::cout << "Lane Type unknown!" << std::endl;
                    }
                }
            }
        }

    }

}



#endif // USE_ONEVERSION
