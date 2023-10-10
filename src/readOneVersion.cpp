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

#include "readOneVersion.h"

#ifdef USE_ONEVERSION

// OneVersion headers:
#include "config/config.h"
#include "lrn/orig/LogicalRoadNetwork.h"
#include "lrn/orig/NetworkNode.h"
#include "lrn/orig/LogicalJunc.h"

typedef std::array<double,2> arr2;

// // // // Static methods using the external OneVersion headers // // // //
class OneVersionStatic
{
public:
    static OneVersion::cartCoordType convert(const geom::Curve::CartCoordType &c)
    {
        OneVersion::cartCoordType here;

        here.x = c.xyz.x();
        here.y = c.xyz.y();
        here.z = c.xyz.z();

        here.tx = c.tangent.x();
        here.ty = c.tangent.y();
        here.tz = c.tangent.z();

        return here;
    }

    static void setOVID(OneVersion::OVID& id, const Lane* l)
    {
        if (l)
        {
            id.laneID = l->index();
            id.lgIndex = l->group()->index();
            id.roadIDM = l->road()->getRoadID().major();
            id.roadIDm = l->road()->getRoadID().minor();
            id.nnodeID = l->road()->parentNode()->id();
        }
        else
        {
            id.laneID = -1;
            id.lgIndex = -1;
            id.roadIDM = -1;
            id.roadIDm = -1;
            id.nnodeID = -1;
        }
    }

    static void setOVID(OneVersion::OVID& id, const RoadNode* r)
    {
        id.laneID = -1;
        id.lgIndex = -1;
        if (r)
        {
            id.roadIDM = r->getRoadID().major();
            id.roadIDm = r->getRoadID().minor();
            id.nnodeID = r->parentNode()->id();
        }
        else
        {
            id.roadIDM = -1;
            id.roadIDm = -1;
            id.nnodeID = -1;
        }
    }

    static arr2 roadToCart(double distance, double offset, double loft, const Lane* l)
    {
        arr2 xy = {0, 0};

        if (l->laneType() == Lane::Patch)
        {
            geom::CartCoord cartCoord;
            geom::CurveCoord offsetCoord;
            offsetCoord.distance = distance + l->group()->startDistance();
            offsetCoord.offset = offset + l->offsetForDistance(offsetCoord.distance);
            l->centreRefCurve()->curveToCartesian(offsetCoord, cartCoord);

            xy = {cartCoord.xyz.x(), cartCoord.xyz.y()};
        }

        return xy;
    }

    static int populateLane(OneVersion::smaL& smal, uint id, const Lane* l)
    {
        int err = 0;

        smal.length = l->length();
        smal.lgLength = l->group()->length();
        smal.lgStartDistance = l->group()->startDistance();
        smal.dir = l->dir();
        setOVID(smal.ovID, l);
        if (l->nearsideLane())
            setOVID(smal.nearsideLaneOVID, l->nearsideLane());
        if (l->offsideLane())
            setOVID(smal.offsideLaneOVID, l->offsideLane());
        if (l->laneType() == Lane::Patch)
        {

            arr2 origin = roadToCart(0, 0, 0, l);
            arr2 end = roadToCart(smal.length, 0, 0, l);


            smal.curve.centreFunction.a = l->centreFunction().a();
            smal.curve.centreFunction.b = l->centreFunction().b();

            // If this is a Patch, then use centreRefCurve:
            const geom::Curve* curve = l->centreRefCurve();

            for (uint m = 0; m < curve->numSegments(); ++m)
            {
                smal.curve.segments.push_back(OneVersion::segment());

                OneVersion::segment *s = &(smal.curve.segments.back());
                s->length = curve->segmentAttribute(m, geom::Curve::Length);
                s->cummulativeLength = curve->segmentAttribute(m, geom::Curve::CumulativeLength);

                geom::Curve::CartCoordType c;
                curve->getPointAtIndex(m, c);
                s->start = convert(c);

                curve->getPointAtIndex(m+1, c);
                s->end = convert(c);

                if (curve->segmentType(m) == geom::Curve::StraightSegment)
                    s->type = OneVersion::SegmentType::straight;

                else if (curve->segmentType(m) == geom::Curve::CircularSegment)
                {
                    s->type = OneVersion::SegmentType::circular;

                    s->radius = curve->segmentAttribute(m, geom::Curve::Radius);
                    s->rightHand = curve->segmentAttribute(m, geom::Curve::RightHand);
                    s->centreX = curve->segmentAttribute(m, geom::Curve::CentreX);
                    s->centreY = curve->segmentAttribute(m, geom::Curve::CentreY);
                    s->centreZ = curve->segmentAttribute(m, geom::Curve::CentreZ);

                }
                else if (curve->segmentType(m) == geom::Curve::PiecewiseLinearSegment)
                {
                    // it looks like it's just a straight, and you have lots of these straights...
                    s->type = OneVersion::SegmentType::piecewiseLinear;

                    /*
                // Get all the points from "m" to the end.
                curve->numPoints();
                geom::Curve::CartCoordType start;
                curve->getPointAtIndex(m, start);

                geom::Curve::CartCoordType end;
                curve->getPointAtIndex(m, end);
                */
                }
                else
                {
                    std::cout << "This segment is not yet supported!" << std::endl;
                }
            }
        }
        else if (l->laneType() == Lane::ContraFlow)
        {
            std::cout << "ContraFlow lanes are not supported yet!" << std::endl;
        }
        else if (l->laneType() == Lane::VaryingWidth)
        {
            std::cout << "VaryingWidth lanes are not supported yet!" << std::endl;
        }
        else
        {
            std::cout << "Lane Type unknown!" << std::endl;
        }

        return err;

    }



    static void loadEverySectionAndLane(std::vector<OneVersion::smaS> &sections, const RoadNodeArray &rna)
    {
        // For Every Road Node:
        for (uint i = 0; i < rna.size(); i++)
        {
            sections.push_back(OneVersion::smaS());
            sections.back().id = sections.size() - 1;

            // Set all possible identifiers!
            setOVID(sections.back().ovID, rna[i]);
            if (!rna[i]->getForwardsRoad())
                std::cout << "section: " << sections.back().ovID.to_string() << " doesn't have a Road Forwards" << std::endl;
            setOVID(sections.back().forwardsRoadOVID, rna[i]->getForwardsRoad());
            if (!rna[i]->getBackwardsRoad())
                std::cout << "section: " << sections.back().ovID.to_string() << " doesn't have a Road Backwards" << std::endl;
            setOVID(sections.back().backwardsRoadOVID, rna[i]->getBackwardsRoad());
            if (rna[i]->parentNode()->forwardsNode())
                sections.back().forwardsNode = rna[i]->parentNode()->forwardsNode()->id();
            if (rna[i]->parentNode()->backwardsNode())
                sections.back().backwardsNode = rna[i]->parentNode()->backwardsNode()->id();
            if (rna[i]->parentNode()->startJunc())
                sections.back().startJunction = rna[i]->parentNode()->startJunc()->number();
            if (rna[i]->parentNode()->endJunc())
                sections.back().endJunction = rna[i]->parentNode()->endJunc()->number();
            if (rna[i]->parentNode()->junction())
                sections.back().junction = rna[i]->parentNode()->junction()->number();

            // Get some extra data:
            sections.back().friction = rna[i]->getFriction();
            sections.back().speedLimit = rna[i]->getSpeedLim();
            sections.back().lgSize = rna[i]->numLaneGroups();

            // Explore what is numLanes:
            // uint numLanes = rna[j]->getNumLanes(); // what is numLanes?
            // For Every Lane Group:
            uint laneID = 0;
            for (uint j = 0; j < rna[i]->numLaneGroups(); ++j)
            {
                // std::cout << "here's a new lane group: " << k << std::endl; // mark it.
                const LaneGroup *lg = rna[i]->laneGroupForIndex(j);
                // For Every Lane:
                for (uint k = 0; k < lg->numLanes(); ++k)
                {
                    // std::cout << "here's lane: " << .back()< ":" << l << std::endl;
                    sections.back().lanes.push_back(OneVersion::smaL());
                    populateLane(sections.back().lanes.back(), laneID, lg->laneForIndex(k));

                    if (sections.back().lanes.back().ovID.lgIndex !=
                            sections.back().lanes.back().lgIndex )
                    {
                        std::cout << "[ Error ] There's something we didn't understand" << std::endl;
                    }

                    laneID += 1;
                }
            }
        }
    }
};


// // // // End of Static methods // // // //

typedef OneVersionStatic OVS;


readOneVersion::readOneVersion(std::string iFile) :
    _ready(false)
{
    LogicalRoadNetwork *ovn = new LogicalRoadNetwork(iFile);
    if (!ovn)
    {
        std::cout << "[ Error ] OneVersion's LogicalRoadNetwork failed to load!" << std::endl;
        return;
    }

    std::cout << "-- There are " << ovn->getNetNodeCount() << " network nodes and "
              << ovn->getJunctionCount() << " junctions in " << iFile << std::endl;

    // Loop over every possible lane, and add its details to the right place, i e,
    // For Every Network Node
    //    For every Road Node
    //      For every LaneGroup in RN.laneGroup.array
    //        create a Section, and put in there the lanes within the LaneGroup.


    // For Every Network Node:
    for (uint i = 1; i <= ovn->getNetNodeCount(); ++i) // who did that 1-based?!
    {
        NetworkNode *nn = ovn->getNetNode(i);
        OVS::loadEverySectionAndLane(sections, nn->getRoadList());
    }


    // Now loop over the junctions:
    // For Every Junction
    for (uint i = 1; i <= ovn->getJunctionCount(); ++i) // that is 1-based too!
    {
        LogicalJunc *jn = ovn->getJunc(i);
        const NetworkNodeArray nna = jn->corridors();

        // And for every network node within that junction:
        for (uint j = 0; j < nna.size(); ++j)
            OVS::loadEverySectionAndLane(sections, nna[j]->getRoadList());
    }

    _ready = true;
}
#else
readOneVersion::readOneVersion(std::string iFile) : _ready(false) {}
#endif // USE_ONEVERSION
