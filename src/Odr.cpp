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

#include "Odr.h"
#include <algorithm>
#include <cstring>
using namespace odrones;


std::vector<Odr::offset> Odr::offset::simplify(const std::vector<offset> &v)
{
    std::vector<Odr::offset> off;
    // If the input vector v is empty, return a Zero offset:
    if (v.size() == 0)
    {
        off.push_back(Odr::offset());
        return off;
    }

    // Sort the offsets in "s" ascending order
    std::vector<std::pair<scalar, uint>> order;
    for (uint i = 0; i < v.size(); ++i)
        order.push_back(std::pair<scalar, uint>(v[i].s, i));
    std::sort(order.begin(), order.end());


    // Put all the data in order and flattened.
    off.push_back(v[order[0].second]);
    for (uint i = 1; i < order.size(); ++ i)
    {
        Odr::offset oi = v[order[i].second];
        if ((mvf::areSameValues(off.back().s, oi.s)) && (mvf::areSameValues(off.back().se, oi.se))
                && (off.back().lr == oi.lr))
            off.back() += oi;
        else
            off.push_back(oi);
    }

    return off;
}


std::string Odr::geomString(const Odr::Attr::Geometry &g)
{
    if (g == Odr::Attr::Geometry::line)
        return "line";
    else if (g == Odr::Attr::Geometry::arc)
        return "arc";
    else if (g == Odr::Attr::Geometry::spiral)
        return "spiral";
    else if (g == Odr::Attr::Geometry::paramPoly3)
        return "paramPoly3";
    else if (g == Odr::Attr::Geometry::bezier3)
        return "bezier3";
    else if (g == Odr::Attr::Geometry::none)
        return "none";
    else
        return "clueless!";
}

bool Odr::isRoadTypeValid(const std::string &rt)
{
    if ((strcmp(rt.c_str(), Odr::Kind::Bicycle) == 0) ||
            (strcmp(rt.c_str(), Odr::Kind::LowSpeed) == 0) ||
            (strcmp(rt.c_str(), Odr::Kind::Motorway) == 0) ||
            (strcmp(rt.c_str(), Odr::Kind::Pedestrian) == 0) ||
            (strcmp(rt.c_str(), Odr::Kind::Rural) == 0) ||
            (strcmp(rt.c_str(), Odr::Kind::TownArterial) == 0) ||
            (strcmp(rt.c_str(), Odr::Kind::TownCollector) == 0) ||
            (strcmp(rt.c_str(), Odr::Kind::TownExpressway) == 0) ||
            (strcmp(rt.c_str(), Odr::Kind::TownLocal) == 0) ||
            (strcmp(rt.c_str(), Odr::Kind::TownPlayStreet) == 0) ||
            (strcmp(rt.c_str(), Odr::Kind::TownPrivate) == 0) ||
            (strcmp(rt.c_str(), Odr::Kind::Town) == 0) )
        return true;

    return false;
}

std::string Odr::roadTypeString(Odr::Kind::RoadType rt)
{
    if (rt == Odr::Kind::RoadType::bicycle)
        return "bicycle";
    else if (rt == Odr::Kind::RoadType::lowSpeed)
        return "lowSpeed";
    else if (rt == Odr::Kind::RoadType::motorway)
        return "motorway";
    else if (rt == Odr::Kind::RoadType::pedestrian)
        return "pedestrian";
    else if (rt == Odr::Kind::RoadType::rural)
        return "rural";
    else if (rt == Odr::Kind::RoadType::townArterial)
        return "townArterial";
    else if (rt == Odr::Kind::RoadType::townCollector)
        return "townCollector";
    else if (rt == Odr::Kind::RoadType::townExpressway)
        return "townExpressway";
    else if (rt == Odr::Kind::RoadType::townLocal)
        return "townLocal";
    else if (rt == Odr::Kind::RoadType::townPlayStreet)
        return "townPlayStreet";
    else if (rt == Odr::Kind::RoadType::townPrivate)
        return "townPrivate";
    else if (rt == Odr::Kind::RoadType::town)
        return "town";

    return "unknown";
}

std::string Odr::laneTypeString(Odr::Kind::LaneType lt)
{
    if (lt == Odr::Kind::LaneType::driving)
        return "driving";
    else if (lt == Odr::Kind::LaneType::sidewalk)
        return "sidewalk";
    else if (lt == Odr::Kind::LaneType::walking)
        return "walking";
    else if (lt == Odr::Kind::LaneType::shoulder)
        return "shoulder";
    else if (lt == Odr::Kind::LaneType::border)
        return "border";
    else if (lt == Odr::Kind::LaneType::stop)
        return "stop";
    else if (lt == Odr::Kind::LaneType::restricted)
        return "restricted";
    else if (lt == Odr::Kind::LaneType::parking)
        return "parking";
    else if (lt == Odr::Kind::LaneType::median)
        return "median";
    else if (lt == Odr::Kind::LaneType::biking)
        return "biking";
    else if (lt == Odr::Kind::LaneType::curb)
        return "curb";
    else if (lt == Odr::Kind::LaneType::entry)
        return "entry";
    else if (lt == Odr::Kind::LaneType::exit)
        return "exit";
    else if (lt == Odr::Kind::LaneType::onRamp)
        return "onRamp";
    else if (lt == Odr::Kind::LaneType::offRamp)
        return "offRamp";
    else if (lt == Odr::Kind::LaneType::connectingRamp)
        return "connectingRamp";
    else if (lt == Odr::Kind::LaneType::slipLane)
        return "slipLane";

    else
        return "unknown";
}

Odr::Kind::LaneType Odr::laneTypeFromCString(const char* c)
{
    if (!strcmp(c, Odr::Kind::Driving))
        return Odr::Kind::LaneType::driving;

    if (!strcmp(c, Odr::Kind::Sidewalk))
        return Odr::Kind::LaneType::sidewalk;

    if (!strcmp(c, Odr::Kind::Walking))
        return Odr::Kind::LaneType::walking;

    if (!strcmp(c, Odr::Kind::Shoulder))
        return Odr::Kind::LaneType::shoulder;

    if (!strcmp(c, Odr::Kind::Border))
        return Odr::Kind::LaneType::border;

    if (!strcmp(c, Odr::Kind::Stop))
        return Odr::Kind::LaneType::stop;

    if (!strcmp(c, Odr::Kind::Restricted))
        return Odr::Kind::LaneType::restricted;

    if (!strcmp(c, Odr::Kind::Parking))
        return Odr::Kind::LaneType::parking;

    if (!strcmp(c, Odr::Kind::Median))
        return Odr::Kind::LaneType::median;

    if (!strcmp(c, Odr::Kind::Biking))
        return Odr::Kind::LaneType::biking;

    if (!strcmp(c, Odr::Kind::Curb))
        return Odr::Kind::LaneType::curb;

    if (!strcmp(c, Odr::Kind::Entry))
        return Odr::Kind::LaneType::entry;

    if (!strcmp(c, Odr::Kind::Exit))
        return Odr::Kind::LaneType::exit;

    if (!strcmp(c, Odr::Kind::OnRamp))
        return Odr::Kind::LaneType::onRamp;

    if (!strcmp(c, Odr::Kind::OffRamp))
        return Odr::Kind::LaneType::offRamp;

    if (!strcmp(c, Odr::Kind::ConnectingRamp))
        return Odr::Kind::LaneType::connectingRamp;

    if (!strcmp(c, Odr::Kind::SlipLane))
        return Odr::Kind::LaneType::slipLane;

    return Odr::Kind::LaneType::unknown;

}


Odr::Kind::RoadType Odr::roadTypeFromCString(const char* c)
{
    if (!strcmp(c, Odr::Kind::Bicycle))
        return Odr::Kind::RoadType::bicycle;

    if (!strcmp(c, Odr::Kind::LowSpeed))
        return Odr::Kind::RoadType::lowSpeed;

    if (!strcmp(c, Odr::Kind::Motorway))
        return Odr::Kind::RoadType::motorway;

    if (!strcmp(c, Odr::Kind::Pedestrian))
        return Odr::Kind::RoadType::pedestrian;

    if (!strcmp(c, Odr::Kind::Rural))
        return Odr::Kind::RoadType::rural;

    if (!strcmp(c, Odr::Kind::TownArterial))
        return Odr::Kind::RoadType::townArterial;

    if (!strcmp(c, Odr::Kind::TownCollector))
        return Odr::Kind::RoadType::townCollector;

    if (!strcmp(c, Odr::Kind::TownExpressway))
        return Odr::Kind::RoadType::townExpressway;

    if (!strcmp(c, Odr::Kind::TownLocal))
        return Odr::Kind::RoadType::townLocal;

    if (!strcmp(c, Odr::Kind::TownPlayStreet))
        return Odr::Kind::RoadType::townPlayStreet;

    if (!strcmp(c, Odr::Kind::TownPrivate))
        return Odr::Kind::RoadType::townPrivate;

    if (!strcmp(c, Odr::Kind::Town))
        return Odr::Kind::RoadType::town;


    return Odr::Kind::RoadType::unknown;
}


