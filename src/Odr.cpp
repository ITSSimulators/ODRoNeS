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
using namespace odrones;

const char* Odr::Elem::OpenDrive = "OpenDRIVE";
const char* Odr::Elem::Header = "header";

const char* Odr::Elem::Road = "road";
const char* Odr::Elem::Type = "type";
const char* Odr::Elem::PlanView = "planView";
const char* Odr::Elem::Lane = "lane";
const char* Odr::Elem::Lanes = "lanes";
const char* Odr::Elem::LaneOffset = "laneOffset";
const char* Odr::Elem::LaneSection = "laneSection";

const char* Odr::Elem::Junction = "junction";
const char* Odr::Elem::Connection = "connection";
const char* Odr::Elem::LaneLink = "laneLink";

const char* Odr::Elem::Link = "link";
const char* Odr::Elem::Predecessor = "predecessor";
const char* Odr::Elem::Successor = "successor";

const char* Odr::Elem::Left = "left";
const char* Odr::Elem::Center = "center";
const char* Odr::Elem::Right = "right";

const char* Odr::Elem::Geometry = "geometry";
const char* Odr::Elem::Line = "line";
const char* Odr::Elem::Arc = "arc";
const char* Odr::Elem::Spiral = "spiral";
const char* Odr::Elem::ParamPoly3 = "paramPoly3";
const char* Odr::Elem::Bezier3 = "bezier3";

const char* Odr::Elem::Width = "width";
const char* Odr::Elem::Speed = "speed";
const char* Odr::Elem::Border = "border";

const char* Odr::Elem::Signals = "signals";
const char* Odr::Elem::Signal = "signal";
const char* Odr::Elem::Validity = "validity";
const char* Odr::Elem::UserData = "userData";
const char* Odr::Elem::Dependency = "dependency";

const char* Odr::Attr::Type     = "type";
const char* Odr::Attr::Name     = "name";
const char* Odr::Attr::Id       = "id";
const char* Odr::Attr::Junction = "junction";
const char* Odr::Attr::Length   = "length";
const char* Odr::Attr::Rule     = "rule";

const char* Odr::Attr::IncomingRoad = "incomingRoad";
const char* Odr::Attr::ConnectingRoad = "connectingRoad";
const char* Odr::Attr::ContactPoint = "contactPoint";
const char* Odr::Attr::From = "from";
const char* Odr::Attr::To = "to";

const char* Odr::Attr::SingleSide = "singleSide";

const char* Odr::Attr::ElementType = "elementType";
const char* Odr::Attr::ElementId = "elementId";

const char* Odr::Attr::Level = "level";

const char* Odr::Attr::Curvature = "curvature";
const char* Odr::Attr::CurvStart = "curvStart";
const char* Odr::Attr::CurvEnd = "curvEnd";
const char* Odr::Attr::S = "s";
const char* Odr::Attr::T = "t";
const char* Odr::Attr::X = "x";
const char* Odr::Attr::Y = "y";
const char* Odr::Attr::Hdg = "hdg";

const char* Odr::Attr::sOffset = "sOffset";
const char* Odr::Attr::A = "a";
const char* Odr::Attr::B = "b";
const char* Odr::Attr::C = "c";
const char* Odr::Attr::D = "d";

const char* Odr::Attr::Max = "max";

const char* Odr::Attr::aU = "aU";
const char* Odr::Attr::bU = "bU";
const char* Odr::Attr::cU = "cU";
const char* Odr::Attr::dU = "dU";
const char* Odr::Attr::aV = "aV";
const char* Odr::Attr::bV = "bV";
const char* Odr::Attr::cV = "cV";
const char* Odr::Attr::dV = "dV";
const char* Odr::Attr::pRange = "pRange";

const char* Odr::Attr::bz0x = "bz0x";
const char* Odr::Attr::bz0y = "bz0y";
const char* Odr::Attr::bz1x = "bz1x";
const char* Odr::Attr::bz1y = "bz1y";
const char* Odr::Attr::bz2x = "bz2x";
const char* Odr::Attr::bz2y = "bz2y";
const char* Odr::Attr::bz3x = "bz3x";
const char* Odr::Attr::bz3y = "bz3y";

const char* Odr::Attr::Unit = "unit";

const char* Odr::Attr::Dynamic = "dynamic";
const char* Odr::Attr::Orientation = "orientation";
const char* Odr::Attr::ZOffset = "zOffset";
const char* Odr::Attr::Value = "value";
const char* Odr::Attr::Height = "height";
const char* Odr::Attr::Width = "width";
const char* Odr::Attr::Text = "text";
const char* Odr::Attr::HOffset = "hOffset";
const char* Odr::Attr::Pitch = "pitch";
const char* Odr::Attr::Roll = "roll";

const char* Odr::Attr::FromLane = "fromLane";
const char* Odr::Attr::ToLane = "toLane";

const char* Odr::Kind::arcLength = "arcLength";
const char* Odr::Kind::normalized = "normalized";


const char* Odr::Kind::Driving = "driving";
const char* Odr::Kind::Sidewalk = "sidewalk";
const char* Odr::Kind::Walking = "walking";
const char* Odr::Kind::Unknown = "unknown";

const char* Odr::Kind::Start = "start";
const char* Odr::Kind::End = "end";

const char* Odr::Kind::Road = "road";
const char* Odr::Kind::Junction = "junction";

const char* Odr::Kind::mph = "mph";
const char* Odr::Kind::ms = "m/s";
const char* Odr::Kind::kmh = "km/h";

const char* Odr::Kind::Plus = "+";
const char* Odr::Kind::Minus = "-";
const char* Odr::Kind::None = "none";

const char* Odr::Kind::False = "false";
const char* Odr::Kind::True = "true";

const char* Odr::Kind::RHT = "RHT";
const char* Odr::Kind::LHT = "LHT";

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



