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

#include "readOdr.h"
#include "xmlUtils.h"
#include "bezier3.h"
#include "arc.h"

using namespace odrones;


void ReadOdr::printRoads() const
{
    for (uint i = 0; i < _sections.size(); ++i)
    {
        for (uint j = 0; j < _sections[i].lanes.size(); ++j)
        {
            std::string plString = ", has " + std::to_string(_sections[i].lanes[j].prevLane.size()) + " previous lanes";
            for (size_t k=0; k < _sections[i].lanes[j].prevLane.size(); ++k)
            {
                if (k ==0) plString += ": ";
                else plString += ", ";
                plString += std::to_string( _sections[i].lanes[j].prevLane[k]->sID ) + ":"
                          + std::to_string( _sections[i].lanes[j].prevLane[k]->id );
                uint sID = _sections[i].lanes[j].prevLane[k]->sID;
                plString += "(" + std::to_string(_sections[sID].odrID) + ":"
                          + std::to_string(_sections[i].lanes[j].prevLane[k]->odrID) + ")";
            }

            std::string nlString = ", has " + std::to_string(_sections[i].lanes[j].nextLane.size()) + " next lanes";
            for (size_t k=0; k < _sections[i].lanes[j].nextLane.size(); ++k)
            {
                if (k ==0) nlString += ": ";
                else nlString += ", ";
                nlString += std::to_string( _sections[i].lanes[j].nextLane[k]->sID ) + ":"
                          + std::to_string( _sections[i].lanes[j].nextLane[k]->id );
                uint sID = _sections[i].lanes[j].nextLane[k]->sID;
                nlString += "(" + std::to_string(_sections[sID].odrID) + ":"
                          + std::to_string(_sections[i].lanes[j].nextLane[k]->odrID) + ")";
            }

            std::cout << "lane: " << _sections[i].id << ":" << _sections[i].lanes[j].id
                      << " (" << _sections[i].odrID << ":" << _sections[i].lanes[j].odrID
                      << ") has length: " << _sections[i].lanes[j].length
                      << ", is " << _sections[i].lanes[j].kind
                      << ", has sign:" << _sections[i].lanes[j].sign
                      << plString << nlString << std::endl;

        }
    }

}


bool ReadOdr::ready() const
{
    return _ready;
}

void ReadOdr::ready(bool r)
{
    _ready = r;
}

Odr::smaL* ReadOdr::getLaneWithODRIds(uint rOdrID, int lOdrID, int lsID)
{
    for (size_t i = 0; i < _sections.size(); ++i)
    {
        // Get the right road:
        if (_sections[i].odrID != rOdrID) continue;

        // Get the correct ndxLS: either lsID itself, or the max value.
        uint ndxLS;
        if (lsID < 0)
            ndxLS = _sections[i].lsSize - 1;
        else ndxLS = static_cast<uint>(lsID);

        // Find the right lane:
        for (size_t j = 0; j < _sections[i].lanes.size(); ++j)
        {
            if ((_sections[i].lanes[j].odrID == lOdrID) &&
                (_sections[i].lanes[j].ndxLS == ndxLS)) return &(_sections[i].lanes[j]);
        }
    }
    return nullptr;
}


ReadOdr& ReadOdr::operator+=(const ReadOdr &r)
{
    // Check that both instances are zero:
    if ( (!r.ready()) || (!ready()))
        throw std::invalid_argument("[ Error ] ReadOdr += doesn't work if one of the instances is not ready");


    // Check that we're adding the same stuff:
    if (r.k() != _k)
        throw std::invalid_argument("[ Error ] ReadOdr += tries to add an instance of a different kind k");

    // Check that there are no repeated Odr section IDs.
    for (uint i = 0; i < r.sections.size(); ++i)
    {
        for (uint j = 0; j < _sections.size(); ++j)
        {
            if (r.sections[i].odrID == _sections[j].odrID)
                throw std::invalid_argument("[ Error ] ReadOdr += tries to add in an instance with repeated Odr IDs. Renumber first.");
        }
    }

    // The udConnections become the new ones:
    _udConnections = r.udConnections;

    // Add the new roads:
    append(r);

    return *this;
}


ReadOdr& ReadOdr::operator=(const ReadOdr &r)
{
    _k = r._k;
    _udConnections = r._udConnections;
    _sections.clear();

    append(r);

    return *this;
}

void ReadOdr::renumber(uint shift)
{
    for (uint i = 0; i < _sections.size(); ++i)
    {
        _sections[i].odrID += shift;
    }
}

bool ReadOdr::uniqueSectionOdrIDs()
{
    std::vector<uint> ids;
    for (uint i = 0; i < _sections.size(); ++i)
    {
        if (std::find(ids.begin(), ids.end(), _sections[i].odrID) != ids.end())
        {
            std::cout << "[ ReadOdr ] has a repeated section.odrID: " << _sections[i].odrID << std::endl;
            return false;
        }
        ids.push_back(_sections[i].odrID);
    }
    return true;
}

void ReadOdr::transform(const arr2& t, scalar r)
{
    for (uint i = 0; i < _sections.size(); ++i)
    {
        for (uint j = 0; j < _sections[i].geom.size(); ++j)
        {
            arr2 p = {_sections[i].geom[j].x, _sections[i].geom[j].y};
            mvf::rotateVectorByAngle(p, r);
            p = {p[0] + t[0], p[1] + t[1]};
            _sections[i].geom[j].x = p[0];
            _sections[i].geom[j].y = p[1];
            _sections[i].geom[j].hdg += r;

            // To be removed with the end of the Beziers.
            arr2 bz0 = {_sections[i].geom[j].bz0x, _sections[i].geom[j].bz0y};
            mvf::rotateVectorByAngle(bz0, r);
            bz0 = {bz0[0] + t[0], bz0[1] + t[1]};
            _sections[i].geom[j].bz0x = bz0[0];
            _sections[i].geom[j].bz0y = bz0[1];

            arr2 bz1 = {_sections[i].geom[j].bz1x, _sections[i].geom[j].bz1y};
            mvf::rotateVectorByAngle(bz1, r);
            bz1 = {bz1[0] + t[0], bz1[1] + t[1]};
            _sections[i].geom[j].bz1x = bz1[0];
            _sections[i].geom[j].bz1y = bz1[1];

            arr2 bz2 = {_sections[i].geom[j].bz2x, _sections[i].geom[j].bz2y};
            mvf::rotateVectorByAngle(bz2, r);
            bz2 = {bz2[0] + t[0], bz2[1] + t[1]};
            _sections[i].geom[j].bz2x = bz2[0];
            _sections[i].geom[j].bz2y = bz2[1];

            arr2 bz3 = {_sections[i].geom[j].bz3x, _sections[i].geom[j].bz3y};
            mvf::rotateVectorByAngle(bz3, r);
            bz3 = {bz3[0] + t[0], bz3[1] + t[1]};
            _sections[i].geom[j].bz3x = bz3[0];
            _sections[i].geom[j].bz3y = bz3[1];

        }
    }
}

void ReadOdr::append(const ReadOdr &r)
{
    // Basics:
    for (uint i = 0; i < r.sections.size(); ++i)
    {
        _sections.push_back(Odr::smaS());
        _sections.back().id = _sections.size() -1; // r.sections[i].id;
        _sections.back().odrID = r.sections[i].odrID;
        _sections.back().lsSize = r.sections[i].lsSize;
        _sections.back().name = r.sections[i].name;
        _sections.back().type = r.sections[i].type;
        _sections.back().geom = r.sections[i].geom;
        _sections.back().rule = r.sections[i].rule;
        _sections.back().loffset = r.sections[i].loffset;
        _sections.back().tsigns = r.sections[i].tsigns;
        for (uint j = 0; j < r.sections[i].lanes.size(); ++j)
        {
            Odr::smaL smal;
            smal.id = r.sections[i].lanes[j].id;
            smal.sID = r.sections[i].lanes[j].sID;
            smal.ndxLS = r.sections[i].lanes[j].ndxLS;
            smal.odrID = r.sections[i].lanes[j].odrID;
            smal.sign = r.sections[i].lanes[j].sign;
            smal.length = r.sections[i].lanes[j].length;
            smal.speed = r.sections[i].lanes[j].speed;
            smal.startingS = r.sections[i].lanes[j].startingS;

            smal.w = r.sections[i].lanes[j].w;
            smal.border = r.sections[i].lanes[j].border;
            smal.kind = r.sections[i].lanes[j].kind;

            _sections.back().lanes.push_back(smal);
        }
    }

    // Links:
    for (uint i = 0; i < r.sections.size(); ++i)
    {
        for (uint j = 0; j < r.sections[i].lanes.size(); ++j)
        {
            for (uint k = 0; k < r.sections[i].lanes[j].prevLane.size(); ++k)
            {
                Odr::smaL *lk = r.sections[i].lanes[j].prevLane[k];
                if (lk != nullptr)
                {
                    Odr::smaL* l = getLaneWithODRIds(_sections[lk->sID].odrID, lk->odrID, lk->ndxLS);
                    _sections[i].lanes[j].prevLane.push_back(l);
                }
                else
                {
                    std::cerr << "Warning team:Null lk for prevLane\n";
                }
            }

            for (uint k = 0; k < r.sections[i].lanes[j].nextLane.size(); ++k)
            {
                Odr::smaL *lk = r.sections[i].lanes[j].nextLane[k];
                if (lk != nullptr)
                {
                    Odr::smaL* l = getLaneWithODRIds(_sections[lk->sID].odrID, lk->odrID, lk->ndxLS);
                    _sections[i].lanes[j].nextLane.push_back(l);
                }
                else
                {
                    std::cerr << "Warning team:Null lk for nextLane\n";
                }
            }
        }
    }
}


const Odr::smaS* ReadOdr::odrSection(uint odrID) const
{
    for (uint i = 0; i < _sections.size(); ++i)
    {
        if (odrID == _sections[i].odrID)
            return &(_sections[i]);
    }
    return nullptr;
}

void Odr::smaL::writeXMLWidth(tinyxml2::XMLElement *elem, tinyxml2::XMLDocument &doc) const
{
    for (uint k = 0; k < w.size(); ++k)
    {
        tinyxml2::XMLElement* width = doc.NewElement(Odr::Elem::Width);
        xmlUtils::setAttrOffsetSOffset(width, w[k]);
        elem->InsertEndChild(width);
    }
}

void Odr::smaL::writeXMLBorder(tinyxml2::XMLElement *elem, tinyxml2::XMLDocument &doc) const
{
    for (uint k = 0; k < border.size(); ++k)
    {
        tinyxml2::XMLElement* b = doc.NewElement(Odr::Elem::Border);
        xmlUtils::setAttrOffsetSOffset(b, border[k]);
        elem->InsertEndChild(b);
    }
}

void Odr::smaL::writeXML(tinyxml2::XMLElement *elem, tinyxml2::XMLDocument &doc) const
{
    elem->SetAttribute(Odr::Attr::Id, odrID);
    elem->SetAttribute(Odr::Attr::Type, kind.c_str());
    elem->SetAttribute(Odr::Attr::Level, Odr::Kind::False);

    tinyxml2::XMLElement* link = doc.NewElement(Odr::Elem::Link);
    if (nextLane.size())
    {
        tinyxml2::XMLElement* successor = doc.NewElement(Odr::Elem::Successor);
        successor->SetAttribute(Odr::Attr::Id, nextLane[0]->odrID);
        link->InsertEndChild(successor);
    }
    if (prevLane.size())
    {
        tinyxml2::XMLElement* predecessor = doc.NewElement(Odr::Elem::Predecessor);
        predecessor->SetAttribute(Odr::Attr::Id, prevLane[0]->odrID);
        link->InsertEndChild(predecessor);
    }
    elem->InsertEndChild(link);

    writeXMLWidth(elem, doc);
}

bool ReadOdr::simplifySingleArc(Odr::smaS &s)
{
    if (s.geom.size() == 1) return false;

    scalar tol = 2e-3;
    std::vector<arr2> points; // real points, without the control ones.
    for (uint i = 0; i < s.geom.size(); ++i)
    {
        points.push_back({s.geom[i].bz0x, s.geom[i].bz0y});
        points.push_back({s.geom[i].bz3x, s.geom[i].bz3y});
    }

    arc a_o;
    a_o.setWith3Points(points[0], points[1], points.back());
    vec2 to = a_o.to();
    uint cnt = 1;
    if (!a_o.ready()) return false; // in this case, the three points were aligned, and creating the arc was unsuccessful.
    // see if they're a single arc, that is they all have the same radius of curvature:
    for (uint i = 2; i < points.size() -1; ++i)
    {
        arc a_i;
        a_i.setWith3Points(points[0], points[i], points.back());
        if (!mvf::areCloseEnough(a_o.radiusOfCurvature(), a_i.radiusOfCurvature(), tol))
            return false;
        to = to + a_i.to();
    }

    // Create a new arc with an averaged to
    to.normalise();
    arc a_f(points[0], points.back(), to.a2());
    // a_f.printOut();

    s.geom.clear();
    s.geom.push_back( Odr::geometry() );
    s.geom.back().g = Odr::Attr::Geometry::arc;
    s.geom.back().s = 0;
    s.geom.back().x = a_f.origin()[0];
    s.geom.back().y = a_f.origin()[1];
    s.geom.back().hdg = std::atan2(a_f.to()[1], a_f.to()[0]);
    s.geom.back().length = a_f.length();
    s.geom.back().curvature = 1.0 / a_o.radiusOfCurvature();

    return true;
}

bool ReadOdr::simplifyStraights(Odr::smaS &s)
{
    // 1 - swap Beziers for straights
    bool swaps = false;
    for (uint i = 0; i < s.geom.size(); ++i)
    {
        if (s.geom[i].g != Odr::Attr::Geometry::bezier3)
            continue;

        std::vector<arr2> cp;
        cp.push_back({s.geom[i].bz0x, s.geom[i].bz0y});
        cp.push_back({s.geom[i].bz1x, s.geom[i].bz1y});
        cp.push_back({s.geom[i].bz2x, s.geom[i].bz2y});
        cp.push_back({s.geom[i].bz3x, s.geom[i].bz3y});


        if (mvf::areAligned(cp))
        {
            s.geom[i].zeroBezier();
            s.geom[i].g = Odr::Attr::Geometry::line;
            arr2 tg = mvf::tangent(cp[0], cp[3]);
            s.geom[i].hdg = std::atan2(tg[1], tg[0]);
            if (!mvf::areCloseEnough(s.geom[i].length, mvf::distance(cp[0], cp[3]), 1e-4))
                std::cout << "[ ReadBOdr ] simplified straight has different length: "
                          << s.geom[i].length << " vs " << mvf::distance(cp[0], cp[3]) << std::endl;
            swaps = true;
            continue;
        }
    }

    // 2 - Join straights:
    std::vector<int> nuke;
    for (uint i = 0; i < s.geom.size() -1; ++i)
    {
        if (s.geom[i].g != Odr::Attr::Geometry::line) continue;

        uint io = i;
        while ((i < s.geom.size() -1) &&
               (s.geom[i+1].g == Odr::Attr::Geometry::line) &&
               (mvf::areCloseEnough(s.geom[io].hdg, s.geom[i+1].hdg, 1e-6)))
        {
            s.geom[io].length += s.geom[i+1].length;
            nuke.push_back(i+1);

            ++i;
        }
    }

    for (uint i = nuke.size() -1; i <= 0; --i)
        s.geom.erase(s.geom.begin()+nuke[i]);


    return swaps;
}

void ReadOdr::simplifyGeometries(Odr::smaS &s)
{

    if (simplifySingleArc(s))
        return;

    simplifyStraights(s);



}
