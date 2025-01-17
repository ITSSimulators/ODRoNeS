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
using namespace odrones;


void ReadOdr::printRoads()
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


bool ReadOdr::isReady()
{
    return ready;
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


ReadOdr& ReadOdr::operator=(const ReadOdr &r)
{
    _k = r._k;
    _udConnections = r._udConnections;

    // Basics:
    for (uint i = 0; i < r.sections.size(); ++i)
    {
        _sections.push_back(Odr::smaS());
        _sections.back().id = r.sections[i].id;
        _sections.back().odrID = r.sections[i].odrID;
        _sections.back().lsSize = r.sections[i].lsSize;
        _sections.back().name = r.sections[i].name;
        _sections.back().type = r.sections[i].type;
        _sections.back().geom = r.sections[i].geom;
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
                Odr::smaL *l = getLaneWithODRIds(_sections[lk->sID].odrID, lk->odrID, lk->ndxLS);
                _sections[i].lanes[j].prevLane.push_back(l);
            }

            for (uint k = 0; k < r.sections[i].lanes[j].nextLane.size(); ++k)
            {
                Odr::smaL *lk = r.sections[i].lanes[j].nextLane[k];
                Odr::smaL *l = getLaneWithODRIds(_sections[lk->sID].odrID, lk->odrID, lk->ndxLS);
                _sections[i].lanes[j].nextLane.push_back(l);
            }
        }
    }

    return *this;
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

