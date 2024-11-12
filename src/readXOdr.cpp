// 
//  This file is part of the ODRoNeS (OpenDRIVE Road Network System) package.
//  
//  Copyright (c) 2024 Albert Solernou, University of Leeds.
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

#include "readXOdr.h"
using namespace odrones;


void ReadXOdr::XMLCheckResult(int a_eResult)
{
    if (a_eResult != tinyxml2::XML_SUCCESS)
    {
        printf("tinyXML Error: %i\n", a_eResult);
    }
}


Odr::smaL* ReadXOdr::getLaneWithODRIds(uint rOdrID, int lOdrID, int lsID)
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

// though we don't use it.
std::vector<Odr::connection> ReadXOdr::readJunction(tinyxml2::XMLElement *c)
{
    std::vector<Odr::connection> j;

    if (c) c = c->FirstChildElement(Odr::Elem::Connection);
    while (c)
    {
        // Read the connection id, incoming road and connecting road;
        Odr::connection smaC;
        j.push_back(smaC);
        XMLCheckResult(c->QueryIntAttribute(Odr::Attr::Id, &j.back().id));
        XMLCheckResult(c->QueryIntAttribute(Odr::Attr::IncomingRoad, &j.back().incomingRoad));
        XMLCheckResult(c->QueryIntAttribute(Odr::Attr::ConnectingRoad, &j.back().connectingRoad));

        // Read the contact point and assign 0 or one depending on whether it is start or end:
        const char *txt = nullptr;
        txt = c->Attribute(Odr::Attr::ContactPoint);
        std::string cp = txt;
        if (cp.compare(Odr::Kind::Start) == 0) j.back().contactPoint = 0;
        else if (cp.compare(Odr::Kind::End) == 0) j.back().contactPoint = 1;
        else
        {
            std::cerr << "[ ERROR ] unable to assign contact point for connection id: "
                      << j.back().id << std::endl;
            break;
        }

        // Read all the laneLinks:
        tinyxml2::XMLElement *llXML = c->FirstChildElement(Odr::Elem::LaneLink);
        while (llXML)
        {
            Odr::laneLink ll;
            j.back().llink.push_back(ll);
            XMLCheckResult(llXML->QueryIntAttribute(Odr::Attr::From, &j.back().llink.back().from));
            XMLCheckResult(llXML->QueryIntAttribute(Odr::Attr::To, &j.back().llink.back().to));

            // loop over:
            llXML = llXML->NextSiblingElement(Odr::Elem::LaneLink);
        }

        // loop over
        c = c->NextSiblingElement(Odr::Elem::Connection);
    }

    return j;

}

std::vector<Odr::tsign> ReadXOdr::readTrafficSigns(tinyxml2::XMLElement *xmlsgns)
{
    std::vector<Odr::tsign> tsigns;
    if (!xmlsgns)
    {
        std::cerr << "Unable to read traffic signs from a nullptr!" << std::endl;
        return tsigns;
    }

    tinyxml2::XMLElement *s = xmlsgns->FirstChildElement(Odr::Elem::Signal);
    if (!s)
    {
        std::cerr << "no sign to read, no sign found in the signs section!" << std::endl;
        return tsigns;
    }

    while (s)
    {
        const char* txt = nullptr;
        tsigns.push_back(Odr::tsign());

        s->QueryDoubleAttribute(Odr::Attr::S, &tsigns.back().s);
        s->QueryDoubleAttribute(Odr::Attr::T, &tsigns.back().t);
        txt = s->Attribute(Odr::Attr::Id);
        if (txt) tsigns.back().id = txt;
        txt = s->Attribute(Odr::Attr::Name);
        if (txt)
        tsigns.back().name = txt;
        txt = s->Attribute(Odr::Attr::Dynamic);
        if (txt)
        {
            std::string dynamic = txt;
            if (!dynamic.compare("no")) tsigns.back().dynamic = false;
            else if (!dynamic.compare("yes")) tsigns.back().dynamic = true;
            else
                std::cerr << "[ Error ] Unable to tell whether this sign is dynamic or not" << std::endl;
        }
        txt = s->Attribute(Odr::Attr::Orientation);
        if (txt)
        {
            std::string orientation = txt;
            if (!orientation.compare(Odr::Kind::Plus))
                tsigns.back().orientation = 1;
            else if (!orientation.compare(Odr::Kind::Minus))
                tsigns.back().orientation = -1;
            else if (!orientation.compare(Odr::Kind::None))
                tsigns.back().orientation = 0;
            else
            {
                std::cerr << "[ Error ] Unable to tell the orientation for this sign" << std::endl;
            }
        }
        s->QueryDoubleAttribute(Odr::Attr::ZOffset, &tsigns.back().zOffset);
        s->QueryDoubleAttribute(Odr::Attr::Value, &tsigns.back().value);
        txt =  s->Attribute(Odr::Attr::Unit);
        if (txt) tsigns.back().unit = txt;
        s->QueryDoubleAttribute(Odr::Attr::Height, &tsigns.back().height);
        s->QueryDoubleAttribute(Odr::Attr::Width, &tsigns.back().width);
        txt = s->Attribute(Odr::Attr::Text);
        if (txt) tsigns.back().text = txt;
        s->QueryDoubleAttribute(Odr::Attr::HOffset, &tsigns.back().zOffset);
        s->QueryDoubleAttribute(Odr::Attr::Pitch, &tsigns.back().pitch);
        s->QueryDoubleAttribute(Odr::Attr::Roll, &tsigns.back().roll);

        tinyxml2::XMLElement *dpn = s->FirstChildElement(Odr::Elem::Dependency);
        if (dpn)
        {
            std::cerr << "[ Warning ] this signal includes a dependency, but it will be ignored!" << std::endl;
        }

        tinyxml2::XMLElement *vld = s->FirstChildElement(Odr::Elem::Validity);
        if (vld)
        {
            s->QueryIntAttribute(Odr::Attr::FromLane, &tsigns.back().fromLane);
            s->QueryIntAttribute(Odr::Attr::ToLane, &tsigns.back().toLane);
        }

        tinyxml2::XMLElement *ud = s->FirstChildElement(Odr::Elem::UserData);
        if (ud)
        {
            std::cerr << "[ Warning ] this signal includes user data, but it will be ignored!" << std::endl;
        }

        s = s->NextSiblingElement(Odr::Elem::Signal);
    }
    return tsigns;

}

std::vector<Odr::geometry> ReadXOdr::readGeometry(tinyxml2::XMLElement *pv)
{

    std::vector<Odr::geometry> v;

    if (!pv)
    {
        std::cerr << "unable to read geometry from a nullptr!" << std::endl;
        return v;
    }

    tinyxml2::XMLElement *g = pv->FirstChildElement(Odr::Elem::Geometry);
    if (!g)
    {
        std::cerr << "no geometry to be read!" << std::endl;
        return v;
    }

    while (g)
    {
        v.push_back(Odr::geometry());

        g->QueryDoubleAttribute(Odr::Attr::S, &v.back().s);
        g->QueryDoubleAttribute(Odr::Attr::X, &v.back().x);
        g->QueryDoubleAttribute(Odr::Attr::Y, &v.back().y);
        g->QueryDoubleAttribute(Odr::Attr::Hdg, &v.back().hdg);
        g->QueryDoubleAttribute(Odr::Attr::Length, &v.back().length);

        tinyxml2::XMLElement *shp = g->FirstChildElement(Odr::Elem::Line);
        if (shp) v.back().g = Odr::Attr::Geometry::line;
        else
        {
            shp = g->FirstChildElement(Odr::Elem::Arc);
            if (shp) v.back().g = Odr::Attr::Geometry::arc;
            else
            {
                shp = g->FirstChildElement(Odr::Elem::Spiral);
                if (shp) v.back().g = Odr::Attr::Geometry::spiral;
                else
                {
                    shp = g->FirstChildElement(Odr::Elem::ParamPoly3);
                    if (shp) v.back().g = Odr::Attr::Geometry::paramPoly3;
                    else
                    {
                        shp = g->FirstChildElement(Odr::Elem::Bezier3);
                        if (shp) v.back().g = Odr::Attr::Geometry::bezier3;
                    }
                }
            }
        }

        bool read = false;
        if (v.back().g == Odr::Attr::Geometry::line)
            read = true;
        else if (v.back().g == Odr::Attr::Geometry::arc)
        {
            shp->QueryDoubleAttribute(Odr::Attr::Curvature, &v.back().curvature);
            read = true;
        }
        else if (v.back().g == Odr::Attr::Geometry::spiral)
        {
            shp->QueryDoubleAttribute(Odr::Attr::CurvStart, &v.back().curvStart );
            shp->QueryDoubleAttribute(Odr::Attr::CurvEnd, &v.back().curvEnd );
            read = true;
        }
        else if (v.back().g == Odr::Attr::Geometry::paramPoly3)
        {
            read = true;
            shp->QueryDoubleAttribute(Odr::Attr::aU, &v.back().aU);
            shp->QueryDoubleAttribute(Odr::Attr::bU, &v.back().bU);
            shp->QueryDoubleAttribute(Odr::Attr::cU, &v.back().cU);
            shp->QueryDoubleAttribute(Odr::Attr::dU, &v.back().dU);
            shp->QueryDoubleAttribute(Odr::Attr::aV, &v.back().aV);
            shp->QueryDoubleAttribute(Odr::Attr::bV, &v.back().bV);
            shp->QueryDoubleAttribute(Odr::Attr::cV, &v.back().cV);
            shp->QueryDoubleAttribute(Odr::Attr::dV, &v.back().dV);
            std::string prange = shp->Attribute(Odr::Attr::pRange);
            if (prange.compare(Odr::Kind::arcLength) == 0)
                v.back().pRange = Odr::Attr::ParamPoly3Range::arcLength;
            else if (prange.compare(Odr::Kind::normalized) == 0)
                v.back().pRange = Odr::Attr::ParamPoly3Range::normalized;
            else read = false;
        }
        else if (v.back().g == Odr::Attr::Geometry::bezier3)
        {
            shp->QueryDoubleAttribute(Odr::Attr::bz0x, &v.back().bz0x);
            shp->QueryDoubleAttribute(Odr::Attr::bz0y, &v.back().bz0y);
            shp->QueryDoubleAttribute(Odr::Attr::bz1x, &v.back().bz1x);
            shp->QueryDoubleAttribute(Odr::Attr::bz1y, &v.back().bz1y);
            shp->QueryDoubleAttribute(Odr::Attr::bz2x, &v.back().bz2x);
            shp->QueryDoubleAttribute(Odr::Attr::bz2y, &v.back().bz2y);
            shp->QueryDoubleAttribute(Odr::Attr::bz3x, &v.back().bz3x);
            shp->QueryDoubleAttribute(Odr::Attr::bz3y, &v.back().bz3y);
            read = true;
        }

        g = g->NextSiblingElement(Odr::Elem::Geometry);
        if (!read)
        {
            std::cerr << "[ Error ] Geometry could not be identified!" << std::endl;
        }
    }

    return v;
}

std::vector<Odr::offset> ReadXOdr::readLaneOffset(tinyxml2::XMLElement *lanes)
{
    std::vector<Odr::offset> loff;
    tinyxml2::XMLElement *loffXML = lanes->FirstChildElement(Odr::Elem::LaneOffset);
    while (loffXML)
    {
        Odr::offset loff_i;

        loff_i.lr = Odr::offset::LR::L;
        loffXML->QueryDoubleAttribute(Odr::Attr::A, &(loff_i.a));
        loffXML->QueryDoubleAttribute(Odr::Attr::B, &(loff_i.b));
        loffXML->QueryDoubleAttribute(Odr::Attr::C, &(loff_i.c));
        loffXML->QueryDoubleAttribute(Odr::Attr::D, &(loff_i.d));
        loffXML->QueryDoubleAttribute(Odr::Attr::S, &(loff_i.s));

        loff.push_back(loff_i);
        loffXML = loffXML->NextSiblingElement(Odr::Elem::LaneOffset);
    }
    if (loff.size())
        loff.back().lr = Odr::offset::LR::RL;
    return loff;
}

bool ReadXOdr::hasLaneOffset(tinyxml2::XMLElement *lanes)
{
    tinyxml2::XMLElement *laneOffset = lanes->FirstChildElement(Odr::Elem::LaneOffset);
    while (laneOffset)
    {
        double a = 0;
        laneOffset->QueryDoubleAttribute(Odr::Attr::A, &a);
        if (!mvf::areSameValues(a, 0)) return true;
        laneOffset->QueryDoubleAttribute(Odr::Attr::B, &a);
        if (!mvf::areSameValues(a, 0)) return true;
        laneOffset->QueryDoubleAttribute(Odr::Attr::C, &a);
        if (!mvf::areSameValues(a, 0)) return true;
        laneOffset->QueryDoubleAttribute(Odr::Attr::D, &a);
        if (!mvf::areSameValues(a, 0)) return true;
        laneOffset = lanes->NextSiblingElement(Odr::Elem::LaneOffset);
    }
    return false;
}

bool ReadXOdr::hasComplicatedOffset(Odr::offset &o)
{

    if (!mvf::areSameValues(o.s, 0)) return true;
    else if (!mvf::areSameValues(o.b, 0)) return true;
    else if (!mvf::areSameValues(o.c, 0)) return true;
    else if (!mvf::areSameValues(o.d, 0)) return true;
    return false;
}




int ReadXOdr::getRoadLinkData(uint &rLinkID, uint &rLinkCP, tinyxml2::XMLElement *fbLinkXML)
{
    if (!fbLinkXML) return 0;

    const char* txt = fbLinkXML->Attribute(Odr::Attr::ElementType);
    if (!txt)
    {
        std::cerr << "[ Error ] no element type in Link" << std::endl;
        return 1;
    }
    std::string elementType = txt;
    if (elementType.compare(Odr::Kind::Road) == 0)
    {
        int linkID;
        XMLCheckResult(fbLinkXML->QueryIntAttribute(Odr::Attr::ElementId, &linkID));
        // XMLCheckResult(fbLinkXML->QueryIntAttribute(Odr::Attr::Id, &linkID));
        rLinkID = static_cast<uint>(linkID);

        const char* txt = fbLinkXML->Attribute(Odr::Attr::ContactPoint);
        if (!txt) rLinkCP = 2;
        else
        {
            std::string contactPoint = txt;
            if (contactPoint.compare(Odr::Kind::Start) == 0) rLinkCP = 0;
            else if (contactPoint.compare(Odr::Kind::End) == 0) rLinkCP = 1;
            else
            {
                std::cerr << "[ Error ] Unrecognised contact point in link" << std::endl;
                return 1;
            }
        }
    }

    return 0;
}


int ReadXOdr::addLane(tinyxml2::XMLElement *road, tinyxml2::XMLElement *lane, uint ndxS, uint ndxL, uint ndxLS, double startingS)
{
    const char *txt = lane->Attribute(Odr::Attr::Type);
    if (!txt) return -1;
    std::string kind = txt;

    _sections[ndxS].lanes.push_back(Odr::smaL());

    _sections[ndxS].lanes.back().id = ndxL;
    _sections[ndxS].lanes.back().sID = ndxS;
    _sections[ndxS].lanes.back().ndxLS = ndxLS;
    _sections[ndxS].lanes.back().kind = kind;
    _sections[ndxS].lanes.back().startingS = startingS;

    double length = 0;
    XMLCheckResult(road->QueryDoubleAttribute(Odr::Attr::Length, &length));
    _sections[ndxS].lanes.back().length = length;

    int odrID = 0;
    XMLCheckResult(lane->QueryIntAttribute(Odr::Attr::Id, &odrID));
    _sections[ndxS].lanes.back().odrID = odrID;

    if (odrID > 0) _sections[ndxS].lanes.back().sign = 1;
    else if (odrID < 0) _sections[ndxS].lanes.back().sign = -1;
    else
    {
        std::cerr << "[ Error ] trying to create a lane with Odr ID = 0" << std::endl;
        return -1;
    }

    tinyxml2::XMLElement *width = lane->FirstChildElement(Odr::Elem::Width);
    while (width)
    {
        _sections[ndxS].lanes.back().w.push_back(Odr::offset());
        _sections[ndxS].lanes.back().w.back().lr = Odr::offset::LR::L;
        XMLCheckResult(width->QueryDoubleAttribute(Odr::Attr::sOffset, &(_sections[ndxS].lanes.back().w.back().s) ));
        XMLCheckResult(width->QueryDoubleAttribute(Odr::Attr::A, &(_sections[ndxS].lanes.back().w.back().a) ));
        XMLCheckResult(width->QueryDoubleAttribute(Odr::Attr::B, &(_sections[ndxS].lanes.back().w.back().b) ));
        XMLCheckResult(width->QueryDoubleAttribute(Odr::Attr::C, &(_sections[ndxS].lanes.back().w.back().c) ));
        XMLCheckResult(width->QueryDoubleAttribute(Odr::Attr::D, &(_sections[ndxS].lanes.back().w.back().d) ));
        width = width->NextSiblingElement(Odr::Elem::Width);
    }
    if (!_sections[ndxS].lanes.back().w.size())
        _sections[ndxS].lanes.back().w.push_back(Odr::offset()); // {0.,0.,0.,0.,0.});
    _sections[ndxS].lanes.back().w.back().lr = Odr::offset::LR::RL;


    tinyxml2::XMLElement *border = lane->FirstChildElement(Odr::Elem::Border);
    while (border)
    {
        _sections[ndxS].lanes.back().border.push_back(Odr::offset());
        _sections[ndxS].lanes.back().border.back().lr = Odr::offset::LR::L;
        XMLCheckResult(border->QueryDoubleAttribute(Odr::Attr::sOffset, &(_sections[ndxS].lanes.back().border.back().s) ));
        XMLCheckResult(border->QueryDoubleAttribute(Odr::Attr::A, &(_sections[ndxS].lanes.back().border.back().a) ));
        XMLCheckResult(border->QueryDoubleAttribute(Odr::Attr::B, &(_sections[ndxS].lanes.back().border.back().b) ));
        XMLCheckResult(border->QueryDoubleAttribute(Odr::Attr::C, &(_sections[ndxS].lanes.back().border.back().c) ));
        XMLCheckResult(border->QueryDoubleAttribute(Odr::Attr::D, &(_sections[ndxS].lanes.back().border.back().d) ));
        border = border->NextSiblingElement(Odr::Elem::Border);
    }
    if (!_sections[ndxS].lanes.back().border.size())
        _sections[ndxS].lanes.back().border.push_back(Odr::offset()); // {0.,0.,0.,0.,0.} );
    _sections[ndxS].lanes.back().border.back().lr = Odr::offset::LR::RL;


    tinyxml2::XMLElement *speed = lane->FirstChildElement(Odr::Elem::Speed);
    if (speed)
    {
        XMLCheckResult(speed->QueryDoubleAttribute(Odr::Attr::Max, &(_sections[ndxS].lanes.back().speed)));

        std::string units = speed->Attribute(Odr::Attr::Unit);
        if (units.compare(Odr::Kind::mph) == 0)
            _sections[ndxS].lanes.back().speed *= ct::mphToMs;
        else if (units.compare(Odr::Kind::kmh) == 0)
            _sections[ndxS].lanes.back().speed *= ct::kmhToMs;
        else if (units.compare(Odr::Kind::ms) != 0)
            std::cerr << "[ Error ] unknown speed units: " << units << std::endl;

        if (speed->NextSiblingElement(Odr::Elem::Speed))
            std::cout << "[ Warning ] there's more than one speed limit to be parsed, but we only do one." << std::endl;
    }
    else _sections[ndxS].lanes.back().speed = defaultSpeed;


    ndxL += 1;
    return static_cast<int>(ndxL);
}

uint ReadXOdr::linkLanes(tinyxml2::XMLElement *lXML, uint ndxS, uint ndxL, uint rPrevID, uint rNextID) //, int rPrevCP, int rNextCP)
{

    while (lXML)
    {
        int ndxLS = static_cast<int>(_sections[ndxS].lanes[ndxL].ndxLS);

        std::string type = lXML->Attribute(Odr::Attr::Type); // presence of Type has been checked in addLane
        tinyxml2::XMLElement *lLink = lXML->FirstChildElement(Odr::Elem::Link);
        if (lLink)
        {
            tinyxml2::XMLElement *lPrev = lLink->FirstChildElement(Odr::Elem::Predecessor);
            if (lPrev)
            {
                int lPrevID = 0;
                XMLCheckResult(lPrev->QueryIntAttribute(Odr::Attr::Id, &lPrevID));
                // Now the lane may be linked
                // to a lane in the previous road (last laneSection):
                uint rLink = rPrevID;
                int lsLink = -1;
                // or to a lane in the previous lane section (same road), if ndxLS > 0:
                if (ndxLS > 0)
                {
                    rLink = ndxS;
                    lsLink = ndxLS - 1;
                }
                Odr::smaL *l = getLaneWithODRIds(rLink, lPrevID, lsLink);
                _sections[ndxS].lanes[ndxL].prevLane.push_back(l);
            }


            tinyxml2::XMLElement *lNext = lLink->FirstChildElement(Odr::Elem::Successor);
            if (lNext)
            {
                int lNextID = 0;
                XMLCheckResult(lNext->QueryIntAttribute(Odr::Attr::Id, &lNextID));
                // Now the lane may be linked to the next road (first laneSection)
                uint rLink = rNextID;
                int lsLink = 0;
                // or to the next laneSection (same road), if it exists:
                for (uint i = 0; i < _sections[ndxS].lanes.size(); ++i)
                {
                    if ( (_sections[ndxS].lanes[i].ndxLS == (ndxLS + 1)) &&
                         (_sections[ndxS].lanes[i].odrID == lNextID) )
                    {
                        rLink = ndxS;
                        lsLink = _sections[ndxS].lanes[ndxL].ndxLS;
                        break;
                    }
                }
                Odr::smaL *l = getLaneWithODRIds(rLink, lNextID, lsLink);
                _sections[ndxS].lanes[ndxL].nextLane.push_back(l);
            }
        }

        ndxL += 1;

        lXML = lXML->NextSiblingElement(Odr::Elem::Lane);
    }

    return ndxL;
}



ReadXOdr::ReadXOdr(std::string iFile, bool isOdrFile)
{
    ready = false;
    if (!loadXodr(iFile, isOdrFile)) ready = true;
}

int ReadXOdr::loadXodr(std::string iFile, bool isOdrFile)
{

    tinyxml2::XMLDocument doc;
    if (isOdrFile)
        XMLCheckResult(doc.LoadFile(iFile.c_str()));
    else
        XMLCheckResult(doc.Parse(iFile.c_str()));

    tinyxml2::XMLElement *root = doc.FirstChildElement(Odr::Elem::OpenDrive);
    if (root == nullptr) return tinyxml2::XML_ERROR_FILE_READ_ERROR;

    // Parse the roads:
    // 1 - Get the thick of it:
    std::cout << "reading the roads!" << std::endl;

    // Step 1: load all the data but the linkage:
    tinyxml2::XMLElement *r = root->FirstChildElement(Odr::Elem::Road);
                          // r = root->FirstChildElement(Odr::Elem::Road);
    uint ndxS = 0;
    while (r)
    {
        // MISSING - Look for Objects such as zebra crossing.

        _sections.push_back(Odr::smaS());

        int roadID = 0;
        double length;
        XMLCheckResult(r->QueryIntAttribute(Odr::Attr::Id, &roadID));
        XMLCheckResult(r->QueryDoubleAttribute(Odr::Attr::Length, &length));

        _sections[ndxS].id = ndxS;
        _sections[ndxS].odrID = static_cast<uint>(roadID);

        _sections[ndxS].geom = readGeometry(r->FirstChildElement(Odr::Elem::PlanView));

        tinyxml2::XMLElement *tSigns = r->FirstChildElement(Odr::Elem::Signals);
        if (tSigns) _sections[ndxS].tsigns = readTrafficSigns(tSigns);

        tinyxml2::XMLElement *lanes = r->FirstChildElement(Odr::Elem::Lanes);
        _sections[ndxS].loffset = readLaneOffset(lanes);

        uint ndxL = 0;
        uint ndxLS = 0;
        tinyxml2::XMLElement *laneSection = lanes->FirstChildElement(Odr::Elem::LaneSection);
        while (laneSection)
        {
            double startingS = 0;
            XMLCheckResult(laneSection->QueryDoubleAttribute(Odr::Attr::S, &startingS));

            bool oneSided = false;
            if (laneSection->QueryBoolAttribute(Odr::Attr::SingleSide, &oneSided) == tinyxml2::XML_NO_ATTRIBUTE)
                oneSided = false;
            if (oneSided)
                std::cout << "[ Warning ] SingleSide laneSection here!" << std::endl;

            tinyxml2::XMLElement *left = laneSection->FirstChildElement(Odr::Elem::Left);
            tinyxml2::XMLElement *l = nullptr;
            if (left) l = left->FirstChildElement(Odr::Elem::Lane);
            while (l)
            {
                int storage = addLane(r, l, ndxS, ndxL, ndxLS, startingS);
                if (storage < 0) return 1;
                else ndxL = static_cast<uint>(storage);
                l = l->NextSiblingElement(Odr::Elem::Lane);
            }

            tinyxml2::XMLElement *right = laneSection->FirstChildElement(Odr::Elem::Right);
            if (right) l = right->FirstChildElement(Odr::Elem::Lane);
            while (l)
            {
                int storage = addLane(r, l, ndxS, ndxL, ndxLS, startingS);
                if (storage < 0) return 1;
                else ndxL = static_cast<uint>(storage);
                l = l->NextSiblingElement(Odr::Elem::Lane);
            }

            laneSection = laneSection->NextSiblingElement(Odr::Elem::LaneSection);
            ndxLS += 1;
        }

        _sections[ndxS].lsSize = ndxLS; // remember the amount of lane sections.

        ndxS += 1;
        r = r->NextSiblingElement(Odr::Elem::Road);
    }


    // Link the roads with next and previous:
    // We don't read the junctions tables,
    //  but we could load it into a series of vectors of connections.
    //  If needed, this:
    // std::vector<Odr::connection> j = readJunction(root->FirstChildElement(Odr::Elem::Junction));
    //  reads the first Junction, and then you'd need a loop.

    // Now loop over and figure out who's prev and who's next:
    r = root->FirstChildElement(Odr::Elem::Road);
    ndxS = 0;
    while (r)
    {
        int roadID = -1;
        XMLCheckResult(r->QueryIntAttribute(Odr::Attr::Id, &roadID));

        // Get the Links at the road level:
        uint rPrevID = 0;
        uint rNextID = 0;
        uint rPrevCP = 2; // contact point
        uint rNextCP = 2; // contact point
        tinyxml2::XMLElement *rLink = r->FirstChildElement(Odr::Elem::Link);
        if (!rLink)
        {
            ndxS += 1;
            r = r->NextSiblingElement(Odr::Elem::Road);
            continue;
        }

        if (getRoadLinkData(rPrevID, rPrevCP, rLink->FirstChildElement(Odr::Elem::Predecessor)))
        {
            std::cerr << "[ Errors ] in linking road " << roadID << " with its predecessor " << std::endl;
            return 1;
        }

        if (getRoadLinkData(rNextID, rNextCP, rLink->FirstChildElement(Odr::Elem::Successor)))
        {
            std::cerr << "[ Errors ] in linking road " << roadID << " with its successor " << std::endl;
            return 1;
        }


        // Loop over the lanes:
        tinyxml2::XMLElement *lanes = r->FirstChildElement(Odr::Elem::Lanes);

        uint ndxL = 0;
        tinyxml2::XMLElement *laneSection = lanes->FirstChildElement(Odr::Elem::LaneSection);
        while (laneSection)
        {
            // Grab the Left data, so that linkLanes loops over it:
            tinyxml2::XMLElement *left = laneSection->FirstChildElement(Odr::Elem::Left);
            if (left)
               ndxL = linkLanes(left->FirstChildElement(Odr::Elem::Lane), ndxS, ndxL, rPrevID, rNextID); //, rPrevCP, rNextCP);

            // Grab the Right data, so that linkLanes loops over it:
            tinyxml2::XMLElement *right = laneSection->FirstChildElement(Odr::Elem::Right);
            if (right)
               ndxL = linkLanes(right->FirstChildElement(Odr::Elem::Lane), ndxS, ndxL, rPrevID, rNextID); //, rPrevCP, rNextCP);

            // Go to the Next laneSection:
            laneSection = laneSection->NextSiblingElement(Odr::Elem::LaneSection);
        }


        // end of the loop; seek the next element:
        ndxS += 1;
        r = r->NextSiblingElement(Odr::Elem::Road);

    }

    return 0;
}


