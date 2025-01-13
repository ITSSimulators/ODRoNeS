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

#include "rns.h"
#include <boost/format.hpp>
#ifdef USE_ONEVERSION
#include "readOneVersion.h"
#endif // USE_ONEVERSION

using namespace odrones;

RNS::RNS()
{
    initialise();
    return;
}

void RNS::initialise()
{
    _sections = nullptr;
    _sectionsSize = 0;
    _verbose = true;
    _ready = false;
}


RNS::~RNS()
{
    clearMemory();
}

RNS& RNS::operator=(RNS& r)
{
    assignInputRNSToThis(r);
    return *this;
}

RNS::RNS(const RNS& r)
{
    assignInputRNSToThis(r);
}


RNS::RNS(std::string mapFile, concepts::drivingSide drivingSide, bool loadSidewalk, bool verbose)
{
    initialise();
    _verbose = verbose;
    _ready = makeRoads(mapFile, drivingSide, loadSidewalk);
    return;
}


void RNS::clearMemory()
{
    if (_sectionsSize > 0)
    {
        delete[] _sections;
        _sections = nullptr;
        _sectionsSize = 0;
    }
    _sectionsSize = 0;
    _tSigns.clear();
    _ready = false;

}


void RNS::assignInputRNSToThis(const RNS& r)
{
    _drivingSide = r._drivingSide;

    // Copy all the sections with all the lanes:
    _sectionsSize = r._sectionsSize;
    if (r._sectionsSize)
    {
        _sections = new section[r._sectionsSize];
        for (size_t i = 0; i < _sectionsSize; ++i)
            _sections[i] = r._sections[i];
    }
    else
        _sections = nullptr;

    // Now, we copied the sections and the sections copied the lanes.
    // However, lanes have pointers to other lanes belonging to the same rns.
    // Instead, the new rls lanes need to point to the internal copied lanes.
    // Thus, here comes the rewiring:
    for (uint is = 0; is < _sectionsSize; ++is)
    {
        for (uint jl = 0; jl < _sections[is].size(); ++jl)
        {
            // Section:
            _sections[is][jl]->setSection( _sections[is] ); // this one's easy, lanes belong to sections.

            // Port Lane:
            const lane* wl = _sections[is][jl]->getPortLane();
            if (wl)
                _sections[is][jl]->setPortLane( getLaneWithSUID(wl->getID(), wl->getSectionID()) );

            // Starboard Lane:
            wl = _sections[is][jl]->getStarboardLane();
            if (wl)
                _sections[is][jl]->setStarboardLane( getLaneWithSUID(wl->getID(), wl->getSectionID()) );

            // Next Lane:
            if (_sections[is][jl]->hasNextLane())
            {
                const lane** nls; size_t nlsSize;
                std::tie(nls, nlsSize) = _sections[is][jl]->getNextLanes();
                for (uint knl = 0; knl < nlsSize; ++knl)
                    _sections[is][jl]->setNextLane(knl, getLaneWithSUID(nls[knl]->getID(), nls[knl]->getSectionID()));
            }

            // Prev Lane:
            if (_sections[is][jl]->hasPrevLane())
            {
                const lane** pls; size_t plsSize;
                std::tie(pls, plsSize) = _sections[is][jl]->getPrevLanes();
                for (uint knl = 0; knl < plsSize; ++knl)
                    _sections[is][jl]->setPrevLane(knl, getLaneWithSUID(pls[knl]->getID(), pls[knl]->getSectionID()));
            }
        }
    }

    _ready = r._ready;

}


void RNS::setSections(uint sectionsSize)
{
    _sectionsSize = sectionsSize;
    _sections = new section[_sectionsSize];
    for (uint i = 0; i < _sectionsSize; ++i)
        _sections[i].setID(static_cast<int>(i));
}


uint RNS::sectionsSize() const
{
    return _sectionsSize;
}


uint RNS::lanesSize() const
{
    uint lanesSize = 0;
    for (uint i = 0; i < _sectionsSize; ++i)
    {
        for (uint j = 0; j < _sections[i].size(); ++j)
            lanesSize += 1;
    }
    return lanesSize;
}


section& RNS::sections(uint ndx) const
{
    return _sections[ndx];
}


section& RNS::operator[](uint ndx)
{
    return _sections[ndx];
}


bool RNS::ready() const
{
    return _ready;
}


void RNS::ready(bool r)
{
    _ready = r;
}


concepts::drivingSide RNS::drivingSide() const
{
    return _drivingSide;
}


void RNS::drivingSide(concepts::drivingSide side)
{
    _drivingSide = side;
}

std::vector<lane::tSign> RNS::tSigns() const
{
    return _tSigns;
}

void RNS::tSigns(const std::vector<lane::tSign> &t)
{
    _tSigns = t;

}

bool RNS::makeRoads(std::string mapFile, concepts::drivingSide drivingSide, bool loadSidewalk)
{

    if ( !std::filesystem::path{mapFile}.extension().string().compare(".xodr"))
        return makeOpenDRIVERoads(mapFile, drivingSide, loadSidewalk);
    else if ( !std::filesystem::path{mapFile}.extension().string().compare(".bin"))
#ifdef USE_ONEVERSION
        return makeOneVersionRoads(mapFile);
#else
        std::cerr << "[ Error ] ODRoNeS was compiled without support for OneVersion. Unable to load a OneVersion file." << std::endl;
#endif // USE_ONEVERSION
    else
    {
        std::cerr << "[ Error ] unrecognised extension " << std::filesystem::path(mapFile).extension() << std::endl;
        std::cerr << "---  Try loading a .xodr file if using OpenDRIVE or a .bin file if using OneVersion" << std::endl;
    }
    return false;
}

bool RNS::makeOneVersionRoads(std::string mapFile)
{
    _ready = false;

    _drivingSide = concepts::drivingSide::leftHand; // Ask Tony whether this comes from the file itself.

    // Load the roads file:
    readOneVersion read(mapFile);
    if (!read.ready())
    {
        std::cerr << "[ RNS ] unable to load the OneVersion map" << std::endl;
        return false;
    }

    // Allocate and do the geometry of for the lanes:
    uint sectionsSize = 0;
    for (uint i = 0; i < read.sections.size(); ++i)
        sectionsSize += read.sections[i].lgSize;
    setSections(sectionsSize);

    uint sectionsNdx = 0;
    for (uint i = 0; i < read.sections.size(); ++i)
    {
        if (!read.sections[i].lanes.size())
        {
            std::cout << "[ Strange ] section: " << read.sections[i].id << ", aka " << read.sections[i].idString() << " has no lane... " << std::endl;
            continue;
        }

        for (uint j = 0; j < read.sections[i].lgSize; ++j)
        {
            // get the number of lanes in this laneGroup:
            uint jthLGSize = 0;
            for (uint k = 0; k < read.sections[i].lanes.size(); ++k)
                if (read.sections[i].lanes[k].lgIndex == j) jthLGSize += 1;

            // allocate the right amount of lanes for this section:
            _sections[sectionsNdx].set(jthLGSize);
            // read just the lanes within this lane group:
            _sections[sectionsNdx].setOneVersionRoad(read.sections[i], j);

            // and carry on:
            sectionsNdx += 1;
        }
    }


    // Now do the linking.
    // Here, the stored knowledge is RoadNodes linking to RoadNodes,
    //   and NetworkNodes linking to NetworkNodes and Junctions (I believe).
    for (uint i = 0; i < read.sections.size(); ++i)
    {
        for (uint m = 0; m < read.sections[i].lgSize; ++m)
        {
            OneVersion::OVID ovID = read.sections[i].ovID;
            ovID.lgIndex = m;
            section *si = getSectionWithOVId(ovID);
            if (!si)
            {
                std::cerr << "[ Error ] No section for: " << i << ", ovID: "
                          << read.sections[i].ovID.to_string() << std::endl;
            }

            // Link this section with the next RoadNode:
            std::vector<uint> nextRNs = getSectionIDsWithOVRoadNodeId(read.sections[i].forwardsRoadOVID);
            // now do a loop over the nextRNs, and a double loop later to link all the lanes in range
            for (uint k = 0; k < nextRNs.size(); ++k)
                linkLanesInSections(*si, _sections[nextRNs[k]]);

            // Link this section with the previous RoadNode:
            std::vector<uint> prevRNs = getSectionIDsWithOVRoadNodeId(read.sections[i].backwardsRoadOVID);
            // now do a loop over the prevRNs, and a double loop later to link all the lanes in range
            for (uint k = 0; k < prevRNs.size(); ++k)
                linkLanesInSections(*si, _sections[prevRNs[k]]);

            // If this is the last section of the NetworkNode,
            //   - link it to the first section in the next NetworkNode.
            //   - link it to the next Junction

            // If this is the first section of the NetworkNode,
            //   - link it to the last section in the previous NetworkNode.
            //   - link it to the previous Junction
        }
    }

    // Set port and starboard lanes, and assume left hand driving:
    for (uint i = 0; i < _sectionsSize; ++i)
    {
        if (!_sections[i].isTransitable()) continue;
        _sections[i].setPortAndStarboard(_drivingSide); // true, false);
    }


    _ready = true;
    return _ready;
}

bool RNS::makeOpenDRIVERoads(std::string odrMap, concepts::drivingSide drivingSide, bool loadSidewalk)
{
    if (verbose())
        std::cout << "[ Warning ] work in progress; ignoring loadSidewalk value: " << loadSidewalk << std::endl;

    bool isOdrFile = false;
    if ((odrMap.length() < 255) && (std::filesystem::exists(odrMap)))
        isOdrFile = true;

    ReadXOdr read(odrMap, isOdrFile);

    if (!read.isReady())
    {
        std::cerr << "[ LRN ] unable to load the opendrive map" << std::endl;
        return false;
    }

    return makeOpenDRIVERoads(read, drivingSide, loadSidewalk);
}

bool RNS::makeOpenDRIVERoads(ReadOdr &read, concepts::drivingSide drivingSide, bool loadSidewalk)
{

    _ready = false;

    _letter = read;

    _drivingSide = drivingSide;

    // Allocate and do the geometry for the lanes:
    // We will allocate as many sections as laneSections:
    uint sectionsSize = 0;
    for (uint i = 0; i < read.sections.size(); ++i)
        sectionsSize += read.sections[i].lsSize;
    setSections(sectionsSize);

    uint sectionsNdx = 0;
    for (uint i = 0; i < read.sections.size(); ++i)
    {
        if (!read.sections[i].lanes.size())
        {
            std::cout << "[ Strange ] section: " << read.sections[i].id << ":" << read.sections[i].odrID << " has no lane... " << std::endl;
            continue;
        }
        if (static_cast<uint>(read.sections[i].id) != i)
        {
            std::cerr << "PANIC!" << std::endl;
            return false;
        }

        for (uint j = 0; j < read.sections[i].lsSize; ++j)
        {
            // get the number of lanes in this laneSection:
            uint jthLSSize = 0;
            for (uint k = 0; k < read.sections[i].lanes.size(); ++k)
                if (read.sections[i].lanes[k].ndxLS == j) jthLSSize += 1;

            // allocate the right amount of lanes for this section:
            _sections[sectionsNdx].set(jthLSSize);

            // read just the lanes within this laneSection:
            _sections[sectionsNdx].setOdrRoad(read.sections[i], j);

            // ask _sections to spit out a tsigns vector for display purposes.
            // _tSigns.insert(_tSigns.end(), _sections[sectionsNdx].lrnTSigns().begin(), _sections[sectionsNdx].lrnTSigns().end())
            sectionsNdx += 1;
        }
    }

    // Now do the linking:
    if (read.k() == ReadOdr::kind::xodr)
    {
        for (uint i = 0; i < read.sections.size(); ++i)
        {
            for (uint j = 0; j < read.sections[i].lanes.size(); ++j)
            {
                lane* lij = getLaneWithODRIds(read.sections[i].odrID, read.sections[i].lanes[j].odrID);
                if (!lij)
                    std::cerr << "[ Error ] No lane for: " << i << ", " << j << ", segfault on the way... " << std::endl;
                for (uint k = 0; k < read.sections[i].lanes[j].nextLane.size(); ++k)
                {
                    Odr::smaL *sml = read.sections[i].lanes[j].nextLane[k];
                    if (!sml) continue;

                    lane* nextLane = getLaneWithODRIds(read.sections[sml->sID].odrID, sml->odrID);
                    linkLanesIfInRange(lij, nextLane);
                }

                for (uint k = 0; k < read.sections[i].lanes[j].prevLane.size(); ++k)
                {
                    Odr::smaL *sml = read.sections[i].lanes[j].prevLane[k];
                    if (!sml) continue;

                    lane* prevLane = getLaneWithODRIds(read.sections[sml->sID].odrID, sml->odrID);
                    linkLanesIfInRange(lij, prevLane);
                }
            }
        }
    }
    else
        linkLanesGeometrically();


    // Set port and starboard lanes, and assume left hand driving:
    for (uint i = 0; i < _sectionsSize; ++i)
    {
        if (!_sections[i].isTransitable()) continue;
        _sections[i].setPortAndStarboard(drivingSide); // true, false);
    }


    // Get the traffic signs:
    for (uint i = 0; i < _sectionsSize; ++i)
    {
        std::vector<lane::tSign> ts = _sections[i].getTSigns();
        _tSigns.insert(std::end(_tSigns), std::begin(ts), std::end(ts));
    }


    // MISSING Crosswalk lanes:


    /*
    calcNumberOfLanes();
    setAvailableActors();
    */

    _ready = true;
    return true;
}


void RNS::setPortAndStarboard(concepts::drivingSide drivingSide)
{

    // Set port and starboard lanes, and assume left hand driving:
    for (uint i = 0; i < _sectionsSize; ++i)
    {
        if (!_sections[i].isTransitable()) continue;
        _sections[i].setPortAndStarboard(drivingSide); // true, false);
    }

}

void RNS::setPortAndStarboard()
{
    return setPortAndStarboard(_drivingSide);
}


void RNS::printLanes() const
{

    for (uint i = 0; i < _sectionsSize; ++i)
    {
        for (uint j = 0; j < _sections[i].size(); ++j)
        {
            lane *l = _sections[i][j];
            if (!(l->isTransitable())) continue;
            arr2 o = l->getOrigin();
            arr2 d = l->getDestination();
            arr2 tri, bli;
            l->getBoundingBox(bli, tri);
            auto [pls, plSize] = l->getPrevLanes();
            std::string plString = ", has " + std::to_string(plSize) + " previous lanes";
            for (size_t k=0; k<plSize; ++k)
            {
                if (k == 0) plString += ": ";
                else plString += ", ";
                plString += pls[k]->getCSUID();
            }
            auto [nls, nlSize] = l->getNextLanes();
            std::string nlString = ", has " + std::to_string(nlSize) + " next lanes";
            for (size_t k=0; k<nlSize; ++k)
            {
                if (k == 0) nlString += ": ";
                else nlString += ", ";
                nlString += nls[k]->getCSUID();
            }
            std::cout << l->getShapeString() << " lane: " << l->getCSUID();
            std::cout << " starts in: (" << o[0] << ", " << o[1] <<  "), ends in: (" << d[0] << ", " << d[1] << ")"
                      << ", _to: (" << l->getTo()[0] << ", " << l->getTo()[1] << ")"
                      << ", is " << l->getLength() << " m long, has max speed: " << l->getSpeed()
                      // << ", bounding box: " << bli[0] << ", " << bli[1] << " to " << tri[0] << ", " << tri[1] << std::endl;
                      << plString << nlString << std::endl;

        }
    }

}

void RNS::write(const std::string &mapFile) const
{
    // Create a TinyXML2 object:
    tinyxml2::XMLDocument xmlMap;
    tinyxml2::XMLNode *root = xmlMap.NewElement(Odr::Elem::OpenDrive);
    xmlMap.InsertEndChild(root);

    // Print out a header:
    tinyxml2::XMLElement* header = xmlMap.NewElement(Odr::Elem::Header);
    header->SetAttribute("revMajor", 1);
    header->SetAttribute("revMinor", 6);
    header->SetAttribute("vendor", "University of Leeds, Simulator5");
    // ... with some user data:
    tinyxml2::XMLElement* userData = xmlMap.NewElement(Odr::Elem::UserData);
    userData->SetAttribute("extension", "24_11: bezier3, no junctions");
    header->InsertEndChild(userData);
    root->InsertEndChild(header); // into root.

    // If we built that from using ReadBOdr, the original input was incomplete:
    if (_letter.k() == ReadOdr::kind::bodr)
    {
        // Iterate over the roads, because there are no road sections:
        for (uint i = 0; i < _sectionsSize; ++i)
        {
            // Road definition and attributes:
            tinyxml2::XMLElement* xmlRoad = xmlMap.NewElement(Odr::Elem::Road);
            const Odr::smaS *smas = _letter.odrSection(_sections[i].odrID());
            if (!smas)
            {
                std::cerr << "[ RNS::Write Error ] smas not found "
                          << "for road with odrID: " << _sections[i].odrID() << std::endl;
            }
            xmlRoad->SetAttribute(Odr::Attr::Name, smas->name.c_str());
            xmlUtils::setAttrDouble(xmlRoad, Odr::Attr::Length, _sections[i].zero()->getLength());
            xmlRoad->SetAttribute(Odr::Attr::Id, _sections[i].odrID());
            xmlRoad->SetAttribute(Odr::Attr::Junction, -1);
            if (_drivingSide == concepts::drivingSide::leftHand)
                xmlRoad->SetAttribute(Odr::Attr::Rule, Odr::Kind::LHT);
            else if (_drivingSide == concepts::drivingSide::rightHand)
                xmlRoad->SetAttribute(Odr::Attr::Rule, Odr::Kind::RHT);

            // Road -> Type:
            tinyxml2::XMLElement* type = xmlMap.NewElement(Odr::Elem::Type);
            xmlUtils::setAttrDouble(type, Odr::Attr::S, 0.00);
            type->SetAttribute(Odr::Attr::Type, smas->type.c_str());
            // Road -> type -> speed:
            tinyxml2::XMLElement* speed = xmlMap.NewElement(Odr::Elem::Speed);
            xmlUtils::setAttrDouble(speed, Odr::Attr::Max, _sections[i].maxSpeed());
            speed->SetAttribute(Odr::Attr::Unit, Odr::Kind::ms);
            type->InsertEndChild(speed); // push speed
            xmlRoad->InsertEndChild(type); // push type

            // Road -> link - We don't need road linking, lanes are linked individually.
            tinyxml2::XMLElement* roadLink = xmlMap.NewElement(Odr::Elem::Link);
            xmlRoad->InsertEndChild(roadLink);

            // Road -> PlanView - i e, geometries
            tinyxml2::XMLElement* planView = xmlMap.NewElement(Odr::Elem::PlanView);
            if (!_sections[i].zero()->xmlPlanView(planView, xmlMap))
                std::cerr << "zero was unable to run xmlPlanView" << std::endl;
            xmlRoad->InsertEndChild(planView);

            // Road -> Lanes:
            tinyxml2::XMLElement *lanes = xmlMap.NewElement(Odr::Elem::Lanes);
            // Road -> Lanes -> Lanes offset:
            for (uint j = 0; j < smas->loffset.size(); ++j)
            {
                tinyxml2::XMLElement* laneOffset = xmlMap.NewElement(Odr::Elem::LaneOffset);
                xmlUtils::setAttrOffsetS(laneOffset, smas->loffset[j]);
                lanes->InsertEndChild(laneOffset);
            }

            // Road -> Lanes -> Lane Section:
            tinyxml2::XMLElement* laneSection = xmlMap.NewElement(Odr::Elem::LaneSection);
            xmlUtils::setAttrDouble(laneSection, Odr::Attr::S, 0.);

            // Gather the indices for left & right lanes:
            std::vector<uint> leftUint, rightUint;
            for (uint j = 0; j < smas->lanes.size(); ++j)
            {
                if (smas->lanes[j].odrID < 0)
                    rightUint.push_back(j);
                else if (smas->lanes[j].odrID > 0)
                    leftUint.push_back(j);
            }

            // Road -> Lanes -> LaneSection -> Left
            tinyxml2::XMLElement* leftXML = xmlMap.NewElement(Odr::Elem::Left);
            for (uint j = 0; j < leftUint.size(); ++j)
            {
                tinyxml2::XMLElement* laneXML = xmlMap.NewElement(Odr::Elem::Lane);
                const lane* l = _sections[i].getOdrLane(smas->lanes[leftUint[j]].odrID);
                if (!l)
                {
                    std::cerr << "[ Error ] RNS::write - lane not found" << std::endl;
                    return;
                }
                l->xmlLaneAttributesAndLinks(laneXML, xmlMap);
                smas->lanes[leftUint[j]].writeXMLWidth(laneXML, xmlMap);
                smas->lanes[leftUint[j]].writeXMLBorder(laneXML, xmlMap);
                leftXML->InsertEndChild(laneXML);
            }
            if (leftUint.size())
                laneSection->InsertEndChild(leftXML);


            // Road -> Lanes -> LaneSection -> Centre
            tinyxml2::XMLElement* centreXML = xmlMap.NewElement(Odr::Elem::Center);
            tinyxml2::XMLElement* centreLane = xmlMap.NewElement(Odr::Elem::Lane);
            centreLane->SetAttribute(Odr::Attr::Id, 0);
            centreLane->SetAttribute(Odr::Attr::Type, Odr::Kind::None);
            centreLane->SetAttribute(Odr::Attr::Level, Odr::Kind::False);
            tinyxml2::XMLElement* lane0Link = xmlMap.NewElement(Odr::Elem::Link);
            centreLane->InsertEndChild(lane0Link); // push laneLink into Lane
            centreXML->InsertEndChild(centreLane); // push centreLane into Center
            laneSection->InsertEndChild(centreXML); // push Center into Lane Section


            // Road -> Lanes -> LaneSection -> Right
            tinyxml2::XMLElement* rightXML = xmlMap.NewElement(Odr::Elem::Right);
            for (uint j = 0; j < rightUint.size(); ++j)
            {
                tinyxml2::XMLElement* laneXML = xmlMap.NewElement(Odr::Elem::Lane);
                const lane* l = _sections[i].getOdrLane(smas->lanes[rightUint[j]].odrID);
                if (!l)
                {
                    std::cerr << "[ Error ] RNS::write - lane not found" << std::endl;
                    return;
                }
                l->xmlLaneAttributesAndLinks(laneXML, xmlMap);
                smas->lanes[rightUint[j]].writeXMLWidth(laneXML, xmlMap);
                smas->lanes[rightUint[j]].writeXMLBorder(laneXML, xmlMap);
                rightXML->InsertEndChild(laneXML);
            }
            if (rightUint.size())
                laneSection->InsertEndChild(rightXML);

            lanes->InsertEndChild(laneSection);
            xmlRoad->InsertEndChild(lanes);

            root->InsertEndChild(xmlRoad); // into root.
        }
    }
    else
    {

        /*
            // loop over letter.sections:
            const Odr::smaS *smas = &(_letter.sections[5]);

            // Gather the indices for left & right lanes:
            std::vector<uint> leftUint, rightUint;
            for (uint j = 0; j < smas->lanes.size(); ++j)
            {
                if (smas->lanes[j].odrID < 0)
                    rightUint.push_back(j);
                else if (smas->lanes[j].odrID > 0)
                    leftUint.push_back(j);
            }

            // Road -> Lanes -> LaneSection -> Left
            tinyxml2::XMLElement* leftXML = xmlMap.NewElement(Odr::Elem::Left);
            for (uint j = 0; j < leftUint.size(); ++j)
            {
                tinyxml2::XMLElement* lane = xmlMap.NewElement(Odr::Elem::Lane);
                smas->lanes[leftUint[j]].writeXML(lane, xmlMap);
                leftXML->InsertEndChild(lane);
            }
            if (leftUint.size())
                laneSection->InsertEndChild(leftXML);
        */
        std::cout << "not implemented!" << std::endl;
    }

    xmlUtils::CheckResult(xmlMap.SaveFile(mapFile.c_str()));
}

void RNS::linkLanesGeometrically(scalar tol)
{
    for (uint i = 0; i < sectionsSize(); ++i)
    {
        for (uint j = i + 1; j < sectionsSize(); ++j)
            linkLanesInSections(_sections[i], _sections[j], tol);
    }
    return;
}


void RNS::linkLanesIfInRange(lane *li, lane *lj, scalar tol)
{
    if ((mvf::areCloseEnough(li->getDestination(), lj->getOrigin(), tol)) ||
            (mvf::areCloseEnough(li->getDestination(), lj->getDestination(), tol)))
        li->setNextLane(lj, false);

    if ((mvf::areCloseEnough(li->getOrigin(), lj->getOrigin(), tol)) ||
            (mvf::areCloseEnough(li->getOrigin(), lj->getDestination(), tol)))
        li->setPrevLane(lj, false);

    if ((mvf::areCloseEnough(lj->getDestination(), li->getOrigin(), tol)) ||
            (mvf::areCloseEnough(lj->getDestination(), li->getDestination(), tol)))
        lj->setNextLane(li, false);

    if ((mvf::areCloseEnough(lj->getOrigin(), li->getOrigin(), tol)) ||
            (mvf::areCloseEnough(lj->getOrigin(), li->getDestination(), tol)))
        lj->setPrevLane(li, false);
}

void RNS::linkLanesInSections(section &si, section &sj, scalar tol)
{
    for (uint i = 0; i < si.size(); ++i)
    {
        for (uint j = 0; j < sj.size(); ++j)
            linkLanesIfInRange(si[i], sj[j]);
    }
}


lane* RNS::getLaneWithSUID(int sID, int lID) const
{
    int sndx = -1;
    for (uint i = 0; i < _sectionsSize; ++i)
    {
        if (sID == _sections[i].getID())
        {
            sndx = static_cast<int>(i);
            break;
        }
    }

    if (sndx == -1) return nullptr;

    for (size_t i = 0; i < _sections[sndx].size(); ++i)
    {
        if (_sections[sndx][i]->getID() == lID)
            return _sections[sndx][i];
    }

    return nullptr;
}

const lane* RNS::getCLaneWithODRIds(uint rID, int lID) const
{
    for (uint i = 0; i < _sectionsSize; ++i)
    {
        if (_sections[i].odrID() != rID) continue;
        for (uint j = 0; j < _sections[i].size(); ++j)
        {
            if (_sections[i][j]->odrID() == lID) return _sections[i][j];
        }
    }

    return nullptr;
}

lane* RNS::getLaneWithODRIds(uint rID, int lID) const
{
    for (uint i = 0; i < _sectionsSize; ++i)
    {
        if (_sections[i].odrID() != rID) continue;
        for (uint j = 0; j < _sections[i].size(); ++j)
        {
            if (_sections[i][j]->odrID() == lID) return _sections[i][j];
        }
    }

    return nullptr;
}


lane* RNS::getLaneWithOVId(const OneVersion::OVID &lID) const
{
    OneVersion::OVID sID = lID;
    sID.laneID = -1;
    for (uint i = 0; i < _sectionsSize; ++i)
    {
        if (_sections[i].ovID() != sID) continue;
        for (uint j = 0; j < _sections[i].size(); ++j)
        {
            if (_sections[i][j]->ovID() == lID) return _sections[i][j];
        }
    }

    return nullptr;
}


section* RNS::getSectionWithOVId(const OneVersion::OVID &sID) const
{
    for (uint i = 0; i < _sectionsSize; ++i)
    {
        if (_sections[i].ovID() == sID) return &(_sections[i]);
    }
    return nullptr;

}

std::vector<uint> RNS::getSectionIDsWithOVRoadNodeId(int rnMID, int rnmID) const
{
    OneVersion::OVID rnID;
    rnID.roadIDM = rnMID;
    rnID.roadIDm = rnmID;
    return getSectionIDsWithOVRoadNodeId(rnID);
}


std::vector<uint> RNS::getSectionIDsWithOVRoadNodeId(const OneVersion::OVID &rnID) const
{
    std::vector<uint> s;
    if ((rnID.roadIDM == -1) || (rnID.roadIDm == -1)) return s;
    for (uint i = 0; i < _sectionsSize; ++i)
    {
        if (_sections[i].ovID().sameRoadIDs(rnID))
            s.push_back(i);

    }
    return s;
}

std::vector<uint> RNS::getSectionIDsWithOVNodeId(int nID) const
{
    std::vector<uint> s;
    if (nID < 0) return s;

    for (uint i = 0; i < _sectionsSize; ++i)
    {
        if (_sections[i].ovID().nnodeID == nID)
            s.push_back(i);
    }
    return s;
}

int RNS::getSectionIDWithODRIDWithRoadCoord(uint rID, scalar s) const
{
    for (uint i = 0; i < _sectionsSize; ++i)
    {
        if ((_sections[i].odrID() == rID) && (_sections[i].isInOdrRange(s)))
            return i;
    }
    return -1;
}


int RNS::findPortAndStarboardLanes(lane* &port, lane* &starboard, lane *l1, lane *l2, scalar dToEoL1, scalar dToEoL2) const
{
    port = nullptr;
    if (dToEoL1 > l1->getLength())
    {
        std::cerr << "[ ERROR ] getPortLane cannot be used with a dToEoL larger than the length of " << l1->getSUID() << std::endl;
        return 2;
    }
    else if (dToEoL2 > l2->getLength())
    {
        std::cerr << "[ ERROR ] getPortLane cannot be used with a dToEoL larger than the length of " << l2->getSUID() << std::endl;
        return 3;
    }
    arr2 o1, o2;
    l1->getPointAtDistance(o1, l1->getLength() - dToEoL1);
    l2->getPointAtDistance(o2, l2->getLength() - dToEoL2);

    arr2 t1, t2;
    l1->getTangentInPoint(t1, o1);
    l2->getTangentInPoint(t2, o2);

    arr2 o12 = {o2[0] - o1[0], o2[1] - o1[1]};
    scalar m = mvf::magnitude(o12);
    if (m < 0.1)  // the points o1 and o2 on both lanes are too close.
        return 1;
    o12 = {o12[0] / m, o12[1] / m};
    // mvf::normalise(o12);

    // subtended angle returns -pi <= angle <= pi
    if ((mvf::subtendedAngle(t1, o12) < 0) &&
            (mvf::subtendedAngle(t2, {-o12[0], -o12[1]}) > 0))
    {
        port = l1;
        starboard = l2;
        return 0;
    }
    else if ((mvf::subtendedAngle(t1, o12) > 0)
             && (mvf::subtendedAngle(t2, {-o12[0], -o12[1]}) < 0))
    {
        port = l2;
        starboard = l1;
        return 0;
    }
    else
    {
        std::cerr << "failed to find which one is port between " << l1->getSUID() << " and " << l2->getSUID() << std::endl;
        return 1;
    }
}



lane::lCoord RNS::getLaneCoordsForPoint(const arr2 &o, scalar tol) const
{
    // 1 - Generic algorithm to find the closest lane:
    //   For each section check if o is within its bounding box,
    //   and if it is, check whether o is within any of its lanes bounding boxes.
    lane::lCoord lcoo = {nullptr, {0, 0}, 0., 1e3};
    for (uint is = 0; is < sectionsSize(); ++is)
    {
        arr2 bli, tri;
        _sections[is].getBoundingBox(bli, tri);
        if (!mvf::isPointInBoxBLcTRc(o, {bli[0] - tol, bli[1] - tol}, {tri[0] + tol, tri[1] + tol})) continue;
        // if (!mvf::isPointInBoxTLcBRc(o, {bli[0] - tol, tri[1] + tol}, {tri[0] + tol, bli[1] - tol})) continue; // tl, br

        arr2 pj;
        for (uint jl = 0; jl < _sections[is].size(); ++jl)
        {
            _sections[is][jl]->getBoundingBox(bli, tri);
            // if (!mvf::isPointInBoxTLcBRc(o, {bli[0] - tol, tri[1] + tol}, {tri[0] + tol, bli[1] - tol})) continue; // tl, br
            if (!mvf::isPointInBoxBLcTRc(o, {bli[0] - tol, bli[1] - tol}, {tri[0] + tol, tri[1] + tol})) continue;

            if (!_sections[is][jl]->projectPointOntoLane(pj, o)) continue;

            scalar oj = mvf::distance(pj, o);
            if (oj > tol) continue;
            if (oj < lcoo.loff)
            {
                lcoo.loff = oj;
                lcoo.l = _sections[is][jl];
                lcoo.pos = pj;
                lcoo.s = lcoo.l->unsafeDistanceFromTheBoL(pj);
            }
        }
    }

    return lcoo;
}

arr2 RNS::getPosForLaneCoords(const lane::lCoord &lc) const
{
    arr2 p;
    lc.l->getPointWithOffset(p, lc.pos, lc.loff);
    return p;
}


arr2 RNS::getPosForRoadCoords(uint rID, scalar s, scalar offset, scalar height) const
{
    arr2 p = {0., 0.};
    int sID = getSectionIDWithODRIDWithRoadCoord(rID, s);
    if (sID == -1)
        return p;
    _sections[sID].zero()->getPointWithOffset(p, s, offset);
    return p;


}


const lane* RNS::getLaneWithPoint(const arr2 &p, scalar tol) const
{
    lane::lCoord lcoo = getLaneCoordsForPoint(p, tol);
    if (lcoo.loff < tol) return lcoo.l;
    else return nullptr;
}



std::vector<uint> RNS::buildForwardsFromSecID(uint first, const std::vector<uint> &ids) const
{
    std::vector<uint> poppable = ids;
    std::vector<uint> output;

    // Check if "first" is in ids:
    uint accounted = 0;
    for (uint i = 0; i < ids.size(); ++i)
    {
        if (first == ids[i])
        {
            accounted = 1;
            poppable.erase(poppable.begin()+i);
            output.push_back(ids[i]);
            break;
        }
    }


    for (uint i = 0; i < ids.size() - accounted; ++i)
    {
        bool bond = false;
        for (uint j = 0; j < poppable.size(); ++j)
        {
            if (_sections[poppable[j]].isConnected(_sections[first]))
            {
                bond = true;
                first = poppable[j];
                output.push_back(first);
                poppable.erase(poppable.begin()+j);
                break;
            }
        }
        if (!bond)
        {
            output.clear();
            break;
        }
    }
    return output;
}

std::vector<uint> RNS::buildBackwardsFromSecID(uint last, const std::vector<uint> &ids) const
{

    std::vector<uint> output = buildForwardsFromSecID(last, ids);
    std::reverse(output.begin(), output.end());
    return output;
}


bool RNS::findFirstLinkIDInSection(int &first, uint last, const std::vector<uint> &ids_o) const
{
    bool success = false;
    first = -1;
    for (uint i = 0; i < ids_o.size(); ++i )
    {
        if (_sections[ids_o[i]].isConnected(_sections[last]))
        {
            first = ids_o[i];
            success = true;
            break;
        }
    }
    return success;
}

bool RNS::findLastAndFirstLinkingSectionIDs(int &last_o, int &first_e, const std::vector<uint> &ids_o, const std::vector<uint> &ids_e) const
{
    bool success = false;
    last_o = -1;
    first_e = -1;
    for (uint i = 0; i < ids_o.size(); ++i)
    {
        for (uint j = 0; j < ids_e.size(); ++j)
        {
            if (_sections[ids_o[i]].isConnected(_sections[ids_e[j]]))
            {
                success = true;
                last_o = ids_o[i];
                first_e = ids_e[j];
                break;
            }
        }
        if (success) break;
    }
    return success;
}


void RNS::getDimensions(int &minX, int &minY, int &maxX, int &maxY) const
{
    arr2 blc, trc;
    getDimensions(blc[0], blc[1], trc[0], trc[1]);

    maxX = static_cast<int>(std::ceil(trc[0]));
    maxY = static_cast<int>(std::ceil(trc[1]));
    minX = static_cast<int>(std::floor(blc[0]));
    minY = static_cast<int>(std::floor(blc[1]));

    return;
}


void RNS::getDimensions(scalar &minX, scalar &minY, scalar &maxX, scalar &maxY) const
{
    minX = minY = maxX = maxY = 0;
    arr2 blc, trc, bli, tri;
    _sections[0][0]->getBoundingBox(blc, trc);

    // Geometrical & Basic Endings Check:
    for (unsigned int i=0; i<_sectionsSize; ++i)
    {
        for (unsigned int j=0; j<_sections[i].size(); ++j)
        {
            if (!_sections[i][j]->getBoundingBox(bli, tri))
            {
                std::cerr << "[ ERROR ] lrn could not get the bounding box for lane "
                          << _sections[i][j]->getSUID() << std::endl;
            }
            mvf::increaseBoxWithBox(blc, trc, bli, tri);
        }
    }

    maxX = trc[0];
    maxY = trc[1];
    minX = blc[0];
    minY = blc[1];

    return;

}

bool RNS::makePriorities(scalar anticipationTime)
{
    if (!makePrioritiesSameEndingDifferentSectionLanes(anticipationTime))
        return false;

    makePrioritiesSameSectionMergeLanes();

    makePrioritiesDifferentEndingDifferentSectionCrossingLanes(anticipationTime);

    return true;
}


bool RNS::makePrioritiesSameEndingDifferentSectionLanes(scalar anticipationTime)
{
    // LANES From Diferent Sections and Same Ending:
    //   - In the absence of traffic signs, give way to the right (or left) depending on driving left (or right) hand:
    for (uint i = 0; i < _sectionsSize; ++i)
    {
        // if (!isSectionInAnyRoute(_sections[i].getID())) continue;

        for (uint j = i + 1; j < _sectionsSize; ++j)
        {
            // if (!isSectionInAnyRoute(_sections[j].getID())) continue;

            for (uint li = 0; li < _sections[i].size(); ++li)
            {
                if (_sections[i][li]->hasConflicts()) continue; // priority is already defined.
                for (uint lj = 0; lj < _sections[j].size(); ++lj)
                {
                    if (_sections[j][lj]->hasConflicts()) continue; // priority is already defined.
                    if ( !mvf::areCloseEnough( _sections[i][li]->getDestination(), _sections[j][lj]->getDestination(), lane::odrTol ) )
                        continue;

                    // Find which one is portlane: starting from the end of the lane (they share destination, and this happens after routes())
                    //   take a point at the end of both lanes that is close to the end but that is separated.
                    lane* portLane = nullptr;
                    lane* starboardLane = nullptr;
                    scalar dToEoL = 1;
                    int errPL = findPortAndStarboardLanes(portLane, starboardLane, _sections[i][li], _sections[j][lj], dToEoL, dToEoL);
                    while (errPL > 0)
                    {
                        dToEoL += 1;
                        errPL = findPortAndStarboardLanes(portLane, starboardLane, _sections[i][li], _sections[j][lj], dToEoL, dToEoL);
                        if (errPL > 1) break; // we went to far, and the method failed.
                    }
                    if (!portLane)
                    {
                        std::cerr << "failed to find which one is port between "
                                  << _sections[i][li]->getSUID() << " and " << _sections[j][lj]->getSUID() << std::endl;
                        return false; // continue;
                    }

                    // Depending on whether we're driving right or left, it's either port or starboard that have priority:
                    lane* hpLane = nullptr;
                    lane* lpLane = nullptr;
                    if (_drivingSide == concepts::drivingSide::leftHand) // portLane has lower priority:
                    {
                        hpLane = starboardLane;
                        lpLane = portLane;
                    }
                    else if (_drivingSide == concepts::drivingSide::rightHand)
                    {
                        hpLane = portLane;
                        lpLane = starboardLane;
                    }
                    else
                    {
                        std::cerr << "nonsense!" << std::endl;
                        return false; // continue;
                    }

                    // create a conflict and assign it to the lower priority lane:
                    conflict lpCnf = conflict::createEoLConflict(lpLane, hpLane, anticipationTime);
                    if (verbose())
                    {
                        for (uint pk = 0; pk < lpCnf.hpLane.size(); ++pk)
                            std::cout << "lane: " << lpCnf.hpLane[pk]->getSUID() << " has priority over " << lpLane->getSUID() << std::endl;
                    }
                    lpLane->addConflict(lpCnf);

                    //   and inform the higher priority lane:
                    conflict hpCnf = conflict::createEoLConflict(hpLane, lpLane, anticipationTime);
                    hpCnf.k = conflict::kind::free;
                    hpLane->addConflict(hpCnf);
                }
            }
        }
    }

    return true;

}

void RNS::makePrioritiesSameSectionMergeLanes()
{
    // LANES From Same Sections:
    //   - if there's no next lane and either port or starboard has one, merge there.
    //   - with same Same Ending:
    //   	* In the absence of traffic signs, merge to the right
    for (uint i = 0; i < _sectionsSize; ++i)
    {
        // if (!isSectionInAnyRoute(_sections[i].getID())) continue;

        if (_sections[i].size() == 1) continue;

        for (uint li = 0; li < _sections[i].size(); ++li)
        {
            if (_sections[i][li]->isToMerge()) continue;

            // If there's no road ahead, try to merge if either port or starboard go anywhere.
            if (!_sections[i][li]->hasNextLane())
            {
                // Preferably, merge towards the starboard (on a leftHand scenario);
                if ((_drivingSide == concepts::drivingSide::leftHand) && ((_sections[i][li]->getStarboardLaneSD()) && (_sections[i][li]->getStarboardLaneSD()->hasNextLane())) )
                    _sections[i][li]->addConflict(conflict::createMergeConflict(_sections[i][li], mvf::side::starboard));

                else if ((_sections[i][li]->getPortLaneSD()) && (_sections[i][li]->getPortLaneSD()->hasNextLane()))
                    _sections[i][li]->addConflict(conflict::createMergeConflict(_sections[i][li], mvf::side::port));

                else if ((_drivingSide == concepts::drivingSide::rightHand) && ((_sections[i][li]->getStarboardLaneSD()) && (_sections[i][li]->getStarboardLaneSD()->hasNextLane())) )
                    _sections[i][li]->addConflict(conflict::createMergeConflict(_sections[i][li], mvf::side::starboard));


                continue; // if neither port nor starboard have next lanes, forget it.
            }


            // Here, lanes within the same section and same destination:
            for (uint lj = li + 1; lj < _sections[i].size(); ++lj )
            {
                if (_sections[i][lj]->isToMerge()) continue;

                if (!mvf::areCloseEnough( _sections[i][li]->getDestination(), _sections[i][lj]->getDestination(), lane::odrTol ) )
                    continue;

                if ( ((_drivingSide == concepts::drivingSide::rightHand) && (_sections[i][li]->getPortLaneSD() == _sections[i][lj])) ||
                     ((_drivingSide == concepts::drivingSide::leftHand) && (_sections[i][li]->getStarboardLaneSD() == _sections[i][lj])) )
                {
                    _sections[i][li]->addConflict(conflict::createMergeConflict(_sections[i][li], mvf::side::port));
                    // _sections[i][lj]->setStopMargin(calcStopMargin(_sections[i][lj], _sections[i][li]));
                    if (verbose())
                        std::cout << "[ lrn ] assigning merge to " << _sections[i][li]->getSUID()
                                  << " giving higher priority to " << _sections[i][lj]->getSUID() << std::endl;
                }

                else if ( ((_drivingSide == concepts::drivingSide::rightHand) && (_sections[i][li]->getStarboardLaneSD() == _sections[i][lj])) ||
                          ((_drivingSide == concepts::drivingSide::leftHand) && (_sections[i][li]->getPortLaneSD() == _sections[i][lj])) )
                {
                    _sections[i][li]->addConflict(conflict::createMergeConflict(_sections[i][li], mvf::side::starboard));
                    // _sections[i][li]->setStopMargin(calcStopMargin(_sections[i][li], _sections[i][lj]));
                    if (verbose())
                        std::cout << "[ lrn ] assigning merge to " << _sections[i][lj]->getSUID()
                                  << " giving higher priority to " << _sections[i][li]->getSUID() << std::endl;
                }

                else std::cerr << "[ ERROR ] lanes: " << _sections[i][li]->getSUID() << " and " << _sections[i][lj]->getSUID()
                                   << " have the same destination, but they're not port/starboard related." << std::endl;

            }
        }
    }
}


void RNS::makePrioritiesDifferentEndingDifferentSectionCrossingLanes(scalar anticipationTime)
{

    // LANES From Diferent Sections and Different Ending May Cross
    //   IF they don't share origin nor destination.
    for (uint i = 0; i < _sectionsSize; ++i)
    {
        // if (!isSectionInAnyRoute(_sections[i].getID())) continue;

        for (uint j = i + 1; j < _sectionsSize; ++j)
        {
            // if (!isSectionInAnyRoute(_sections[j].getID())) continue;

            for (uint li = 0; li < _sections[i].size(); ++li)
            {
                if (_sections[i][li]->isCrosswalk()) continue; // crosswalks are conflicts that have already been configured.

                for (uint lj = 0; lj < _sections[j].size(); ++lj)
                {
                    if (_sections[j][lj]->isCrosswalk()) continue;

                    if ( (_sections[i][li]->isNextLane(_sections[j][lj])) || (_sections[i][li]->isPrevLane(_sections[j][lj])) ) continue;

                    if ( mvf::areCloseEnough( _sections[i][li]->getDestination(), _sections[j][lj]->getDestination(), lane::odrTol ) )
                        continue;

                    if ( mvf::areCloseEnough( _sections[i][li]->getOrigin(), _sections[j][lj]->getOrigin(), lane::odrTol ) )
                        continue;

                    if ( mvf::areCloseEnough( _sections[i][li]->getOrigin(), _sections[j][lj]->getDestination(), lane::odrTol ) )
                        continue;

                    if ( mvf::areCloseEnough( _sections[i][li]->getDestination(), _sections[j][lj]->getOrigin(), lane::odrTol ) )
                        continue;


                    // Get the Intersections:
                    //  Sometimes v_tmp will have pairs of very close points:
                    std::vector<arr2> v_tmp = _sections[i][li]->getIntersectionPoints(_sections[j][lj]);
                    //  so we filter them out in v:
                    std::vector<arr2> v;
                    scalar filterTol = 1e-5;
                    for (uint ck = 0; ck < v_tmp.size(); ++ck)
                    {
                        scalar dmin = 10;
                        for (uint cm = 0; cm < v.size(); ++cm)
                        {
                            scalar d = mvf::distance(v_tmp[ck], v[cm]);
                            if (d < dmin) dmin = d;
                        }
                        if (dmin > filterTol) v.push_back(v_tmp[ck]);
                    }

                    // Well, if there're no intersections, we're done.
                    if (v.size() == 0) continue;

                    // Print out the intersections:
                    if (verbose())
                    {
                        std::cout << "lanes: " << _sections[i][li]->getCSUID() << ", and " << _sections[j][lj]->getCSUID()
                                  << " intersect in " << v.size() << " point";
                        if (v.size() > 1) std::cout << "s; these are: ";
                        else std::cout << ", in: ";
                        std::cout << "(" << v[0][0] << ", " << v[0][1] << ")";
                        for (uint vk = 1; vk < v.size(); ++vk)
                        {
                            std::cout << ", (" << v[vk][0] << ", " << v[vk][1] << ")";
                        }
                        std::cout << " where the curvature of " << _sections[i][li]->getCSUID() << " and "
                                  << _sections[j][lj]->getCSUID() << " is: " << _sections[i][li]->getCurvature(v[0])
                                << " and " << _sections[j][lj]->getCurvature(v[0]);
                        for (uint vk = 1; vk < v.size(); ++vk)
                        {
                            std::cout << ", " << _sections[i][li]->getCurvature(v[vk])
                                      << " and " << _sections[j][lj]->getCurvature(v[vk]);
                        }
                        std::cout << " and where the angle they form is " <<
                                     mvf::subtendedAngle(_sections[i][li]->getTangentInPoint(v[0]),
                                _sections[j][lj]->getTangentInPoint(v[0])) * ct::rad2deg;
                        for (uint vk = 1; vk < v.size(); ++vk)
                        {
                            std::cout << ", " << mvf::subtendedAngle(_sections[i][li]->getTangentInPoint(v[vk]),
                                                                     _sections[j][lj]->getTangentInPoint(v[vk])) * ct::rad2deg;
                        }
                        std::cout << " degrees" << std::endl;
                    }


                    // And store them, so they're shown later by the graphicalLRN:
                    for (uint vk = 0; vk < v.size(); ++vk)
                        crossingPoints.push_back(v[vk]);

                    // Now for each intersection, there is a conflict:
                    for (uint ck = 0; ck < v.size(); ++ck)
                    {
                        scalar dToEoLi = _sections[i][li]->unsafeDistanceToTheEoL(v[ck]) + 0.1;
                        if (dToEoLi > _sections[i][li]->getLength()) dToEoLi = _sections[i][li]->getLength();
                        scalar dToEoLj = _sections[j][lj]->unsafeDistanceToTheEoL(v[ck]) + 0.1;
                        if (dToEoLj > _sections[j][lj]->getLength()) dToEoLj = _sections[j][lj]->getLength();
                        // In principle, starboard has priority.
                        lane *portLane, *starboardLane;
                        if (findPortAndStarboardLanes(portLane, starboardLane, _sections[i][li], _sections[j][lj], dToEoLi, dToEoLj) > 0)
                        {
                            std::cerr << "[ Error ] unable to find out which one is port between: " <<
                                         _sections[i][li]->getCSUID() << " and " << _sections[j][lj]->getCSUID() <<
                                         " at the intersection point (" << v[ck][0] << ", " << v[ck][1] << ")" << std::endl;
                            continue;
                        }
                        lane *lpLane, *hpLane;
                        if (_drivingSide == concepts::drivingSide::leftHand) // assing portLane to lower priority, for the moment:
                        {
                            lpLane = portLane;
                            hpLane = starboardLane;
                        }
                        else if (_drivingSide == concepts::drivingSide::rightHand)
                        {
                            lpLane = starboardLane;
                            hpLane = portLane;
                        }
                        else
                        {
                            std::cerr << "nonsense!" << std::endl;
                            continue;
                        }

                        // However, if the lanes come with an abs( angle ) significantly larger than 90 degrees AND
                        //      AND    there is only one that is straight, the straight lane has higher priority.
                        if (  (std::fabs( mvf::subtendedAngle(_sections[i][li]->getTangentInPoint(v[ck]),
                                                             _sections[j][lj]->getTangentInPoint(v[ck])) ) > 100 * ct::deg2rad) )
                        {
                            // if ( (std::fabs( lpLane->getCurvature(v[ck]) ) < 1e-3) && (std::fabs( hpLane->getCurvature(v[ck]) ) > 1e-3) )
                            if ( (std::fabs( lpLane->getCurvature(v[ck]) )) <  (std::fabs( hpLane->getCurvature(v[ck]) )) )
                            {
                                lane *tmp = lpLane;
                                lpLane = hpLane;
                                hpLane = tmp;
                            }
                        }


                        if (verbose())
                            std::cout << " Lane " << lpLane->getSUID() << " has lower priority than " << hpLane->getSUID()
                                      << " at the intersection point (" << v[ck][0] << ", " << v[ck][1] << ")" << std::endl;


                        // Add the conflict to the low priority lane:
                        lpLane->addConflict( conflict::createIntersectionConflict(v[ck], lpLane, hpLane, anticipationTime) );
                        //   and inform the higher priority lane:
                        conflict cnf = conflict::createIntersectionConflict(v[ck], hpLane, lpLane, anticipationTime);
                        cnf.k = conflict::kind::free;
                        hpLane->addConflict(cnf);
                    }
                }
            }
        }
    }
}

lane* RNS::getLane(const lane *l)
{
    if (!l) return nullptr;
    return _sections[l->getSectionID()][l->getID()];
}

void RNS::crosslinkConflict(lane *l, uint cndx, conflict::cuid ocuid)
{
    conflict::cuid tcuid = {l, l->getConflictSCoord(cndx)};
    if (!conflict::is1stLinkedTo2nd({ocuid.l, ocuid.s}, {tcuid.l, tcuid.s}))
        l->addConflictLink(cndx, ocuid);
        // l->_conflicts[cndx].links.push_back(ocuid);

    if (!conflict::is1stLinkedTo2nd({tcuid.l, tcuid.s}, {ocuid.l, ocuid.s}))
    {
        uint cuidj = ocuid.l->getConflictIdx(ocuid.s);
        getLane(ocuid.l)->addConflictLink(cuidj, tcuid);
    }


    std::vector<conflict::cuid> tlinks = l->getConflictLinks(cndx);
    for (uint t = 0; t < tlinks.size(); ++t)
    {
        if (!conflict::is1stLinkedTo2nd({ocuid.l, ocuid.s}, {tlinks[t].l, tlinks[t].s}))
        {
            uint ndx = tlinks[t].l->getConflictIdx(tlinks[t].s);
            getLane(tlinks[t].l)->addConflictLink(ndx, ocuid);
        }
        if (!conflict::is1stLinkedTo2nd({tlinks[t].l, tlinks[t].s}, {ocuid.l, ocuid.s}))
        {
            uint ndx = ocuid.l->getConflictIdx(ocuid.s);
            getLane(ocuid.l)->addConflictLink(ndx,tlinks[t]);
        }
    }

    std::vector<conflict::cuid> olinks = ocuid.l->getConflictLinks(ocuid.s);
    for (uint o = 0; o < olinks.size(); ++o)
    {
        if (!conflict::is1stLinkedTo2nd({tcuid.l, tcuid.s}, {olinks[o].l, olinks[o].s}))
        {
            uint ndx = olinks[o].l->getConflictIdx(olinks[o].s);
            getLane(olinks[o].l)->addConflictLink(ndx, tcuid);
        }
        if (!conflict::is1stLinkedTo2nd({olinks[o].l, olinks[o].s}, {tcuid.l, tcuid.s}))
        {
            uint ndx = tcuid.l->getConflictIdx(tcuid.s);
            getLane(tcuid.l)->addConflictLink(ndx,olinks[o]);
        }
    }
}

void RNS::crosslinkConflict(lane *l, scalar cSCoord, conflict::cuid cuid)
{
    int idx = l->getConflictIdx(cSCoord);
    if (idx < 0)
    {
        std::cerr << "unable to find a conflict in lane " << l->getSUID() << " at length " << cSCoord << std::endl;
        return;
    }
    return crosslinkConflict(l, static_cast<uint>(idx), cuid);
}


bool RNS::swapConflictPriority(lane *l, uint ci)
{
    bool swapped = false;
    conflict cnf = l->getConflict(ci);
    conflict::kind ko = cnf.k;
    for (uint lj = 0; lj < cnf.hpLane.size(); ++lj)
    {
        int ick = cnf.hpLane[lj]->getConflictIdx(l);
        if (ick < 0) continue;
        uint uck = static_cast<uint>(ick);
        l->setConflictKind(ci, cnf.hpLane[lj]->getConflictKind(uck));
        getLane(cnf.hpLane[lj])->setConflictKind(uck, ko);
        if (verbose())
        {
            std::cout << "swapped priorities and now: " << l->getSUID() << ":"
                      << conflict::kindString(l->getConflictKind(ci))
                      << " and " << cnf.hpLane[lj]->getSUID()
                      << ":" << conflict::kindString(cnf.hpLane[lj]->getConflictKind(uck)) << std::endl;
        }
        swapped = true;
    }
    return swapped;

}


bool RNS::swapConflictPriority(lane *l, scalar s)
{
    return swapConflictPriority(l, static_cast<uint>(l->getConflictIdx(s)));
}
