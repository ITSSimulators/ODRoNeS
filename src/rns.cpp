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

RNS::RNS() :
    _sections(nullptr),
    _sectionsSize(0),
    _ready(false)
{
    return;
}


RNS::~RNS()
{
    clearMemory();
}

RNS& RNS::operator=(RNS& r)
{
    clearMemory();
    assignInputRNSToThis(r);
    return *this;
}

RNS::RNS(const RNS& r)
{
    assignInputRNSToThis(r);
}


RNS::RNS(std::string odrMap, concepts::drivingSide drivingSide, bool loadSidewalk)
{
    _ready = makeRoads(odrMap, drivingSide, loadSidewalk);
}


void RNS::clearMemory()
{
    if (_sectionsSize > 0)
    {
        delete[] _sections;
        _sections = nullptr;
        _sectionsSize = 0;
    }
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
            lane* wl = _sections[is][jl]->getPortLane();
            if (wl)
                _sections[is][jl]->setPortLane( getLaneWithSUID(wl->getID(), wl->getSectionID()) );

            // Starboard Lane:
            wl = _sections[is][jl]->getStarboardLane();
            if (wl)
                _sections[is][jl]->setStarboardLane( getLaneWithSUID(wl->getID(), wl->getSectionID()) );

            // Next Lane:
            if (_sections[is][jl]->hasNextLane())
            {
                lane** nls; size_t nlsSize;
                std::tie(nls, nlsSize) = _sections[is][jl]->getNextLanes();
                for (uint knl = 0; knl < nlsSize; ++knl)
                    _sections[is][jl]->setNextLane(knl, getLaneWithSUID(nls[knl]->getID(), nls[knl]->getSectionID()));
            }

            // Prev Lane:
            if (_sections[is][jl]->hasPrevLane())
            {
                lane** pls; size_t plsSize;
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

bool RNS::makeRoads(std::string odrMap, concepts::drivingSide drivingSide, bool loadSidewalk)
{
    _ready = false;

    _drivingSide = drivingSide;

    std::cout << "[ Warning ] work in progress; ignoring loadSidewalk value: " << loadSidewalk << std::endl;

    bool isOdrFile = false;
    if ((odrMap.length() < 255) && (std::filesystem::exists(odrMap)))
        isOdrFile = true;

    ReadOdr read(odrMap, isOdrFile);

    if (!read.isReady())
    {
        std::cerr << "[ LRN ] unable to load the opendrive map" << std::endl;
        return false;
    }

    // Allocate and do the geometry for the lanes:
    // We will allocate as many sections as laneSections:
    uint sectionsSize = 0;
    for (uint i = 0; i < read.sections.size(); ++i)
        sectionsSize += read.sections[i].lsSize;
    setSections(static_cast<uint>(sectionsSize));

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
            std::cout << "PANIC!" << std::endl;
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
    for (uint i = 0; i < read.sections.size(); ++i)
    {
        for (uint j = 0; j < read.sections[i].lanes.size(); ++j)
        {
            lane* lij = getLaneWithODRIds(read.sections[i].odrID, read.sections[i].lanes[j].odrID);
            // std::cout << "lane: " << lij->getCSUID() << std::endl;
            if (!lij)
               std::cout << "[ Error ] No lane for: " << i << ", " << j << ", segfault on the way... " << std::endl;
            for (uint k = 0; k < read.sections[i].lanes[j].nextLane.size(); ++k)
            {
                Odr::smaL *sml = read.sections[i].lanes[j].nextLane[k];
                if (!sml) continue;

                lane* nextLane = getLaneWithODRIds(read.sections[sml->sID].odrID, sml->odrID);

                if ((mvf::areCloseEnough(lij->getDestination(), nextLane->getOrigin(), lane::odrTol)) ||
                    (mvf::areCloseEnough(lij->getDestination(), nextLane->getDestination(), lane::odrTol)))
                {
                    lij->setNextLane(nextLane, false);
                    // std::cout << "assigning " << nextLane->getCSUID() << " as nextlane to " << lij->getCSUID() << std::endl;
                }

                if ((mvf::areCloseEnough(lij->getOrigin(), nextLane->getOrigin(), lane::odrTol)) ||
                    (mvf::areCloseEnough(lij->getOrigin(), nextLane->getDestination(), lane::odrTol)))
                {
                    lij->setPrevLane(nextLane, false);
                    // std::cout << "assigning " << nextLane->getCSUID() << " as prevlane to " << lij->getCSUID() << std::endl;
                }

                if ((mvf::areCloseEnough(nextLane->getDestination(), lij->getOrigin(), lane::odrTol)) ||
                    (mvf::areCloseEnough(nextLane->getDestination(), lij->getDestination(), lane::odrTol)))
                    nextLane->setNextLane(lij, false);

                if ((mvf::areCloseEnough(nextLane->getOrigin(), lij->getOrigin(), lane::odrTol)) ||
                    (mvf::areCloseEnough(nextLane->getOrigin(), lij->getDestination(), lane::odrTol)))
                    nextLane->setPrevLane(lij, false);
            }

            for (uint k = 0; k < read.sections[i].lanes[j].prevLane.size(); ++k)
            {
                Odr::smaL *sml = read.sections[i].lanes[j].prevLane[k];
                if (!sml) continue;

                lane* prevLane = getLaneWithODRIds(read.sections[sml->sID].odrID, sml->odrID);

                if ((mvf::areCloseEnough(lij->getDestination(), prevLane->getOrigin(), lane::odrTol)) ||
                    (mvf::areCloseEnough(lij->getDestination(), prevLane->getDestination(), lane::odrTol)))
                {
                    lij->setNextLane(prevLane, false);
                    // std::cout << "assigning " << prevLane->getCSUID() << " as nextlane to " << lij->getCSUID() << std::endl;
                }

                if ((mvf::areCloseEnough(lij->getOrigin(), prevLane->getOrigin(), lane::odrTol)) ||
                    (mvf::areCloseEnough(lij->getOrigin(), prevLane->getDestination(), lane::odrTol)))
                {
                    lij->setPrevLane(prevLane, false);
                    // std::cout << "assigning " << prevLane->getCSUID() << " as prevlane to " << lij->getCSUID() << std::endl;
                }

                if ((mvf::areCloseEnough(prevLane->getDestination(), lij->getOrigin(), lane::odrTol)) ||
                    (mvf::areCloseEnough(prevLane->getDestination(), lij->getDestination(), lane::odrTol)))
                    prevLane->setNextLane(lij, false);

                if ((mvf::areCloseEnough(prevLane->getOrigin(), lij->getOrigin(), lane::odrTol)) ||
                    (mvf::areCloseEnough(prevLane->getOrigin(), lij->getDestination(), lane::odrTol)))
                    prevLane->setPrevLane(lij, false);

            }
        }
    }


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

lane* RNS::getLaneWithODRIds(uint rID, int lID) const
{
    for (uint i = 0; i < _sectionsSize; ++i)
    {
        if (_sections[i].getOdrID() != rID) continue;
        for (uint j = 0; j < _sections[i].size(); ++j)
        {
            if (_sections[i][j]->getOdrID() == lID) return _sections[i][j];
        }
    }

    return nullptr;
}




int RNS::findPortAndStarboardLanes(lane* &port, lane* &starboard, lane *l1, lane *l2, scalar dToEoL1, scalar dToEoL2) const
{
    port = nullptr;
    if (dToEoL1 > l1->getLength())
    {
        std::cout << "[ ERROR ] getPortLane cannot be used with a dToEoL larger than the length of " << l1->getSUID() << std::endl;
        return 2;
    }
    else if (dToEoL2 > l2->getLength())
    {
        std::cout << "[ ERROR ] getPortLane cannot be used with a dToEoL larger than the length of " << l2->getSUID() << std::endl;
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
    lane::lCoord lcoo = {nullptr, {0, 0}, 1e3};
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

            if (!_sections[is][jl]->projectPointOntoLane(pj, o, false)) continue;

            scalar oj = mvf::distance(pj, o);
            if (oj > tol) continue;
            if (oj < lcoo.loff)
            {
                lcoo.loff = oj;
                lcoo.l = _sections[is][jl];
                lcoo.pos = pj;
            }
        }
    }

    return lcoo;
}


lane* RNS::getLaneWithPoint(const arr2 &p, scalar tol) const
{
    lane::lCoord lcoo = getLaneCoordsForPoint(p, tol);
    if (lcoo.loff < tol) return lcoo.l;
    else return nullptr;
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
                std::cout << "[ ERROR ] lrn could not get the bounding box for lane "
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
                        std::cout << "nonsense!" << std::endl;
                        return false; // continue;
                    }

                    // create a conflict and assign it to the lower priority lane:
                    conflict lpCnf = conflict::createEoLConflict(lpLane, hpLane, anticipationTime);
                    for (uint pk = 0; pk < lpCnf.hpLane.size(); ++pk)
                        std::cout << "lane: " << lpCnf.hpLane[pk]->getSUID() << " has priority over " << lpLane->getSUID() << std::endl;
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
                if ((_drivingSide == concepts::drivingSide::leftHand) && ((_sections[i][li]->getStarboardLane()) && (_sections[i][li]->getStarboardLane()->hasNextLane())) )
                    _sections[i][li]->addConflict(conflict::createMergeConflict(_sections[i][li], mvf::side::starboard));

                else if ((_sections[i][li]->getPortLane()) && (_sections[i][li]->getPortLane()->hasNextLane()))
                    _sections[i][li]->addConflict(conflict::createMergeConflict(_sections[i][li], mvf::side::port));

                else if ((_drivingSide == concepts::drivingSide::rightHand) && ((_sections[i][li]->getStarboardLane()) && (_sections[i][li]->getStarboardLane()->hasNextLane())) )
                    _sections[i][li]->addConflict(conflict::createMergeConflict(_sections[i][li], mvf::side::starboard));


                continue; // if neither port nor starboard have next lanes, forget it.
            }


            // Here, lanes within the same section and same destination:
            for (uint lj = li + 1; lj < _sections[i].size(); ++lj )
            {
                if (_sections[i][lj]->isToMerge()) continue;

                if (!mvf::areCloseEnough( _sections[i][li]->getDestination(), _sections[i][lj]->getDestination(), lane::odrTol ) )
                    continue;

                if ( ((_drivingSide == concepts::drivingSide::rightHand) && (_sections[i][li]->getPortLane() == _sections[i][lj])) ||
                     ((_drivingSide == concepts::drivingSide::leftHand) && (_sections[i][li]->getStarboardLane() == _sections[i][lj])) )
                {
                    std::cout << "[ lrn ] assigning merge to " << _sections[i][li]->getSUID()
                              << " giving higher priority to " << _sections[i][lj]->getSUID() << std::endl;
                    _sections[i][li]->addConflict(conflict::createMergeConflict(_sections[i][li], mvf::side::port));
                    // _sections[i][lj]->setStopMargin(calcStopMargin(_sections[i][lj], _sections[i][li]));
                }

                else if ( ((_drivingSide == concepts::drivingSide::rightHand) && (_sections[i][li]->getStarboardLane() == _sections[i][lj])) ||
                          ((_drivingSide == concepts::drivingSide::leftHand) && (_sections[i][li]->getPortLane() == _sections[i][lj])) )
                {
                    std::cout << "[ lrn ] assigning merge to " << _sections[i][lj]->getSUID()
                              << " giving higher priority to " << _sections[i][li]->getSUID() << std::endl;
                    _sections[i][li]->addConflict(conflict::createMergeConflict(_sections[i][li], mvf::side::starboard));
                    // _sections[i][li]->setStopMargin(calcStopMargin(_sections[i][li], _sections[i][lj]));
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
                            std::cout << "nonsense!" << std::endl;
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


                        std::cout << " Lane " << lpLane->getSUID() << " has lower priority than " << hpLane->getSUID() <<
                                         " at the intersection point (" << v[ck][0] << ", " << v[ck][1] << ")" << std::endl;


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

