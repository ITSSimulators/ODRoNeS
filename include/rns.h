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

#ifndef RNS_H
#define RNS_H

#include <filesystem>
#include "rnsconcepts.h"
#include "section.h"

///! The Road Network System:
class RNS
{

public:
    RNS();
    RNS(std::string odrMap, concepts::drivingSide drivingSide, bool loadSidewalk);
    RNS(const RNS &r); ///< copy construct
    RNS& operator=(RNS& r); ///< copy assign
    ~RNS();

    void clearMemory();
    void assignInputRNSToThis(const RNS& r);

    void setSections(uint sectionsSize); ///< allocate and enumerate so many sections;

    uint sectionsSize() const; ///< get the amount of sections;
    uint lanesSize() const; ///< get the absolute number of lanes;

    section& sections(uint ndx) const; ///< const access the sections
    section& operator[](uint ndx); ///< access section ndx.

    bool ready() const; /*! return true if _sections are ready */
    void ready(bool r); ///< manually set _ready to r.

    concepts::drivingSide drivingSide() const; ///< return the driving side.
    void drivingSide(concepts::drivingSide side); ///< manually set the driving side.

    bool makeRoads(std::string odrMap, concepts::drivingSide drivingSide, bool loadSidewalk);
    void printLanes() const; ///< print sections and lanes

    lane* getLaneWithSUID(int sID, int lID) const;
    lane* getLaneWithODRIds(uint rID, int lID) const;

    //! Given two lanes l1 and l2, take a point on each one that is at a fraction (scalar between 0 and 1)
    //!    of their length. Knowing the direction of the lanes in this points,
    //!    return the lane that is on the Port side.
    int findPortAndStarboardLanes(lane* &port, lane* &starboard, lane* l1, lane* l2, scalar dToEoL1, scalar dToEoL2) const;

    //! Given the point o, find the set of lane coordinates l, p (projected point) and loff (lateral offset)
    //!   that is not farther from o than tol. lCoord.l will be nullptr if nothing was found closer than tol.
    lane::lCoord getLaneCoordsForPoint(const arr2 &o, scalar tol) const;
    //! Currently unused...
    lane* getLaneWithPoint(const arr2 &p, scalar tol = mvf::absolutePrecision) const;

    std::vector<lane::tSign> tSigns() const;
    void tSigns(const std::vector<lane::tSign> &t);


    //! get the size of whole map rounded "up".
    void getDimensions(int &minX, int &minY, int &maxX, int &maxY) const;
    void getDimensions(scalar &minX, scalar &minY, scalar &maxX, scalar &maxY) const;

    /*! establish the priorities, essentially through the methods below: */
    bool makePriorities(scalar anticipationTime);
private:
    /*! arrange conflicts and default priorities for lanes in different sections and same ending: priority is to the right */
    bool makePrioritiesSameEndingDifferentSectionLanes(scalar anticipationTime);
    /*! lanes within the same section with no next lane, or same ending will be marked as "merge" */
    void makePrioritiesSameSectionMergeLanes();
    /*! arrange conflicts and default priorities for lanes in different sections and with different endings that present intersections */
    void makePrioritiesDifferentEndingDifferentSectionCrossingLanes(scalar anticipationTime);






public:
    std::vector<arr2> crossingPoints; ///< DEBUG!!


private:
    section* _sections; ///< the list of sections that conform the rns
    uint _sectionsSize; ///< the amount of sections

    concepts::drivingSide _drivingSide; ///< driving side of the road.

    std::vector<lane::tSign> _tSigns;  ///< a convenience vector with an instance of every traffic sign.
    // std::vector<conflict::staticObj> _sObjects; ///< a convenience vector with

    bool _ready; ///< whether the RNS is ready or not.


};


#endif // RNS_H
