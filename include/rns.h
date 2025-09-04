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

#ifndef ODRONES_RNS_H
#define ODRONES_RNS_H

#include "rnsconcepts.h"
#include "section.h"

namespace odrones {

typedef odrones::concepts concepts;
typedef odrones::lane lane;
typedef odrones::section section;
typedef odrones::OneVersion OneVersion;
typedef odrones::mvf mvf;
typedef odrones::conflict conflict;

///! The Road Network System:
class RNS
{

public:
    RNS();
    RNS(std::string odrMap, const char* drivingSide, bool exhaustiveLinking, bool fineTune, bool loadSidewalk, bool verbose = false);
    RNS(const RNS &r); ///< copy construct
    RNS& operator=(RNS& r); ///< copy assign
    ~RNS();

    void clearMemory();
    void initialise();
    void assignInputRNSToThis(const RNS& r);

    void setSections(uint sectionsSize); ///< allocate and enumerate so many sections;

    uint sectionsSize() const; ///< get the amount of sections;
    uint lanesSize() const; ///< get the absolute number of lanes;

    section& sections(uint ndx) const; ///< const access the sections
    section& operator[](uint ndx); ///< access section ndx.

    lane* getLane(const lane *l); ///< access a lane for which you only have a const pointer.

    bool ready() const; /*! return true if _sections are ready */
    void ready(bool r); ///< manually set _ready to r.

    bool verbose() const { return _verbose; }
    void verbose(bool v) { _verbose = v; }

    scalar linkTolerance() const { return _linkTol; }
    void linkTolerance(scalar tol) { _linkTol = tol; }

    concepts::drivingSide drivingSide() const; ///< return the driving side.
    void drivingSide(concepts::drivingSide side); ///< manually set the driving side.

    bool makeRoads(std::string mapFile, const char* drivingSide, bool exhaustiveLinking, bool fineTune, bool loadSidewalk);
    bool makeOpenDRIVERoads(std::string mapFile, const char* drivingSide, bool exhaustiveLinking, bool loadSidewalk);
    bool makeOpenDRIVERoads(std::string mapFile, const char* drivingSide, bool exhaustiveLinking, bool fineTune, bool loadSidewalk);
    bool makeOpenDRIVERoads(ReadOdr &read, const char* drivingSide, bool exhaustiveLinking, bool fineTune, bool loadSidewalk);
    bool makeOneVersionRoads(std::string mapFile);
    void printLanes() const; ///< print sections and lanes
    void write(const std::string &mapFile) const;

    void setPortAndStarboard(concepts::drivingSide drivingSide); ///< sets Port and Starboard for every section AND flips two-way sections if drivingSide is known.
    void setPortAndStarboard(); ///< it uses _drivingSide

    void flipOneWaySections(); ///< flips one way sections after, using the knowledge gained from two-way sections in setPortAndStarboard.


    /*! establish the priorities, essentially through the methods below: */
    bool makePriorities(scalar anticipationTime);

    //! Given the point o, find the set of lane coordinates l, p (projected point), s, and loff (lateral offset)
    //!   that is not farther from o than tol. lCoord.l will be nullptr if nothing was found closer than tol.
    lane::lCoord getLaneCoordsForPoint(const arr2 &o, scalar tol) const;

    arr2 getPosForLaneCoords(const lane::lCoord &lc) const;
    arr2 getPosForRoadCoords(uint rID, scalar s, scalar offset, scalar height) const;

    //! get the size of whole map rounded "up".
    void getDimensions(int &minX, int &minY, int &maxX, int &maxY) const;
    void getDimensions(scalar &minX, scalar &minY, scalar &maxX, scalar &maxY) const;


    std::vector<lane::tSign> tSigns() const;
private:
    void tSigns(const std::vector<lane::tSign> &t);

    lane* getLaneWithSUID(int sID, int lID) const;
    lane* getLaneWithODRIds(uint rID, int lID) const;
    lane* getLaneWithOVId(const OneVersion::OVID &lID) const;
    section* getSectionWithOVId(const OneVersion::OVID &sID) const;

public:
    const lane* getCLaneWithSUID(uint sID, uint lID) const;
    const lane* getCLaneWithODRIds(uint rID, int lID) const;

    std::vector<uint> getSectionIDsWithOVRoadNodeId(const OneVersion::OVID &rnID) const; ///< returning a vector because a node may have a number of laneGroups, and rns store each one in a different section. That will be an empty vector if roadIDM or roadIDm are < 0
    std::vector<uint> getSectionIDsWithOVRoadNodeId(int rnMID, int rnmID) const; ///< returning a vector because a node may have a number of laneGroups, and rns store each one in a different section. That will be an empty vector if roadIDM or roadIDm are < 0
    std::vector<uint> getSectionIDsWithOVNodeId(int nID) const; ///< returning a vector because a node may have a number of laneGroups, and rns store each one in a different section. That will result in an empty vector if nID is < 0;

    const section* getSectionWithODRId(uint rID) const;
    int getSectionIDWithODRIDWithRoadCoord(uint rID, scalar s) const; ///< -1 if not found.


    // Order ids.
    std::vector<uint> buildBackwardsFromSecID(uint last, const std::vector<uint> &ids) const; /*! put the ids array in order so that the sections link and finish at "last" */
    std::vector<uint> buildForwardsFromSecID(uint first, const std::vector<uint> &ids) const; /*! put the ids array in order so that the sections link and finish at "last" */

    bool findLastAndFirstLinkingSectionIDs(int &last_o, int &first_e, const std::vector<uint> &ids_o, const std::vector<uint> &ids_e ) const; ///< defaulting to -1 and -1, given two arrays of section ids, ids_o and ids_e, find the two that link together the whole set; returning false in case of failure */
    bool findFirstLinkIDInSection(int &first, uint last, const std::vector<uint> &ids_o ) const; ///< defaulting to -1, given section id "last" and an array of section ids, ids_o, find the id of the section linking to last; returning false in case of failure*/


public:
    void crosslinkConflict(lane *l, uint cndx, conflict::cuid cuid); ///< crosslink conflict in l with index cndx with cuid, if they were not already.
    void crosslinkConflict(lane *l, scalar cSCoord, conflict::cuid cuid); ///< crosslink conflict in l at sCoord with cuid, if they were not already.
    bool swapConflictPriority(lane *l, uint i); ///< swap the priorities of conflicts lane l - conflict i and the (free) conflict at the other lane.
    bool swapConflictPriority(lane *l, scalar s); ///< overload


    void linkLanesGeometrically(scalar tol = lane::odrTol); ///< Go over all the section pairs, and link all the lanes if in range

private:
    //! Given two lanes l1 and l2, take a point on each one that is at a fraction (scalar between 0 and 1)
    //!    of their length. Knowing the direction of the lanes in this points,
    //!    return the lane that is on the Port side.
    int findPortAndStarboardLanes(lane* &port, lane* &starboard, lane* l1, lane* l2, scalar dToEoL1, scalar dToEoL2) const;

    //! Currently unused...
    const lane* getLaneWithPoint(const arr2 &p, scalar tol = mvf::absolutePrecision) const;

    //! Assign li as nextLane to lj or lj as nextLane to li, and set the corresponding prevLanes,
    //!  ... as long as any ending pair of points for li and lj are closer than tol.
    uint linkLanesIfInRange(lane *li, lane *lj, scalar tol = lane::odrTol);
    //!  ... as long as the end / start of li and lj are closer than tol.
    bool linkLanesIfInRangeAndOD(lane *li, lane *lj, scalar tol = lane::odrTol);
    //!  ... as long as any ending pair of points for li and lj are closer than tol and the tangents align correctly.
    uint linkLanesIfSound(lane *li, lane *lj, scalar tol = lane::odrTol);

    //! Assign nextLanes and prevLanes to the lanes in sections si and sj by calling linkLanesIfInRange on a double loop. Return true if anything was linked.
    bool linkLanesInSections(section &si, section &sj, scalar tol = lane::odrTol);
    bool linkLanesInSectionsOD(section &si, section &sj, scalar tol = lane::odrTol);
    bool linkLanesInSectionsIfSound(section &si, section &sj, scalar tol = lane::odrTol);

    /*! Check if any two edges of these two sections are close enough */
    bool sectionEdgesInRange(section &si, section &sj, scalar tol = lane::odrTol) const;

    /*! arrange conflicts and default priorities for lanes in different sections and same ending: priority is to the right */
    bool makePrioritiesSameEndingDifferentSectionLanes(scalar anticipationTime);
    /*! lanes within the same section with no next lane, or same ending will be marked as "merge" */
    void makePrioritiesSameSectionMergeLanes();
    /*! arrange conflicts and default priorities for lanes in different sections and with different endings that present intersections */
    void makePrioritiesDifferentEndingDifferentSectionCrossingLanes(scalar anticipationTime);


    /*! adjust Beziers to improve the exhaustive linkage */
    void fineTuneReadOdr(ReadOdr &read) const;




public:
    std::vector<arr2> crossingPoints; ///< DEBUG!!


private:
    section* _sections; ///< the list of sections that conform the rns
    uint _sectionsSize; ///< the amount of sections

    concepts::drivingSide _drivingSide; ///< driving side of the road.

    std::vector<lane::tSign> _tSigns;  ///< a convenience vector with an instance of every traffic sign.
    // std::vector<conflict::staticObj> _sObjects; ///< a convenience vector with

    ReadOdr _letter; ///< keep a copy of the ReadOdr that was used to configure the rns in case we need printing.
    bool _ready; ///< whether the RNS is ready or not.
    bool _verbose; ///< whether to print out to std::out or not.

    scalar _linkTol; ///< linking tolerance.

};

}

#endif // ODRONES_RNS_H
