//
//   This file is part of ODRoNeS (OpenDRIVE Road Network System).
//
//   Copyright (c) 2019-2026 Albert Solernou, University of Leeds.
//
//   The ODRoNeS package is free software; you can redistribute it and/or
//   modify it under the terms of the GNU Lesser General Public
//   License as published by the Free Software Foundation; either
//   version 3 of the License, or (at your option) any later version.
//
//   The ODRoNeS package is distributed in the hope that it will be useful,
//   but WITHOUT ANY WARRANTY; without even the implied warranty of
//   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
//   Lesser General Public License for more details.
//
//   You should have received a copy of the GNU Lesser General Public
//   License along with the ODRoNeS package; if not, see
//   <https://www.gnu.org/licenses/>.
//



#ifndef ODRONES_RNS_H
#define ODRONES_RNS_H

#include "rnsconcepts.h"
#include "section.h"
#include "lCoord.h"

namespace odrones {

typedef odrones::concepts concepts;
typedef odrones::lane lane;
typedef odrones::section section;
typedef odrones::OneVersion OneVersion;
typedef odrones::mvf mvf;
typedef odrones::conflict conflict;

class TrafficLightPhase
{
public:
    TrafficLightPhase(tSign::State s, double duration);

    void setState(tSign::State newState)
    {
        _state = newState;
    }

    tSign::State state() const
    {
        return _state;
    }

    void setDuration(double duration)
    {
        _duration = duration;
    }

    double duration() const
    {
        return _duration;
    }
private:
    tSign::State _state{ tSign::State::unknown };
    double _duration{ 0.0 };
};

class DynamicTrafficSignal
{
public:
    enum State : std::uint32_t
    {
        STATE_UNKNOWN,
        STATE_INITIAL,
        STATE_PHASE
    };
public:
    void setId(std::uint32_t id)
    {
        _id = id;
    }

    std::uint32_t id() const
    {
        return _id;
    }

    void setName(const std::string& name)
    {
        _name = name;
    }

    const std::string& name() const
    {
        return _name;
    }

    void setPhase(std::size_t phaseIndex, double simTime);

    void setLightState(tSign::State newState, double simTime);

    tSign::State state() const
    {
        return _lightState;
    }

    std::size_t numTSigns() const
    {
        return _tSigns.size();
    }

    void addTSign(tSign* sign)
    {
        _tSigns.emplace_back(sign);
    }

    tSign* sign(std::size_t index)
    {
        if (index < _tSigns.size())
            return _tSigns[index];

        return nullptr;
    }

    const tSign* sign(std::size_t index) const
    {
        if (index < _tSigns.size())
            return _tSigns[index];

        return nullptr;
    }

    void addPhase(tSign::State state, double duration);

    TrafficLightPhase* phase(std::size_t index)
    {
        if (index < _phases.size())
            return &_phases[index];

        return nullptr;
    }

    void step(double simTime);

    static std::string stateToString(State value);

    static State parseState(const std::string& str);
private:
    std::uint32_t _id{ ~0U };
    std::string _name;
    using TSignArray = std::vector<tSign*>;
    TSignArray _tSigns;
    using PhaseArray = std::vector<TrafficLightPhase>;
    PhaseArray _phases;
    tSign::State _lightState{ tSign::State::unknown };
    std::size_t _phaseIndex{ 0 };
    State _state{ STATE_INITIAL };
    double _stateChangeTime{ 0.0 };
    void setState(State newState, double simTime);
};

///! The Road Network System:
class RNS
{

public:
    RNS();
    RNS(std::string odrMap, const char* drivingSide, bool exhaustiveLinking, bool fineTune, bool loadSidewalk, bool verbose = false);
    RNS(const RNS &r); ///< copy construct
    RNS& operator=(const RNS& r); ///< copy assign
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

    std::size_t numDynamicTrafficSignals() const
    {
        return _trafficSignals.size();
    }

    DynamicTrafficSignal* dynamicTrafficSignal(std::uint32_t id);

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
    void write(const std::string &mapFile, bool beziers_as_pp3) const;

    void setPortAndStarboard(concepts::drivingSide drivingSide); ///< sets Port and Starboard for every section AND flips two-way sections if drivingSide is known.
    void setPortAndStarboard(); ///< it uses _drivingSide

    void flipOneWaySections(); ///< flips one way sections after, using the knowledge gained from two-way sections in setPortAndStarboard.


    /*! establish the priorities, essentially through the methods below: */
    bool makePriorities(scalar anticipationTime);

    //! Given the point o, find the set of lane coordinates l, p (projected point), s, and loff (lateral offset)
    //!   that is not farther from o than tol. lCoord.l will be nullptr if nothing was found closer than tol.
    lCoord getLaneCoordsForPoint(const arr2 &o, scalar tol) const;

    arr2 getPosForLaneCoords(const lCoord &lc) const;
    arr2 getPosForRoadCoords(uint rID, scalar s, scalar offset, scalar height) const;

    //! get the size of whole map rounded "up".
    void getDimensions(int &minX, int &minY, int &maxX, int &maxY) const;
    void getDimensions(scalar &minX, scalar &minY, scalar &maxX, scalar &maxY) const;

    // Traffic Signs:
    std::vector<tSign> tSigns() const; ///< return a copy of all the traffic signs

    void addDynamicTrafficSignal(DynamicTrafficSignal* signal);

    bool appendTSign(tSign ts, int orientation);  ///< add a traffic sign to every lane in ts.section that has the correct orientation.
private:
    lane* getLaneWithSUID(int sID, int lID) const;
    lane* getLaneWithOVId(const OneVersion::OVID &lID) const;
    section* getSectionWithOVId(const OneVersion::OVID &sID) const;

public:
    const lane* getCLaneWithSUID(uint sID, uint lID) const;
    const lane* getCLaneWithODRIds(uint rID, int lID) const;
    lane* getLaneWithODRIds(uint rID, int lID) const;

    std::vector<uint> getSectionIDsWithOVRoadNodeId(const OneVersion::OVID &rnID) const; ///< returning a vector because a node may have a number of laneGroups, and rns store each one in a different section. That will be an empty vector if roadIDM or roadIDm are < 0
    std::vector<uint> getSectionIDsWithOVRoadNodeId(int rnMID, int rnmID) const; ///< returning a vector because a node may have a number of laneGroups, and rns store each one in a different section. That will be an empty vector if roadIDM or roadIDm are < 0
    std::vector<uint> getSectionIDsWithOVNodeId(int nID) const; ///< returning a vector because a node may have a number of laneGroups, and rns store each one in a different section. That will result in an empty vector if nID is < 0;
    const section* getSectionWithOVRoadNodeId(int major, int minor) const;

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


    void linkLanesGeometrically(scalar tol, bool sameKind); ///< Go over all the section pairs, and link all the lanes if in range and of same kind.

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
    uint linkLanesIfSound(lane *li, lane *lj, scalar tol = lane::odrTol, bool sameKind = false);

    //! Assign nextLanes and prevLanes to the lanes in sections si and sj by calling linkLanesIfInRange on a double loop. Return true if anything was linked.
    bool linkLanesInSections(section &si, section &sj, scalar tol = lane::odrTol);
    bool linkLanesInSectionsOD(section &si, section &sj, scalar tol = lane::odrTol, bool sameKind = false);
    bool linkLanesInSectionsIfSound(section &si, section &sj, scalar tol = lane::odrTol, bool sameKind = false);

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

    ReadOdr _letter; ///< keep a copy of the ReadOdr that was used to configure the rns in case we need printing.
    bool _ready; ///< whether the RNS is ready or not.
    bool _verbose; ///< whether to print out to std::out or not.

    scalar _linkTol; ///< linking tolerance.
    std::vector<DynamicTrafficSignal*> _trafficSignals;
};

}

#endif // ODRONES_RNS_H
