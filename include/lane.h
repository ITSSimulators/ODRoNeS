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

#ifndef ODRONES_LANE_H
#define ODRONES_LANE_H

#include <vector>
#include <tuple>
#include <vector>
#include "matvec.h"
#include "geometry.h"
#include "straight.h"
#include "vwStraight.h"
#include "arc.h"
#include "vwArc.h"
#include "bezier2.h"
#include "bezier3.h"
#include "paramPoly3.h"
#include "vwParamPoly3.h"
#include "vwNumerical.h"
#include "vwSpiral.h"
#include "readOdr.h"
#include "readOneVersion.h"
#include "rnsconcepts.h"

#ifdef QT_CORE_LIB
#include <QPainterPath>
#endif

namespace odrones {

class section;
class lane;
class conflict;

typedef odrones::scalar scalar;
typedef odrones::arr2 arr2;
typedef odrones::mvf mvf;
typedef odrones::Odr Odr;
typedef odrones::bezier2 bezier2;
typedef odrones::bezier3 bezier3;
typedef odrones::conflict conflict;
typedef odrones::OneVersion OneVersion;
typedef odrones::lane lane;

class conflict
{
public:
    struct cuid {const lane *l; scalar s;};
    enum class kind { free, giveWay, stop, mergeStarboard, mergePort, zebraWalk, unknown };
    static std::string kindString(kind k);

    // opendrive static objects:
    enum class staticObjKind {crosswalk, unknown};
    // static std::string staticObjKindString(staticObjKind s);
    struct staticObj
    {
        arr2 pos; ///< the position of the object
        arr2 tg; ///< orientation, currently used only on the graphical side.
        enum staticObjKind kind; ///< type of object
        scalar w, l; ///< width (along the road) and length (perpendicular to the road) of the object.
        scalar s, t; ///< odr lane coordinates of the object
        int section;
        int lane;
    };


public:
    arr2 pos; ///< the point on the lane at which the conflict (crosswalk) crosses the lane
    scalar s; ///< s coordinate (distance from the begining of the road);
    scalar so, se; ///< s coordinates at which we should stop and at which we're safe;
    std::vector<const lane*> hpLane; ///< lanes that have higher priority... (or that are dangerous!)
    std::vector<cuid> links; ///< a vector of conflicts that need to be solved at the same time.
    kind k; ///< the type of conflict, same as ending.

    static bool areSameConflicts(const cuid &i, const cuid &j);

    static bool isMerge(conflict::kind k);

    static bool is1stLinkedTo2nd(const cuid &ci, const cuid &cj); ///< true if ((ci == cj) || (ci is a link in cj))

    /*! signed distance, will return j - i */
    static scalar distance(const cuid &i, const cuid &j);

    //! Given lanes with same destination l1 and l2, calculate the stop margin for lane l1
    static scalar calcStopMargin(const lane* l1, const lane* l2);

    //! Create a conflict that is at the end of portLane, given the higher priority lane hpLane,
    //!    and a certain amount of anticipation time:
    static conflict createEoLConflict(const lane* portLane, lane* hpLane, scalar anticipationTime);

    //! Create a merge conflict at the end of the lane.
    static conflict createMergeConflict(const lane* accLane, mvf::side mergeSide);

    //! Create a conflict on arr2 pos, lane lpLane, given a higher priority lane and an ideal anticipation time:
    static conflict createIntersectionConflict(const arr2 &pos, const lane* lpLane, lane* hpLane, scalar anticipationTime);
    //!   overload using the scoord of lane lpLane instead of pos.
    // static conflict createIntersectionConflict(scalar sCoord, const lane* lpLane, lane* hpLane, scalar anticipationTime);


    //! Fill in the lpLane conflict with lanes up to some anticipation tine,
    //!   or back until the first branch. Return the actual anticipation time.
    static scalar fillInHPLanes(conflict &cnf, const lane* hpLane, scalar anticipationTime);

};



class lane : public numerical
{
public:

    // SUBCLASSES //
    // 1 - Enums
    enum class sign {n, o, p};
    static std::string signString(sign s);
    enum class kind { tarmac, pavement, roundabout, crosswalk, none, unknown };


    // 2 - Traffic signs:
    enum class tSignInfo {giveWay, stop, unknown};
    static std::string tSignInfoString(tSignInfo s);
    class tSign
    {
    public:
        tSign()
        { pos = {0., 0.}; lpos = {0., 0.}; s = 0; info = tSignInfo::unknown; mDir = 0; section = 0; lane = 0; assigned = false;}
        tSign(arr2 ipos, arr2 ilpos, scalar is, tSignInfo iinfo, int imDir, int isection, int ilane, bool iassigned)
        { pos = ipos; lpos = ilpos; s = is; info = iinfo; mDir = imDir; section = isection; lane = ilane; assigned = iassigned;}
    public:
        arr2 pos; ///< the actual position of the sign
        arr2 lpos; ///< the projected position on the lane (in lane)
        scalar s; ///< the distance from the beginning of the lane, calculated with the projected position (in lane)
        tSignInfo info; ///< the meaning of the traffic sign.
        int mDir; ///< either +1 or -1 depending on the orientation of the sign.
        int section; ///< the section that holds the sign (useful in lrn)
        int lane; ///< the lane that holds the sign (useful in lrn)
        bool assigned; ///< whether the sign has been assigned successfuly to a lane or not (useful in lrn).

    };


    // 3 - Logical coordinates.
    struct lCoord
    {
        const lane *l;    ///< the lane it's on.
        arr2 pos;         ///< position projected onto the center of the lane
        scalar s;         ///< distance down the lane
        scalar loff;      ///< lateral offset, positive to the right in the direction of the lane (starboard).
    };

    static constexpr scalar odrTol = 1e-2;


public:
    /** will construct the lane from origin to dest, with width width,
     * ending condition end, going from orig to dest with some laneShape, and given some ID */
    lane();
    lane(const lane& t);
    lane& operator=(const lane& t);
    ~lane() override;
    lane(const arr2 &origin, const arr2 &dest, scalar width, scalar speed, mvf::shape shp, sign sgn, bool permanent = true);
    lane(const std::vector<bezier2> &bzr, scalar width, scalar speed, sign sgn, bool permanent = true);
    lane(const std::vector<bezier3> &bzr, scalar width, scalar speed, sign sgn, bool permanent = true);
    lane(const std::vector<arr2> &bzrp, mvf::shape s, scalar width, scalar speed, sign sgn, bool permanent = true);
    void assignInputLaneToThis(const lane& t);

    // Configuring the lane:
    void base();
    void initialise(scalar width, scalar speed, mvf::shape shp, sign sgn, bool permanent = true);
    void set(const arr2 &origin, const arr2 &dest, scalar width, scalar speed, mvf::shape shp, sign sgn, bool permanent = true);
    void set(const arr2 &origin, const arr2 &dest, const arr2 &centre, scalar width, scalar speed, mvf::shape shp, sign sgn, bool permanent = true);
    void set(const std::vector<bezier2> &bzr, scalar width, scalar speed, sign sgn, bool permanent = true);
    void set(const std::vector<bezier3> &bzr, scalar width, scalar speed, sign sgn, bool permanent = true);
    void set(const std::vector<arr2> &bzrp, mvf::shape s, scalar width, scalar speed, sign sgn, bool permanent = true);
    void set(const std::vector<Odr::geometry> &odrg, std::vector<Odr::offset> off, const std::vector<Odr::offset> &width, const Odr::smaL &odrL, scalar se);
    void set(const OneVersion::smaS &sec, uint index);
    void setBezierLines(const std::vector<bezier2> &bzr);
    void setBezierLines(const std::vector<bezier3> &bzr);
    void appendBezierLines(const std::vector<bezier2> &bzr);
    void clearMemory();
    bool isSet() const; ///< return whether the lane is ready or not.

    void setNextLane(const lane *l); ///< setNextLane and don't crosslink.
    void setNextLane(lane *l, bool crosslink); ///< set the following lane in the Logical Road Network
    void setNextLane(uint ndx, lane *l); ///< set the ndxth next lane in the _nextLanes array with minimal checks;
    void setPrevLane(const lane *l); ///< setPrevLane and don't crosslink.
    void setPrevLane(lane *l, bool crosslink); ///< set the previous lane in the Logical Road Network
    void setPrevLane(uint ndx, lane *l); ///< set the ndxth prev lane in the _prevLanes array with minimal checks;
    void setPortLane(const lane *l); ///< set the Port (left) lane.
    void setStarboardLane(const lane *l); ///< set the Starboard (right) lane.
    void setSection(section &s); ///< set the section where this lane sits on.

    // Getting lanes
    const lane* getNextLane() const; ///< returns a pointer to the first next lane
    const lane* getNextLane(uint idx) const; ///< returns a pointer to the ith next lane.
    std::tuple<const lane**, size_t> getNextLanes() const; ///< returns the tuple <_nextLane, _nextLaneSize>
    size_t getNextLaneSize() const; ///< returns the number lanes that start at the end of this one.

    const lane* getPrevLane() const; ///< returns a pointer to the previous lane
    const lane* getPrevLane(uint idx) const; ///< returns a pointer to the ith next lane.
    std::tuple<const lane**, size_t> getPrevLanes() const; ///< returns the tuple <_prevLane, _prevLaneSize>
    size_t getPrevLaneSize() const; ///< returns the number lanes that end at the beginning of this one.

    const lane* getPortLane() const; ///< returns a pointer to the Port (left) lane
    const lane* getStarboardLane() const; ///< returns a pointer to the Starboard (right) lane.
    mvf::side getMergeSide() const; ///< returns the side towards this is merging.
    const lane* getMergeLane() const; ///< returns port, starboard, or null depending on the _mergeSide;

    // And the section:
    section* getSection() const;


    // Get the basics:
    void getOrigin(arr2 &o) const;
    arr2 getOrigin() const;
    void getDestination(arr2 &d) const;
    arr2 getDestination() const;
    arr2 getTheClosestExtreme(const arr2 &p) const;
    void getTo(arr2 &to) const;
    arr2 getTo() const;
    scalar getCurvature(const arr2 &p) const; ///< returns the curvature (1/R) at one point.
    scalar getLength() const; ///< returns the length of the lane.
    uint getGeometrySize() const; ///< get the size of the _geom array.
    // std::vector<std::unique_ptr<odrones::geometry>> getGeometries() const;
    scalar maxSo() const; ///< odr; return the max So coordinate of lane 0.

    scalar getWidth() const;
    scalar getWidth(scalar d) const; ///< get the width at a certain distance down the lane.

    scalar getSpeed() const;
    void setSpeed(const scalar speed);


    // Shape...
    mvf::shape getShape(uint i) const; ///< returns the shape of _geom[i]
    std::string getShapeString(uint i) const; ///< returns a string version of the shape of the lane.
    std::string getShapeString() const; ///< returns either _geom[0], or _shape
    std::string getShapesString() const; ///< returns a string with all the shapes of the lane in the _geom vector.
    bool isOpenDrive() const;
    bool isOneVersion() const;
    bool isArc() const;
    bool isArc(mvf::shape s) const;

    // ... and kind:
    void setKind(kind k);
    kind getKind() const;
    bool isRoundabout() const;
    bool isCrosswalk() const;
    bool isPavement() const;
    bool isTransitable() const;
    bool isToMerge() const;
    bool isInOdrRange(scalar s) const;
    bool actorsSupport(lane::kind k) const;
    bool actorsSupport(odrones::concepts::actor k) const;
    // bool actorsOverlap(const lane *l) const;


    // Traffic Signs:
    void addTSign(tSign ts);
    std::vector<tSign> getTSigns() const;
    uint tSignsSize() const;
    bool hasTSigns() const;
    // tSign getTSign(uint i) const; ///< return a copy of the ith traffic sign.
    tSignInfo getTSignInfo(uint i) const; ///< return the sign info of the ith sign.
    scalar getTSignSCoord(uint i) const; ///< return the distance from the begining of the lane of the ith traffic sign.
    void addStaticObj(conflict::staticObj so);
    // std::vector<sObject> getSObjects() const;


    // Conflicts:
    void addConflict(const conflict &cf); ///< store that conflict in order
    bool addConflictLane(uint i, lane *l); ///< add the lane l to conflict[i].hpLane
    bool addConflictLane(scalar s, lane *l); ///< add lane l to conflict hpLane at coord s.
    // Quering conflict 1:
    bool hasConflicts() const;  ///< whether it has any conflicts or none.
    uint conflictsSize() const; ///< returns the amount of conflicts that this lane has set.
    bool conflictIsCrosswalk(uint i) const; ///< true if conflict type == crosswalk.
    bool conflictIsMerge(uint i) const; ///< true if conflict i is merge.
    // bool conflictIsEoL(uint i) const; ///< return true if s == _length.
    // Conflict getters:
    conflict getConflict(uint i) const; ///< returns the ith conflict;
    conflict getConflict(scalar s) const; ///< returns the conflict at s;
    std::vector<conflict> getConflicts() const; ///< return the whole array of conflicts.
    arr2 getConflictPos(uint i) const; ///< returns the position of the ith conflict.
    arr2 getConflictPos(scalar s) const; ///< returns the position of the conflict that is on s.
    scalar getConflictLength(uint i) const; ///< returns the width of the ith crosswalk;
    scalar getConflictLength(scalar s) const; ///< overload
    std::vector<const lane*> getConflictLanes(uint i) const; ///< return the lanes of the ith conflict.
    std::vector<const lane*> getConflictLanes(scalar s) const; ///< overload
    scalar getConflictSCoord(uint i) const; ///< return s, the distance from the begining of the lane for conflict i.
    int getConflictIdx(const lane *l) const; ///< returns the index of the crosswalk that has lane l; -1 if not found
    int getConflictIdx(scalar s) const; ///< returns the index of the crosswalk at sCoord s; -1 if not found.
    std::vector<conflict::cuid> getConflictLinks(uint i) const; ///< return a copy of the _conflicts[i].link vector;
    std::vector<conflict::cuid> getConflictLinks(scalar s) const; ///< return a copy of the _conflicts[ idx(s) ].link vector;
    void addConflictLink(uint i, conflict::cuid id);
    // uint getConflictLinksSize(uint i) const; ///< return the amount of conflicts linked to the ith conflict.
    conflict::kind getConflictKind(uint i) const; ///< return the kind of conflict that this one is
    conflict::kind getConflictKind(scalar s) const; ///< return the kind of conflict that has conflict at s.
    void setConflictKind(uint i, conflict::kind k); ///< set the kind of conflict to the ith conflict.
    void setConflictKind(scalar s, conflict::kind k); ///< set the kind of the conflict at s.
    void setConflictHPLanes(uint i, lane* hpLane, scalar anticipationTime); ///< clear the HPLanes, and fill the vector calling conflict::fillInHPLanes.
    void setConflictHPLanes(scalar s, lane* hpLane, scalar anticipationTime); ///< clear the HPLanes, and fill the vector calling conflict::fillInHPLanes.
    // Conflict - crosswalks:
    bool addCrosswalk(const conflict::staticObj &so);
    uint crosswalksSize() const;


    // Comparing lanes:
    bool isSameLane(const lane *l) const; ///< true if l has the same _id and _sectionID
    bool isNextLane(const lane *l) const; ///< true if *l != nullptr AND *l is within _nextLine;
    bool isPrevLane(const lane *l) const; ///< true if *l == this._prevLane
    bool isConnected(const lane *l) const; ///< true if *l is not null && is (this, next or prev);
    bool isConnectedButNotThis(const lane *l) const; ///< true if *l is not null and is next or prev;
    bool hasNextLane() const; ///< true if _nextLane != nullptr.
    bool hasMultipleNextLanes() const; ///< true if _nextLaneSize > 1;
    bool hasPrevLane() const; ///< true if _prevLane != nullptr.
    bool hasMultiplePrevLanes() const; ///< true if _prevLaneSize > 1;


    // Geometry in lanes:
    void getTangentInPoint(arr2 &t, const arr2 &p) const;
    arr2 getTangentInPoint(const arr2 &p) const;
    //! returns the road angle in degrees, Qt style: -90 means Eastwards, 0 means Southwards.
    scalar getQtHeadingInPoint(const arr2 &p) const;
    //! returns whether p is on this lane or not...
    //!    within a lateral tolerance of tol... check!
    bool isPointOnLane(const arr2 &p, scalar tol = 1e-3) const;
    //! returns a point that is d metres after o, and true if it is on this same lane.
    bool getPointAfterDistance(arr2& p, const arr2 &o, scalar d) const;
    //! return a point that is at distance s from the origin:
    bool getPointAtDistance(arr2 &p, scalar s) const;
    //! returns the distance from the point until the end of the lane without checking whether p is on the lane
    //! It is unsafe in the sense that it won't check whether the point p belongs to the lane or not.
    scalar unsafeDistanceToTheEoL(const arr2 &p) const;
    scalar unsafeDistanceFromTheBoL(const arr2 &p) const;
    //! return a point p, given a point on the lane o, and a lateral offset loff (positive is starboard, negative is port)
    void getPointWithOffset(arr2& p, const arr2 &o, scalar loff) const;
    //! return a point p, given a distance down the lane, and a lateral offset loff (positive is starboard, negative is port)
    void getPointWithOffset(arr2& p, scalar d, scalar loff) const;
    //! return a vector with all the intersection points between "this" and lane l;
    //!   we'll have intersection points on every type of lane, just not today!
    std::vector<arr2> getIntersectionPoints(const lane *l) const;

    //! returns true and sets the destination intersection point between arr2 origin and arr2 tangent if there is some intersection,
    //!   and false otherwise.
    bool getIntersectionPointFromOT(arr2 &p, const arr2 &o, const arr2 &t) const;
    // static bool getIntersectionPointFromOTtoLane(arr2 &p, arr2 const &o, arr2 &t, const lane* l);
    //! Project point onto lane
    bool projectPointOntoLane(arr2 &p, arr2 const &o) const;
    arr2 projectPointOntoLane(const arr2 &o) const;

    //! return the botton left corner and the top right corner, and 0 if things went well;
    bool getBoundingBox(arr2 &bl, arr2 &tr) const;
    //! calculate the bounding box (without considering lane width) and store it internally into the pair _bbblc, and _bbtrc:
    bool calcBoundingBox();


    //! Lane IDs
    int getID() const;
    int odrID() const;
    OneVersion::OVID ovID() const;
    void setOVID(OneVersion::OVID id);
    void setID(int id);
    void setSectionID(int id);
    void setOdrSectionID(int id);
    static std::string sUID(int sID, int lID);
    std::string getSUID() const; ///< String Unique ID
    std::string getOdrSUID() const; ///< Odr String Unique ID
    std::string getOVSUID() const; ///< OneVersion String ID
    std::string getCSUID() const; ///< Complete SUID: "<SUID> (<OdrSUID>)"
    bool isSUID(std::string name) const;
    bool isCSUID(std::string name) const;
    bool isOdrSUID(std::string name) const;
    int getSectionID() const;


    bool isPermanent() const;

    bool hasDefinedSign() const; // true if _sign != sign::o
    sign getSign() const;
    std::string getSignString() const;
    int getSignInt() const; ///< return either 1, -1, or 0.
    bool isSameSign(sign s) const; // false if _sign == o -> true if _sign == s;
    bool isSameSign(const lane* l) const; // overload.
    static lane::sign invertSign(sign s);


    //! OpenDRIVE auxilliary functions:
    bool flipBackwards(); ///< flip the lane if isFwd != _odrFwd, and the lane is _flippable.
    void lockFlippable(); ///< set _flippable to false;
    bool isFlippable() const; ///< return _flippable
    bool isOdrFwd() const; ///< return _odrFwd
    bool isOdrShapeSupported(mvf::shape s) const; ///< return true if the shape is supported;


private:
    //! Find the correct geometry:
    int getGeometryIndex(const arr2 &p) const;  ///< get the geometry index for this point; return -1 if the point is not there for some tol.
    int getGeometryIndex(scalar d) const; ///< get the geometry index for this point; return -1 if out of bounds;


public:
#ifdef QT_CORE_LIB
    QPainterPath getEdgeQPainterPath(uint n, int e); ///< e = -1 for left edge and e = 1 for right edge.
    QPainterPath getQPainterPath(uint n) const; ///< get a QPainterPath with n points per Bezier line.
    std::vector<QPainterPath> getQPainterPaths(uint n) const; ///< get a vector with the QPainterPaths of each Bezier line in the lane.
    int fillInVerticesAndIndices(scalar step, std::vector<QByteArray> &indexBytes, std::vector<QByteArray> &vertexBytes,
                                 std::vector<int> &indexSize, std::vector<int> &vertexSize) const; ///< as the method states: allocate and fill in the required 3D data.
#endif

private:
    //! fill in the _pointsX/Y arrays
    void nSetupPointsXYUniformly(scalar ds) override;


private:
    scalar _width;
    int _id;
    sign _sign; ///< lanes within the same section that have the same sign have the same direction (positive, o, or negative).
    scalar _length; ///< length of the lane;
    arr2 _bbblc, _bbtrc; ///< bounding box bottom-left corner, bounding box top-right corner.

    scalar _speed; ///< the max speed at which one should drive.
    kind _kind; ///< whether it is tarmac or pavement.

    const lane** _nextLane; ///< next lane array
    uint _nextLaneSize; ///< number of next lanes that this one is linked to.
    const lane** _prevLane; ///< previous lane array
    uint _prevLaneSize; ///< number of previous lanes that this one is linked to.
    const lane* _portLane; ///< lane on the port side.
    const lane* _starboardLane; ///< lane on the starboard side.

    section *_section;
    int _sectionID; ///< literally, the ID of the section where the lane belongs to.

    std::vector<tSign> _tSigns; ///< vector of traffic signs.
    std::vector<conflict> _conflicts; ///< vector of conflicts.

    mvf::shape _shape; ///< shape of the lane

    std::vector<odrones::geometry*> _geom; ///< composed geometry for OpenDRIVE lanes.

    bool _isPermanent; ///< whether this lane is permanent or is a lcPath.

    int _odrID; ///< the OpenDRIVE id of the lane.
    int _odrSectionID; ///< the OpenDRIVE ID of the section
    /*! whether the lane goes forward or backwards (unrelated to userData - vectorLane - travelDir),
     *   used when thinking about flipping the lane */
    bool _odrFwd;
    bool _flippable; ///< whether the lane can be flipped from forward to backwards.
    /// Variables needed for the OpenDRIVE variable width:
    scalar _odrSo; ///< start of the lane section (metres), needed for the variable width, variable speed, elevation...
    std::vector<Odr::offset> _odrWidth; ///< the 5 parametres that define the width;

    OneVersion::OVID _ovID; ///< the OneVersion id of the lane.

};

}

#endif // ODRONES_LANE_H
