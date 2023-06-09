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

#ifndef SECTION_H
#define SECTION_H

#include <vector>
#include <limits>
#include <iostream>
#include <cstdarg>
#include <tuple>
#include <vector>
#include <algorithm>
#include <memory>
#include "matvec.h"
#include "readOdr.h"
#include "rnsconcepts.h"
#include "lane.h"



/*! A section is a bundle of lanes that run parallel (roughly).
 *  Lanes are kept in a vector that cannot be resized,
 *  because this could mean that the pointers to lanes
 *  leading to nextLane, prevLane, etc. would be invalidated.
 */
class section
{
public:
    section();
    section(size_t size);
    section(const section& s);
    section& operator=(const section& s);
    ~section();

    void assignInputSectionToThis(const section& s);
    void set(size_t size);

    bool isReady();

    // int addLane(lane* l); ///< add a lane to this section
    int addLane(const arr2 &origin, const arr2 &dest, scalar width, scalar speed, mvf::shape shp, lane::sign sgn);
    int addLane(const arr2 &origin, const arr2 &dest, const arr2 &centre, scalar width, scalar speed, mvf::shape shp, lane::sign sgn, bool permanent = true);
    int addLane(const std::vector<bezier2> &bzr, scalar width, scalar speed, lane::sign sgn);
    int addLane(const std::vector<bezier3> &bzr, scalar width, scalar speed, lane::sign sgn);
    int addLane(const std::vector<Odr::geometry> &geom, const std::vector<Odr::offset> &off, const Odr::smaL &odrL, scalar se);

    /*! Set port and starboard lanes (and potentially flip some lanes);
     * This method is of general applicability and uses the OpenDrive convention
     *  for which same _sign lanes within the section run in the same direction.
     * If called with either assumeLeftHandDriving or assumeRightHandDriving set to true,
     *  in those sections with multiple lanes and two directions,
     *  it will also flip the directioeither n of those lanes according to the input convention.
     */
    void setPortAndStarboard(bool assumeLeftHandDriving, bool assumeRightHandDriving);
    void setPortAndStarboard(concepts::drivingSide ds); ///< overload

    /*! Once this section has alrady been set with some lanes,
     *    and these lanes have N crosswalk sObjects in their internal vector,
     *    they need to know of the crosswalk lane.
     * This method will do this last bit of configuration.
     */
    bool setIthConflictLane(uint ithCW, lane *cwl);

    /*! this section IS a crosswalk if every lane is of type crosswalk */
    bool isCrosswalk() const;

    bool isTransitable(); ///< i. e., it has at least one transitable road

    /*! store the input data, and add all the lanes for this laneSection ID */
    void setOdrRoad(const Odr::smaS &sec, uint lsID);

    uint getOdrID() const;
    void lockOdrFlippable(); // set _odrFlippable to false for every lane in the section;
    bool flipBackwards(); ///< change the direction of every lane as backwards.

    lane* operator[](size_t index); ///< get a lane
    lane* getLane(size_t index) const; ///< get the same bloody lane
    bool isSameSection(const section *s) const; ///< true if *s == this
    size_t size() const; ///< return the amount of stuff stored
    size_t getMaxSize() const; ///< return the allocated size;

    int getID() const; ///< returns the section ID.
    void setID(int id); ///< sets the section ID.

    std::vector<lane::tSign> getTSigns() const; ///< return a vector with non-repeated traffic signs, i e, query the different _lanes and take the physical traffic signs rather than all the repeated allocations.
    void getBoundingBox(arr2 &blc, arr2 &trc) const;

    void addUpBoundingBoxes(); ///< on  set the box to the sum of all the boxes
    void updateBoundingBox(const arr2 &blc, const arr2 &trc); ///< update the bounding box defined by _bbblc, bbtrc with the input box;
private:
    void updateBoundingBox(uint ndx); ///< update the bounding box defined by _bbblc, bbtrc with the box of lane _sections[ndx];


private:
    int _id;
    lane* _lanes; ///< a dynamic array of lanes;
    size_t _writtenSize;
    size_t _allocSize;
    arr2 _bbblc, _bbtrc; ///< bounding box bottom left corner, bounding box top right corner.
    uint _odrID;
};




#endif // SECTION_H
