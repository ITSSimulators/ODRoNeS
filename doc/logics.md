ODRoNeS (OpenDRIVE Road Network System)
=======================================
The Road Network System `rns` is essentially an array of connected `sections` on a system 
 that will allow you to use both cartesian to road coordinates interchangeably.

`sections`, are essentially a dynamic array of (roughly) parallel lanes
 (matching OpenDRIVE laneSections)
 that can be accessed through operator[] and that can access lanes through operator[] 
 and they are the ODRoNeS equivalent to OpenDRIVE roads.

`lane` is a class that has an origin, destination, and geometrical characteristics
 (that will be different depending on their shape)
 and that will provide information on:
  * distance to the end of the lane,
  * heading of the lane at some point,
  * return a point on the lane after some distance,
  * provide information about the next and previous lane,
  * project a point onto the lane,
  * whether a point is on the lane or not,
  * bounding boxes,
  * intersections with other lanes.
  However, it relies on the geometry (and derived) class(es) to do all the calculations.
  More details can be found later, in the [lanes](#lanes) subsection.

Thus, lanes calculate these things differently depending on whether their geometry,
  and the underlying maths belong to the `mvf` class.

Ultimately, see the `rns.h` and the `lane.h` headers to find out more.



Lanes
-----
Next/Previous lanes:
 * nextLane is an array of lane pointers, where this lane is connected to, 
 and similarly with prevLane. It may be worth stressing that each lane is
 **continuously connected** to the next lanes at its end, 
 and to the previous lanes at its beginning.

 * and when calling setPrevLane, things get automatically set NextLanes,
 and viceversa.

Two lanes diverging from (but not converging to) a point must be kept 
 in different sections.

Port/Starboard lanes:
 * Lanes within a section have `port` and `starboard` lanes.
 * `port` and `starboard` lanes are of the same kind, i e,
   they're all `driving` or `walking` but they can be meant to travel
   in different directions.

Everylane has a vector of `geometries` defining its shape. 
 Currently, ODRoNeS supports the OpenDRIVE shapes straight, arc,
 paramPoly3 and spiral, together with Bezier curves.

Unless otherwise specified, ` lane::foo(scalar s) ` is a function that 
 depends on the distance travelled **down the lane** and does not correspond
 to the `s` coordinate of the OpenDRIVE lane 0. 
 Geometries can transform local distance to lane-zero distance 
 via `geometry::sl0(scalar s)`, and exposing this from `lane` 
 is straightforward.

Every lane has a vector of conflicts, such as crosswalks, or give-way. 
 Conflict functionality is useful for simulation.


## Naming ##
ODRoNeS uses a different convention to the one used OpenDRIVE. 
 Still, the original OpenDRIVE naming is preserved in  `lane::_odrID` for the lane
 and in `lane::_odrSectionID` for the road,
 and calling lane::getOdrSUID() will print the OpenDRIVE name.


## New geometries types ##
 The shortest way of implementing a new geometry is to do so in the following order:
 0. - geometry.invert.
 1. - projectPointOntoLane.
 2. - isPointOnLane.
 3. - getGeomIndexForPoint
 4. - getTangentInPoint.
 5. - unsafeDistanceToTheEoL.
 6. - getPointAfterDistance.
 7. - getIntersectionPointFromOT.
 8. - getCurvature.
 9. - getQPainterPath.
 10. - copy/alloc procedures - without forgetting lane::assignLaneToThis, and lane::cleanMemory.

There are two virtual classes that will help in developing new geometries: parametric and numerical.
 The PROGRAMMER GUIDELINES are that new geometries:
  - derive from geometry.
  - can derive from parametric, numerical or vwNumerical (read below).
  - parametric and numerical methods will never override geometry virtual methods.


## Memory management ##
A lane keeps an array of lane pointers, ` lane** _nextLane ` and ` lane** _prevLane `, 
 and it will deallocate the array when the destructor is called.
A section creates the lanes in a lane array (`lane*`) that the section itself will deallocate.
The RNS allocates and sets an array of section (`section*`), that it will deallocate,
Everything is kept consistent inside lrn. When RNS is copied, 
 new sectors and lanes are created and there is a rewiring process to link the lanes.


## Lane direction ##
OpenDrive lanes can be forward and backward. 
  Read the code on lane::setOdrFwd.
  This could be extended easily to other types of lanes.
  Once a lane is defined as backwards, origin and destination are swapped,
  next/prev lanes are also swapped, together with port/starboard lanes,
  `_to` and everything needed is recalculated, and the lane is kept as backwards.
  
When loading an OpenDrive map using SmartActors' lrn::makeOpenDriveRoads, 
  the direction of the lanes in sections with lanes in both directions
  can be deduced if the map is assumed to be left-hand or right-hand driving.
  OpenDRIVE 1.7 may specify whether the map is left- or right-handed.
  
The direction of those lanes belonging to sections with one single direction 
  is determined later, once the routes are defined, in routes::fixDirectionality, 
  which does not use lane::sings.


### Traffic Signs ###
Traffic signs are read: 
 - kept in lrn as a vector<lane::tSigns> 
 - assigned to every lane as a vector<lane::tSigns>.
 
If a tSign is a Yield -> the lane::ending is set to giveWay,
           is a Stop -> the lane::ending is set to stop;
           

## Lane sign ##
lane::sign can be:
 * p for positive
 * n for negative 
 * o for zero

Following the OpenDrive convention, lanes with the same sign on the same section (road)
 have the same direction, and so, one can change lane there. 
 
 **lane::signs are not to be changed when a lane is set to be backwards.**
 In a correctly formed open OpenDrive map, a two-direction road (section)
 would come with some lanes in the "+" direction and some other lanes in the "-" direction,
 but all of them would start and end at the same side, and hence the "+/-" directions.
 

### laneSections ###
 - readOdr reads the "next" and "previous" lanes correctly from the link information.
 - information regarding the starting s is stored in ` geometry::_roadSo`.
 - different laneSections within the same road may have a different number of lanes. 
 - Multiple sections are created per road, one per laneSection. This means that:
    * we have different RNS lanes in different sections with the repeated Odr indices.
    * OnseSided laneSections have not been tested.


### Variable width ###
 The variable width is calculated using an array of Odr::offsets. 
   The raw data is read at readOdr and stored at Odr::smaL as arrays of widths and borders,
   as well as in smaS as arrays of lane offsets (loffset). 
    
 From the standard:
    - in laneSections, 
      ds is the distance along the road reference line between the start of a new lane offset element 
         and the given position
      ds restarts at zero for each element. The absolute position of an offset value is calculated as follows:
         s = sstart + ds
        where sstart is the start position of the element in the reference line coordinate system.
    - in width (and borders)
      ds is the distance along the road reference line between the start of a new lane width element 
         and the given position
      ds restarts at zero for each element. The absolute position of a width value is calculated as follows:
         s = ssection + offsetstart + ds

Conveniently, `lane::getWidth(scalar s)` returns the width of the lane 
  at a certain travelled distance (that is different to the s coordinate of lane 0). 


 In order to calculate the total offset, each geometry needs a consistent set of Odr::offsets.
 This is set in in section::setOdrRoad, so that for each lane:
   - the lane offset for the section is stored as an array
   - widths for the lanes in the section closer to the centre are grabbed (starting at absolute s + so)
   - and half the width of this lane is added (again starting at absolute s + so metres down the road)
   - using this grouping calculate the end of these offsets (se)
   - and try simplifying the array in Odr::offset::simplify, which will also state whether the interval (s, se)
      has to become [s, se), (s, se] or [s, se].

 This set of consistent information is then passed onto every lane so that it can be passed 
   on of its geometries. 
  Finally, every geometry 
    receives and keeps the starting coordinate s of the geometry:
    ` _roadSo `, where the lane-section starts.


## Conflicts ## 
Currently, we distinguish thee types of conflict: merge, giveWay and zebraWalk.
 
Merge conflicts define:
 * the point at which the car has to merge, 
   but the games use the end of the lane, so there is some inconsistency here.
 * the side of the merge. 
 * no HPLanes are expected or ever checked.

GiveWay is assigned to:
 * Lanes From Diferent Sections and Same Ending.
 * Crosswalk lanes (see below) checking whether meeples should cross or not.

Crosswalks are assigned to:
 * Tarmac and Pavement lanes at the position of the crosswalk.

Conflicts are added:
 * in lrn::MakeOpenDriveRoads - when we add a static object of the crosswalk type,
     we add the conflict crosswalk to every lane of the section (crosswalk).
 * in lrn::MakeOpenDriveRoads - when we create the crosswalk lanes, we add a conflict 
     on the crosswalk lanes at every road crossing (giveWay).
 * in lrn::makePriorities - knowing the directionality, when two lanes end on the same point.


### Linked Conflicts ###
Some conflicts are too close to other conflicts to be solved on their own,
 such as a crosswalk crossing two road lanes.
 **ASSUMPTION** A decent road layout can't have linked conflicts that are route
 dependent, i e, conflicts that are linked only in certain routes.
 Therefore, we pre-process the road network in makePriorities, and store the linked conflicts
 in a vector within the conflict.

A conflict can be uniquely identified with a pair of values composed of
 the lane that holds the conflict and the distance s at which this conflict is found.

Conflicts will be linked IF... 


## Ancillary Classes for geometry ##
 There are three Ancillary Classes to help out with geometries: parametric, numerical and vwNumerical.

### parametric ### 


### numerical ###
 - new class to help out with complicated geometries.
 - the derived class must:
    1. call initialise passing in ds and number of points,
    2. implement and nSetupPointsXYUniformly so that all the points 
        but the last pair are equally distanced.
    3. call setup() after initialising AND after flipping the line. 


### vwNumerical ###   
 - new class to help out with variable width numerical geometries.
 - vwNumerical implements ` scalar offset(s) `,
 - the deriving class needs to:
    1. implement `curvexy_a(scalar s)`, a function of the distance-down-the-lane0-road, in this lane-section
    2. implement `l0xy_a(scalar s)`, return the xy values of lane 0, as a function of the distance-down-the-lane0-road.
    3. start with a call `vwNumerical::base()`
    4. always call `offset(s)` as `offset(t + _roadSo)`, so that the call uses the absolute s value
    5. Include the following bit of code:
          scalar ds = numerical::defaultDs(_l);
          vwNumerical::setup(ds);
          _length = numerical::maxS();

 More details about how variable width is calculated can be read below, 
  under the section OpenDrive Extras.



OneVersion
-----------
OneVersion is an internal format that ODRoNeS supports (temporarily?) though not fully. 
 This is done by linking to OneVersion and quering the library itself, which is fine to load maps.
 Because there are very similar classes and concepts, 
 the only header allowed to import is ` readOneVersion.h `,
 and headers from OneVersion should never be imported directly in ODRoNeS.
 
 
