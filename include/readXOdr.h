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

#ifndef ODRONES_READXODR_H
#define ODRONES_READXODR_H

#include "readOdr.h"

namespace odrones 
{

class ReadXOdr : public ReadOdr
{
public:
    /*! Constructor where iFile is either the input file name (isOdrFile == true) or
     *                                    the text of the file itself (isOdrFile == false) */
    ReadXOdr(std::string iFile, bool isOdrFile);

    ReadXOdr() : ReadOdr(ReadOdr::kind::xodr) {}

private:
    int loadXodr(std::string iFile, bool isOdrFile); ///< return non-zero in case of error.

    void readHeader(tinyxml2::XMLElement *header);

    /*! Read and return a Junction, given the Junction XMLElement */
    std::vector<Odr::connection> readJunction(tinyxml2::XMLElement *c);

    /*! Read all the geometry of an XML Road into a vector */
    std::vector<Odr::geometry> readGeometry(tinyxml2::XMLElement *pv);

    /*! Read all the laneOffset the lanes have given a Lanes XMLElement */
    std::vector<Odr::offset> readLaneOffset(tinyxml2::XMLElement *lanes);

    /*! Read all the Signals gien a Signals XMLElement */
    std::vector<Odr::tsign> readTrafficSigns(tinyxml2::XMLElement *xmlsgns);

    /*! return true if there's some laneOffset in the Lanes XMLElement */
    bool hasLaneOffset(tinyxml2::XMLElement *lanes);

    /*! return true if anything apart from a is non-zero in the offset */
    bool hasComplicatedOffset(Odr::offset &o);

    /*! Get the LinkID and Link Connection Point out of a Road Link XMLElement */
    int getRoadLinkData(uint &rLinkID, uint &rLinkCP, tinyxml2::XMLElement *fbLinkXML);

    /*! Given the lane XMLElement, consider adding a lane to the section, in ndxL
     *   with laneSection index ndxLS and starting at startingS.
     *  Increase +1 ndxL if so, return negative in case of error */
    int addLane(tinyxml2::XMLElement *road, tinyxml2::XMLElement *lane, uint ndxS, uint ndxL, uint ndxLS, double startingS);

    /*! Add previous and next lanes */
    uint linkLanes(tinyxml2::XMLElement *lXML, uint ndxS, uint ndxL, uint rPrevID, uint rNextID); //, int rPrevCP, int rNextCP);

    /*! Read University of Leeds Simulator5 user data */
    void readSim5UserData(tinyxml2::XMLElement* header);
};

} // namespace odrones;

#endif // ODRONES_READXODR_H
