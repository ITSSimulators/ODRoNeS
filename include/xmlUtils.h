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



#ifndef ODRONES_XMLUTILS_H
#define ODRONES_XMLUTILS_H


#include <iostream> 
#include "Odr.h"

namespace tinyxml2
{
    class XMLElement;
}

namespace odrones
{

class xmlUtils
{
public:
	static void CheckResult(int err);
    static void ReadConstCharAttr(tinyxml2::XMLElement *elem, const char *c, std::string &s);
    static std::string ReadConstCharAttr(tinyxml2::XMLElement *elem, const char *c);

    static void setAttrDouble(tinyxml2::XMLElement *elem, const char *c, double q);
    static void setAttrOffsetS(tinyxml2::XMLElement *elem, double s, double a, double b, double c, double d);
    static void setAttrOffsetS(tinyxml2::XMLElement *elem, const Odr::offset &off);
    static void setAttrOffsetSOffset(tinyxml2::XMLElement *elem, double s, double a, double b, double c, double d);
    static void setAttrOffsetSOffset(tinyxml2::XMLElement *elem, const Odr::offset &off);

};



}



#endif // ODRONES_XMLUTILS_H
