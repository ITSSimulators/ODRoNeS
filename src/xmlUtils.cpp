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



#include <tinyxml2.h>
// #include <format>
#include <boost/format.hpp>
#include "xmlUtils.h"

using namespace odrones;

void xmlUtils::CheckResult(int err)
{
	if (err != tinyxml2::XML_SUCCESS)
	{
		std::cout << "tinyXML2 Error: " << err << std::endl;
	}
}

void xmlUtils::ReadConstCharAttr(tinyxml2::XMLElement *elem, const char *c, std::string &s)
{
    const char *txt = elem->Attribute(c);
    if (txt) s = txt;
    else s = "";
}

std::string xmlUtils::ReadConstCharAttr(tinyxml2::XMLElement *elem, const char *c)
{
    std::string s;
    ReadConstCharAttr(elem, c, s);
    return s;
}


void xmlUtils::setAttrDouble(tinyxml2::XMLElement *elem, const char *c, double q)
{
	 elem->SetAttribute(c, (boost::format("%.17e") % q).str().c_str());
    // elem->SetAttribute(c, (std::format("{0:.17e}", q).c_str()));
}



void xmlUtils::setAttrOffsetS(tinyxml2::XMLElement *elem, double s, double a, double b, double c, double d)
{
    setAttrDouble(elem, Odr::Attr::S, s);
    setAttrDouble(elem, Odr::Attr::A, a);
    setAttrDouble(elem, Odr::Attr::B, b);
    setAttrDouble(elem, Odr::Attr::C, c);
    setAttrDouble(elem, Odr::Attr::D, d);
}

void xmlUtils::setAttrOffsetS(tinyxml2::XMLElement *elem, const Odr::offset &off)
{
    setAttrOffsetS(elem, off.s, off.a, off.b, off.c, off.d);
}

void xmlUtils::setAttrOffsetSOffset(tinyxml2::XMLElement *elem, double s, double a, double b, double c, double d)
{
    setAttrDouble(elem, Odr::Attr::sOffset, s);
    setAttrDouble(elem, Odr::Attr::A, a);
    setAttrDouble(elem, Odr::Attr::B, b);
    setAttrDouble(elem, Odr::Attr::C, c);
    setAttrDouble(elem, Odr::Attr::D, d);
}

void xmlUtils::setAttrOffsetSOffset(tinyxml2::XMLElement *elem, const Odr::offset &off)
{
    setAttrOffsetSOffset(elem, off.s, off.a, off.b, off.c, off.d);
}
