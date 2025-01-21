
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
