
#ifndef ODRONES_XMLUTILS_H
#define ODRONES_XMLUTILS_H


#include <iostream> 

#include "boost/format.hpp"

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
