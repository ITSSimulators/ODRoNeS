%module odrones

%{
#include <limits>
#include "constants.h"
#include "matvec.h"
#include "Odr.h"
#include "readOdr.h"
#include "readBOdr.h"
using namespace odrones;
#include "bezier.h"
#include "bezier2.h"
#include "bezier3.h"
#include "rnsconcepts.h"
#include "rns.h"
%}

%include "std_string.i"
%include "stdint.i"
%include "std_array.i"
%include "std_vector.i"
// typedef odrones::scalar scalar;
typedef odrones::uint uint;
// %template(arr2) std::array<odrones::scalar, 2>;
%template(arr2) std::array<double, 2>;
%rename(Odr_geometry) odrones::Odr::geometry;
%template(GeometryVector) std::vector<odrones::Odr::geometry>;
%template(OffsetVector) std::vector<odrones::Odr::offset>;
%template(TsignVector) std::vector<odrones::Odr::tsign>;
%template(SmaLVector) std::vector<odrones::Odr::smaL>;
%template(smaSVector) std::vector<odrones::Odr::smaS>;


%include "constants.h"
%include "matvec.h"

%ignore odrones::geometry::operator=;
%ignore odrones::bezier::operator=;
%ignore odrones::Odr::offset::operator=;
%ignore odrones::Odr::offset::operator*;
%rename (_print) odrones::Odr::offset::print;
%rename (_print) odrones::Odr::geometry::print;
%ignore odrones::ReadOdr::operator=;

%ignore odrones::RNS::operator=;
%ignore odrones::RNS::operator[];
%extend odrones::RNS{
    section& __getitem__(unsigned int i) {
        return (*self)[i];
    }
}

%rename(Odr_Kind_False) odrones::Odr::Kind::False;
%rename(Odr_Kind_True) odrones::Odr::Kind::True;
%rename(Odr_Kind_None) odrones::Odr::Kind::None;
%rename(Odr_laneLink_from) odrones::Odr::laneLink::from;

%include "Odr.h"
%include "readOdr.h"
%include "readBOdr.h"
%include "geometry.h"
%include "parametric.h"
%include "bezier.h"
%include "bezier2.h"
%include "bezier3.h"
%include "rnsconcepts.h"
%include "rns.h"
