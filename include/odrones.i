%module odrones

%{
#include <limits>
#include "constants.h"
#include "matvec.h"
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
typedef odrones::scalar scalar;
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

%include "readOdr.h"
%include "readBOdr.h"
%include "geometry.h"
%include "parametric.h"
%include "bezier.h"
%include "bezier2.h"
%include "bezier3.h"
%include "rnsconcepts.h"
%include "rns.h"
