
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
%include "std_vector.i"

%include "matvec.h"

typedef odrones::uint uint;
%rename(Odr_geometry) odrones::Odr::geometry;
%template(GeometryVector) std::vector<odrones::Odr::geometry>;
%template(OffsetVector) std::vector<odrones::Odr::offset>;
%template(TsignVector) std::vector<odrones::Odr::tsign>;
%template(SmaLVector) std::vector<odrones::Odr::smaL>;
%template(smaSVector) std::vector<odrones::Odr::smaS>;


