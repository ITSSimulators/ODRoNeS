%module readOdr

%include "std_array.i"
%include "std_vector.i"
%{
#include "readOdr.h"
using namespace odrones;
#include "readBOdr.h"
%}

%template(arr2) std::array<odrones::scalar, 2>;

%template(GeometryVector) std::vector<odrones::Odr::geometry>;
%template(OffsetVector) std::vector<odrones::Odr::offset>;
%template(TsignVector) std::vector<odrones::Odr::tsign>;
%template(SmaLVector) std::vector<odrones::Odr::smaL>;
// %template(SmaLPVector) std::vector<odrones::Odr::smaL*>;

%typedef unsigned int uint;
%typedef odrones::uint uint;


%include "readOdr.h"
%include "readBOdr.h"

