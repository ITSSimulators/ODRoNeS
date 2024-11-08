%module odrones

%{
#include <limits>
#include "constants.h"
#include "matvec.h"
#include "bezier.h"
#include "bezier2.h"
#include "bezier3.h"
%}

namespace odrones
{
   typedef unsigned int uint;
   typedef double scalar;
   typedef std::array<scalar, 2> arr2;
}

%ignore odrones::geometry::operator=;
%ignore odrones::bezier::operator=;

%include "std_string.i"
%include "stdint.i"
%include "constants.h"
%include "matvec.h"
%include "geometry.h"
%include "parametric.h"
%include "bezier.h"
%include "bezier2.h"
%include "bezier3.h"
