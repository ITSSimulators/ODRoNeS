%module odrones

%import readOdr.i
%{
#include <limits>
#include "constants.h"
#include "matvec.h"
typedef odrones::scalar scalar;
typedef odrones::arr2 arr2;
#include "bezier.h"
#include "bezier2.h"
#include "bezier3.h"
#include "rns.h"
%}



%include "std_array.i"
%include "std_string.i"
%include "stdint.i"
typedef odrones::scalar scalar;
typedef odrones::uint uint;
%include "constants.h"
%include "matvec.h"
%template(arr2) std::array<odrones::scalar, 2>;

%ignore odrones::geometry::operator=;
%ignore odrones::bezier::operator=;

%include "geometry.h"
%include "parametric.h"
%include "bezier.h"
%include "bezier2.h"
%include "bezier3.h"
%include "rns.h"
