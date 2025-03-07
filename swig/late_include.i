%apply double & OUTPUT { scalar &d1, scalar &d2 };

%include "Odr.h"
%include "readOdr.h"
%include "readBOdr.h"
%include "geometry.h"
%include "parametric.h"
%include "bezier.h"
%include "bezier2.h"
%include "bezier3.h"
%include "rnsconcepts.h"

//! Include matvec.h before rns.h ???
%include "matvec.h"

%include "rns.h"
