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
#include "lane.h"
#include "lCoord.h"
#include "section.h"
#include "rns.h"
%}

%include "std_string.i"
%include "stdint.i"
%include "std_vector.i"

typedef odrones::uint uint;
%rename(Odr_geometry) odrones::Odr::geometry;
%template(GeometryVector) std::vector<odrones::Odr::geometry>;
%template(OffsetVector) std::vector<odrones::Odr::offset>;
%template(SpeedVector) std::vector<odrones::Odr::speedLimit>;
%template(TsignVector) std::vector<odrones::Odr::tsign>;
%template(SmaLVector) std::vector<odrones::Odr::smaL>;
%template(smaSVector) std::vector<odrones::Odr::smaS>;


