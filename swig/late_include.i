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
%include "numerical.h"
%include "lane.h"
%include "lCoord.h"
%include "section.h"
%include "rns.h"
