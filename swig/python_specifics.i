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


%include "std_array.i"
%template(arr2) std::array<double, 2>;

%include "constants.h"
%include "matvec.h"

%ignore odrones::geometry::operator=;
%ignore odrones::bezier::operator=;
%ignore odrones::Odr::offset::operator=;
%rename (_print) odrones::Odr::offset::print;
%rename (_print) odrones::Odr::geometry::print;
%rename (_print) odrones::Odr::smaL::print;
%rename (_print) odrones::Odr::smaS::print;
%rename (_print) odrones::lane::print;
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
