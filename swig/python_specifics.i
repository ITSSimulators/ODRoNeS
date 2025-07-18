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

