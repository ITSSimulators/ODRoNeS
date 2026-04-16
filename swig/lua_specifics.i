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


// Option 1:
// %include "std_array.i"
// %template(arr2) std::array<double, 2>

// Option 2:
// %include <std/std_array.i>
// %template(arr2) std::array<double, 2>

// Option 3:
typedef double scalar;
namespace odrones {

class arr2 
{
public:
    typedef size_t size_type;
    typedef ptrdiff_t difference_type;
    typedef double value_type;
    typedef value_type* pointer;
    typedef const value_type* const_pointer;
    typedef value_type& reference;
    typedef const value_type& const_reference;

	 arr2();
	 arr2(const arr2& other);

    unsigned int size() const;
    unsigned int max_size() const;
    bool empty() const;
    double front()const; // only read front & back
    double back()const;  // not write to them
    // operator [] given as an extension:

    %extend // this is a extra bit of SWIG code
    {
       // [] is replaced by __getitem__ & __setitem__
       // simply throws a string, which causes a lua error
       double __getitem__(unsigned int idx) throw (std::out_of_range)
       {
          if (idx>=self->size())
             throw std::out_of_range("in array::__getitem__()");
          return (*self)[idx];
       }
       void __setitem__(unsigned int idx,double val) throw (std::out_of_range)
       {
          if (idx>=self->size())
             throw std::out_of_range("in array::__setitem__()");
          (*self)[idx]=val;
       }
    }
};

}
