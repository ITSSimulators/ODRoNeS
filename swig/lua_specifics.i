// Option 1:
// %include "std_array.i"
// %template(arr2) std::array<double, 2>

// Option 2:
// %include <std/std_array.i>
// %template(arr2) std::array<double, 2>

// Option 3:
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
