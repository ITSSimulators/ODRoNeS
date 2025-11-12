#ifndef ODRONES_LCOORD_H
#define ODRONES_LCOORD_H

#include "lane.h"

namespace odrones {

// 3 - Logical coordinates.
class lCoord
{
public:
    lCoord()
    {
        _l = nullptr;  _pos = {0., 0.}; _s = 0; _loff = 0;
    }
    lCoord(const lane *l, const arr2& pos, scalar s, scalar loff)
    {
        _l = l; _pos = pos; _s = s; _loff = loff;
    }


    // // Accessors // //
    void l(const lane *l) { _l = l; }
    const lane* l() const { return _l; }
    void deleteL() { delete _l; _l = nullptr; }

    void pos(const arr2& pos) { _pos = pos; }
    arr2 pos() const { return _pos; }
    bool posProjectedFromPos(const arr2& pos) { return _l->projectPointOntoLane(_pos, pos); }
    bool posConsistentWithLandS()
    {
        if ((!_l) || (!sInRange()))
            return false;
        _l->getPointAtDistance(_pos, _s);
        return true;
    }
    bool posConsistentWithLandSandLoff()
    {
        if ((!_l) || (!sInRange()))
            return false;
        _l->getPointWithOffset(_pos, _s, _loff);
        return true;
    }

    void s(scalar s) { _s = s; }
    scalar s() const { return _s; }
    void sConsistentWithLAndPos() { _s = _l->unsafeDistanceFromTheBoL(_pos); }
    bool sInRange() const
    {
        if ((_s < 0) || (!_l) || (_s > _l->getLength()))
            return false;
        return true;
    }

    scalar loff() const { return _loff; }
    void loff(scalar loff) { _loff = loff; }
    bool loff(const arr2& p) ///< assumming that pos has already been set!
    {
        if (!_l)
            return false;

        // magnitude:
        _loff = mvf::distance(p, _pos);
        if (mvf::areSameValues(_loff, 0))
            return true;

        // sign:  (in fact, p has to be at a +- pi/2 angle)
        vec2 v = {p[0] - _pos[0], p[1] - _pos[1]};
        if (v.cross(tangent()) < 0)
            _loff *= -1;

        return true;
    }
    // // End of Accessors // //


    // // Utilities // //
    scalar toEOL() const ///< return the distance to the end of the lane; -1 if no lane.
    {
        if (!_l) return -1;
        return _l->getLength() - _s;
    }

    std::string print() const
    {
        if (_l)
            return _l->getCSUID() + " s: " + std::to_string(_s) + " loff: " + std::to_string(_loff)
                   + " pos: (" + std::to_string(_pos[0]) + ", " + std::to_string(_pos[1]) + ")";
        else
            return "invalid lane";
    }


    void setOrigin()
    {
        _pos = _l->getOrigin();
        _s = 0;
        _loff = 0;
    }
    void setDestination()
    {
        _pos = _l->getDestination();
        _s = _l->getLength();
        _loff = 0;
    }
    arr2 tangent()
    {
        if (!_l)
            std::cerr << "[ lCoord::tangent ] has a its lane set as a nullptr!" << std::endl;

        else if ( (_l != _tgCacheL) || (!mvf::areSamePoints(_pos, _tgCachePos)) )
        {
            _tg = _l->getTangentInPoint(_pos);
            _tgCacheL = _l;
            _tgCachePos = _pos;
        }
        return _tg;
    }
    // // End of Utilities // //

private:
    const lane* _l;       ///< the lane it's on.
    arr2 _pos;            ///< position projected onto the center of the lane
    scalar _s;            ///< distance down the lane
    scalar _loff;         ///< lateral offset, positive to the right in the direction of the lane (starboard).
    arr2 _tg;             ///< cached tangent given l and pos.
    arr2 _tgCachePos{0., 0.}; ///<
    const lane* _tgCacheL{nullptr};
};

}

#endif // ODRONES_LCOORD_H
