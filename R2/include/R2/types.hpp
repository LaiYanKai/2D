#include <iostream>
#include <iomanip>
#include <assert.h>
#include <type_traits>
#include <memory>
#include "P2D/P2D.hpp"

#pragma once
namespace P2D::R2
{
    using Path = std::vector<V2>;

    template <class T>
    void _instances([[maybe_unused]] const bool &inc)
    {
#if P2D_DEBUG
        inc ? ++T::_cnt : --T::_cnt;
#endif
    }

    enum Vis
    {
        No = 0,
        Unknown = 1,
        Yes = 2,
    };
    enum TreeDir
    {
        Src = 0,
        Tgt = 1
    };
    enum ListDir
    {
        Front = 0,
        Back = 1
    };
    enum EraseState
    {
        Keep = 0,
        Del = 1,
    };
    enum QueryType
    {
        Trace,
        Cast,
        // TraceFromTgt
    };
    enum UnitType
    {
        // Pseudo target point
        PS, 
        // Expensive Unknown vis to start
        EU,
        // Expensive vis to start
        EY,
        // Src, Unknown vis to start
        SU, 
        // Src, vis to start
        SY,
        // Tgt, Unknown vis to goal
        TU,
        // Tgt, vis to goal
        TY,
        // Pseudo Tgt at edge. when reached, search is invalid.
        TD,
        // Tgt, traced from tgt
        TR,
    };

    ListDir operator!(const ListDir &ldir);
    TreeDir operator!(const TreeDir &tdir);
    std::ostream &operator<<(std::ostream &out, const Vis &vis);
    std::ostream &operator<<(std::ostream &out, const TreeDir &tdir);
    std::ostream &operator<<(std::ostream &out, const QueryType &qtype);
    std::ostream &operator<<(std::ostream &out, const UnitType &utype);
}