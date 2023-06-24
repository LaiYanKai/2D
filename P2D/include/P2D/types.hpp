#include <math.h>
#include <limits>
#include <stdexcept>
#include <ostream>
#include <assert.h>
#include <sstream>
#include <iomanip>

#pragma once
namespace P2D
{
    using int_t = int;
    using float_t = double;
    using long_t = int;
    using dir_idx_t = signed char;
    using signed_dir_idx_t = signed char;
    using mapkey_t = int32_t;
    const float_t INF = std::numeric_limits<float_t>::infinity();
    const float_t THRES = 1e-8;
    template <typename T>
    class Vec2;
    using V2 = Vec2<int_t>;
    using V2l = Vec2<long_t>;
    using V2f = Vec2<float_t>;

    // short hand for static casting enum classes to int
    template <typename T>
    inline int scast(const T &t) noexcept { return static_cast<int>(t); }

    // ============================ Side ===========================
    enum class Side
    {
        L = 0,
        R = 1
    };
    inline Side operator!(const Side &side) { return side == Side::L ? Side::R : Side::L; }
    inline std::ostream &operator<<(std::ostream &out, const Side &side)
    {
        (side == Side::L) ? out << "L" : out << "R";
        return out;
    }

    // ============================ ListDir ===========================
    enum class ListDir
    {
        Prev = 0,
        Next = 1
    };
    inline ListDir operator!(const ListDir &ldir) { return ldir == ListDir::Prev ? ListDir::Next : ListDir::Prev; }

    // ============================ Erasure ===========================
    enum class Erasure
    {
        Keep = 0,
        Del = 1
    };

}