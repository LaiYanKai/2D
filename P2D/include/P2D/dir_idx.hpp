#include "types.hpp"
#include "math.hpp"
#include "Vec2.hpp"

#pragma once
namespace P2D
{
    inline bool inRange(const dir_idx_t &dir_idx) { return dir_idx >= 0 && dir_idx <= 7; }
    inline bool isOrdinal(const dir_idx_t &dir_idx) { return (dir_idx & 1) == 1; }
    inline bool isCardinal(const dir_idx_t &dir_idx) { return (dir_idx & 1) == 0; }

    template <typename T>
    inline dir_idx_t dirToDirIdx(const T &dir_x, const T &dir_y)
    {
        assert(dir_x != 0 || dir_y != 0);
        if (dir_x > 0)
        {
            if (dir_y > 0)
                return 1;
            else if (dir_y < 0)
                return 7;
            else
                return 0;
        }
        else if (dir_x < 0)
            if (dir_y > 0)
                return 3;
            else if (dir_y < 0)
                return 5;
            else
                return 4;
        else if (dir_y > 0)
            return 2;
        else if (dir_y < 0)
            return 6;
        else
            return 8;
    }
    inline dir_idx_t dirToDirIdx(const V2 &dir) { return dirToDirIdx(dir.x, dir.y); }

    template <typename T>
    inline void dirIdxToDir(const dir_idx_t &dir_idx, T &sgn_dir_x, T &sgn_dir_y)
    {
        assert(dir_idx >= 0 && dir_idx <= 7);
        switch (dir_idx)
        {
        case 0:
            sgn_dir_x = 1;
            sgn_dir_y = 0;
            break;
        case 1:
            sgn_dir_x = 1;
            sgn_dir_y = 1;
            break;
        case 2:
            sgn_dir_x = 0;
            sgn_dir_y = 1;
            break;
        case 3:
            sgn_dir_x = -1;
            sgn_dir_y = 1;
            break;
        case 4:
            sgn_dir_x = -1;
            sgn_dir_y = 0;
            break;
        case 5:
            sgn_dir_x = -1;
            sgn_dir_y = -1;
            break;
        case 6:
            sgn_dir_x = 0;
            sgn_dir_y = -1;
            break;
        case 7:
            sgn_dir_x = 1;
            sgn_dir_y = -1;
            break;
        }
    }

    template <class T>
    inline T dirIdxToDir(const dir_idx_t &dir_idx)
    {
        T sgn_dir;
        dirIdxToDir(dir_idx, sgn_dir[0], sgn_dir[1]);
        return sgn_dir;
    }
    inline V2 dirIdxToDir(const dir_idx_t &dir_idx) { return dirIdxToDir<V2>(dir_idx); }

    template <bool inRange = true>
    inline dir_idx_t addDirIdx(const dir_idx_t &dir_idx1, const dir_idx_t &dir_idx2)
    { // values must be positive (even if negative is accepted by cpp standard)
        // inRange if both values are between 0 and 7 inclusive
        if constexpr (inRange)
        {
            constexpr dir_idx_t LUT[15] = {0, 1, 2, 3, 4, 5, 6, 7, 0, 1, 2, 3, 4, 5, 6};
            return LUT[dir_idx1 + dir_idx2];
        }
        else
        {
            unsigned long sum = dir_idx1 + dir_idx2;
            while (sum >= 8)
                sum -= 8;
            return sum;
        }
    }

    template <bool inRange = true>
    inline signed_dir_idx_t sbtDirIdx(const dir_idx_t &dir_idx1, const dir_idx_t &dir_idx2)
    {
        // inRange if both values are between 0 and 7 inclusive
        if constexpr (inRange)
        {
            signed_dir_idx_t diff = dir_idx1 - dir_idx2; // cast
            if (diff < -4)
                diff += 8;
            else if (diff > 3)
                diff -= 8;
            return diff;
        }
        else
        {
            long diff = dir_idx1 - dir_idx2;
            while (diff < -4)
                diff += 8;
            while (diff > 3)
                diff -= 8;
            return signed_dir_idx_t(diff);
        }
    }
}