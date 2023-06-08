#include <math.h>
#include <limits>
#include <stdexcept>
#include <ostream>
#include <assert.h>
#include <sstream>
#include <iomanip>

#pragma once
namespace RR2star
{
    // ============================ Side ===========================
    enum class Side
    {
        L = 0,
        R = 1
    };
    inline Side operator!(const Side &side) { return side == Side::L ? Side::R : Side::L; }
    inline std::ostream &operator<<(std::ostream &out, const Side &side) {
        (side ==  Side::L) ? out << "L" : out << "R";
        return out;
    } 

    using int_t = int;
    using float_t = double;
    using long_t = int;
    using dir_idx_t = signed char;
    using signed_dir_idx_t = signed char;

    const float_t INF = std::numeric_limits<float_t>::infinity();
    const float_t NaN = std::numeric_limits<float_t>::quiet_NaN();
    const float_t CMP_THRES = 1e-8;

    template <typename T>
    inline T sgn(T value) { return (T(0) < value) - (value < T(0)); }

    template <typename T = float_t, typename U>
    T norm(U x1, U y1, U x2, U y2)
    {
        U dx = x2 - x1;
        U dy = y2 - y1;
        return sqrt(dx * dx + dy * dy);
    }

    template <typename T>
    T dirToDirIdx(const T &dir_x, const T &dir_y)
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
    template <class T>
    dir_idx_t dirToDirIdx(const T &dir) { return dirToDirIdx(dir[0], dir[1]); }

    template <typename T>
    void dirIdxToDir(const dir_idx_t &dir_idx, T &sgn_dir_x, T &sgn_dir_y)
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
    T dirIdxToDir(const dir_idx_t &dir_idx)
    {
        T sgn_dir;
        dirIdxToDir(dir_idx, sgn_dir[0], sgn_dir[1]);
        return sgn_dir;
    }

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

    inline std::string to_string(const float_t &num, const int &w = 8, const int &prec = 3, const bool &left = false)
    {
        std::stringstream out;
        out << std::fixed << std::setw(w) << std::setprecision(prec);
        if (left)
            out << std::left;
        out << num;
        if (left)
            out << std::right;

        return out.str();
    }

    inline std::string get_addr(void *v, size_t n_last_digits = 5)
    {
        std::stringstream ss;
        ss << v;
        if (v == nullptr)
            return "x NA";
        std::string st = ss.str();
        st = st.substr(st.length() - n_last_digits - 1);
        ss.str(std::string());
        ss << "x" << st;
        return ss.str();
    }

    // returns true if val1 + CMP_THRES is gt val2
    inline bool approxGt(const float_t &val1, const float_t &val2) { return val1 - CMP_THRES > val2; }
    // returns true if val1 - CMP_THRES is gt val2
    inline bool approxGe(const float_t &val1, const float_t &val2) { return val1 + CMP_THRES > val2; }
    // returns true if abs(val1 - val2) < CMP_THRES*2
    inline bool approxEq(const float_t &val1, const float_t &val2) { return std::abs(val1 - val2) < CMP_THRES * 2; }

    float_t getPathCost(const std::vector<V2> &path)
    {
        assert(path.size() != 1);
        if (path.empty())
            return NaN;
        else
        {
            float_t cost = 0;
            for (size_t i = 1; i < path.size(); ++i)
                cost += norm(path[i - 1], path[i]);
            return cost;
        }
    }
}