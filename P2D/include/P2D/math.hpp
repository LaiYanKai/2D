#include <math.h>
#include <stdexcept>
#include <ostream>
#include <assert.h>
#include <sstream>
#include <iomanip>
#include "types.hpp"

#pragma once
namespace P2D
{

    template <typename T>
    inline T sgn(T value) { return (T(0) < value) - (value < T(0)); }

    template <typename T = float_t, typename U>
    T norm(U x1, U y1, U x2, U y2)
    {
        U dx = x2 - x1;
        U dy = y2 - y1;
        return sqrt(dx * dx + dy * dy);
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

    // returns true if val1 + THRES is gt val2
    inline bool approxGt(const float_t &val1, const float_t &val2) { return val1 - THRES > val2; }
    // returns true if val1 - THRES is gt val2
    inline bool approxGe(const float_t &val1, const float_t &val2) { return val1 + THRES > val2; }
    // returns true if abs(val1 - val2) < THRES*2
    inline bool approxEq(const float_t &val1, const float_t &val2) { return std::abs(val1 - val2) < THRES * 2; }

}