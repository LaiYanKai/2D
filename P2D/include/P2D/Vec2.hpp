#include <iostream>
#include <type_traits>
#include <iomanip>
#include <vector>
#include "types.hpp"
#include "math.hpp"

#pragma once

namespace P2D
{
    template <typename T>
    class Vec2
    {
    public:
        T x, y;
        Vec2(const T &x, const T &y) : x(x), y(y) {}
        Vec2() : Vec2(0, 0){};
        // template <typename U>
        // Vec2(const U &vec) : Vec2(vec[0], vec[1]) {}

        T &operator[](size_t dim) { return dim == 0 ? x : y; }
        const T &operator[](size_t dim) const { return dim == 0 ? x : y; }
        Vec2<T> &operator+=(const Vec2<T> &rhs)
        {
            for (size_t dim = 0; dim < 2; ++dim)
                this->operator[](dim) += rhs[dim];
            return *this;
        }
        friend Vec2<T> operator+(Vec2<T> lhs, const Vec2<T> &rhs)
        {
            lhs += rhs;
            return lhs;
        }
        Vec2<T> &operator+=(const T &rhs)
        {
            for (size_t dim = 0; dim < 2; ++dim)
                this->operator[](dim) += rhs;
            return *this;
        }
        friend Vec2<T> operator+(Vec2<T> lhs, const T &rhs)
        {
            lhs += rhs;
            return lhs;
        }
        Vec2<T> &operator+=(const T (&rhs)[2])
        {
            for (size_t dim = 0; dim < 2; ++dim)
                this->operator[](dim) += rhs[dim];
            return *this;
        }
        friend Vec2<T> operator+(Vec2<T> lhs, const T (&rhs)[2])
        {
            lhs += rhs;
            return lhs;
        }
        friend Vec2<T> operator+(Vec2<T> lhs, const std::array<T, 2> &rhs)
        {
            lhs += rhs;
            return lhs;
        }
        Vec2<T> &operator+=(const std::array<T, 2> &rhs)
        {
            for (size_t dim = 0; dim < 2; ++dim)
                this->operator[](dim) += rhs[dim];
            return *this;
        }
        Vec2<T> &operator-=(const Vec2<T> &rhs)
        {
            for (size_t dim = 0; dim < 2; ++dim)
                this->operator[](dim) -= rhs[dim];
            return *this;
        }
        friend Vec2<T> operator-(Vec2<T> lhs, const Vec2<T> &rhs)
        {
            lhs -= rhs;
            return lhs;
        }
        friend Vec2<T> operator-(Vec2<T> lhs)
        {
            lhs[0] = -lhs[0];
            lhs[1] = -lhs[1];
            return lhs;
        }
        Vec2<T> &operator-=(const T &rhs)
        {
            for (size_t dim = 0; dim < 2; ++dim)
                this->operator[](dim) -= rhs;
            return *this;
        }
        friend Vec2<T> operator-(Vec2<T> lhs, const T &rhs)
        {
            lhs -= rhs;
            return lhs;
        }
        Vec2<T> &operator-=(const T (&rhs)[2])
        {
            for (size_t dim = 0; dim < 2; ++dim)
                this->operator[](dim) -= rhs[dim];
            return *this;
        }
        friend Vec2<T> operator-(Vec2<T> lhs, const T (&rhs)[2])
        {
            lhs -= rhs;
            return lhs;
        }
        friend Vec2<T> operator-(Vec2<T> lhs, const std::array<T, 2> &rhs)
        {
            lhs -= rhs;
            return lhs;
        }
        Vec2<T> &operator-=(const std::array<T, 2> &rhs)
        {
            for (size_t dim = 0; dim < 2; ++dim)
                this->operator[](dim) -= rhs[dim];
            return *this;
        }
        Vec2<T> &operator*=(const Vec2<T> &rhs)
        {
            for (size_t dim = 0; dim < 2; ++dim)
                this->operator[](dim) *= rhs[dim];
            return *this;
        }
        friend Vec2<T> operator*(Vec2<T> lhs, const Vec2<T> &rhs)
        {
            lhs *= rhs;
            return lhs;
        }
        Vec2<T> &operator*=(const T &rhs)
        {
            for (size_t dim = 0; dim < 2; ++dim)
                this->operator[](dim) *= rhs;
            return *this;
        }
        friend Vec2<T> operator*(Vec2<T> lhs, const T &rhs)
        {
            lhs *= rhs;
            return lhs;
        }
        Vec2<T> &operator*=(const T (&rhs)[2])
        {
            for (size_t dim = 0; dim < 2; ++dim)
                this->operator[](dim) *= rhs[dim];
            return *this;
        }
        friend Vec2<T> operator*(Vec2<T> lhs, const T (&rhs)[2])
        {
            lhs *= rhs;
            return lhs;
        }
        friend Vec2<T> operator*(Vec2<T> lhs, const std::array<T, 2> &rhs)
        {
            lhs *= rhs;
            return lhs;
        }
        Vec2<T> &operator*=(const std::array<T, 2> &rhs)
        {
            for (size_t dim = 0; dim < 2; ++dim)
                this->operator[](dim) *= rhs[dim];
            return *this;
        }
        Vec2<T> &operator/=(const Vec2<T> &rhs)
        {
            for (size_t dim = 0; dim < 2; ++dim)
                this->operator[](dim) /= rhs[dim];
            return *this;
        }
        friend Vec2<T> operator/(Vec2<T> lhs, const Vec2<T> &rhs)
        {
            lhs /= rhs;
            return lhs;
        }
        Vec2<T> &operator/=(const T &rhs)
        {
            for (size_t dim = 0; dim < 2; ++dim)
                this->operator[](dim) /= rhs;
            return *this;
        }
        friend Vec2<T> operator/(Vec2<T> lhs, const T &rhs)
        {
            lhs /= rhs;
            return lhs;
        }
        Vec2<T> &operator/=(const T (&rhs)[2])
        {
            for (size_t dim = 0; dim < 2; ++dim)
                this->operator[](dim) /= rhs[dim];
            return *this;
        }
        friend Vec2<T> operator/(Vec2<T> lhs, const std::array<T, 2> &rhs)
        {
            lhs /= rhs;
            return lhs;
        }
        Vec2<T> &operator/=(const std::array<T, 2> &rhs)
        {
            for (size_t dim = 0; dim < 2; ++dim)
                this->operator[](dim) /= rhs[dim];
            return *this;
        }
        friend Vec2<T> operator/(Vec2<T> lhs, const T (&rhs)[2])
        {
            lhs /= rhs;
            return lhs;
        }
        friend Vec2<T> operator<(const Vec2<T> &lhs, const Vec2<T> &rhs)
        {
            Vec2<T> res;
            for (size_t dim = 0; dim < 2; ++dim)
                res[dim] = lhs[dim] < rhs[dim];
            return res;
        }
        friend Vec2<T> operator>(const Vec2<T> &lhs, const Vec2<T> &rhs)
        {
            Vec2<T> res;
            for (size_t dim = 0; dim < 2; ++dim)
                res[dim] = lhs[dim] > rhs[dim];
            return res;
        }
        friend Vec2<T> operator<=(const Vec2<T> &lhs, const Vec2<T> &rhs)
        {
            Vec2<T> res;
            for (size_t dim = 0; dim < 2; ++dim)
                res[dim] = lhs[dim] <= rhs[dim];
            return res;
        }
        friend Vec2<T> operator>=(const Vec2<T> &lhs, const Vec2<T> &rhs)
        {
            Vec2<T> res;
            for (size_t dim = 0; dim < 2; ++dim)
                res[dim] = lhs[dim] >= rhs[dim];
            return res;
        }
        friend bool operator==(const Vec2<T> &lhs, const Vec2<T> &rhs)
        {
            return lhs[0] == rhs[0] && lhs[1] == rhs[1];
        }
        friend bool operator!=(const Vec2<T> &lhs, const Vec2<T> &rhs)
        {
            return !(lhs == rhs);
        }
        friend bool operator==(const Vec2<T> &lhs, const T &rhs)
        {
            return lhs[0] == rhs && lhs[1] == rhs;
        }
        friend bool operator!=(const Vec2<T> &lhs, const T &rhs)
        {
            return !(lhs == rhs);
        }
        friend Vec2<T> operator!(const Vec2<T> &vec)
        {
            Vec2<T> res;
            for (size_t dim = 0; dim < 2; ++dim)
                res[dim] = !vec[dim];
            return res;
        }
        Vec2<T> &operator=(const T &rhs)
        {
            for (size_t dim = 0; dim < 2; ++dim)
                this->operator[](dim) = rhs;
            return *this;
        }

        template <typename U>
        U normSq() const
        { // obeys numeric promotion
            U _x = U(this->operator[](0));
            U _y = U(this->operator[](1));
            return _x * _x + _y * _y;
        }

        template <typename U>
        U norm() const { return std::sqrt(normSq<U>()); }
        inline float_t norm() const { return norm<float_t>(); }

        friend std::ostream &operator<<(std::ostream &out, Vec2<T> const &vec)
        {
            if (std::is_same<T, float>::value || std::is_same<T, double>::value)
                vec.stream(out, 7, 3);
            else
                vec.stream(out, 4, 0);
            return out;
        }
        std::ostream &stream(std::ostream &out, size_t width = 7, size_t precision = 3) const
        {
            if (std::is_same<T, float>::value || std::is_same<T, double>::value)
            {
                out << std::fixed;
                for (size_t dim = 0; dim < 2; ++dim)
                    out << std::setw(width) << std::setprecision(precision)
                        << this->operator[](dim) << (dim != 1 ? "," : "");
            }
            else
            {
                for (size_t dim = 0; dim < 2; ++dim)
                    out << std::setw(width)
                        << this->operator[](dim) << (dim != 1 ? "," : "");
            }
            return out;
        }
        std::string repr() const
        {
            std::stringstream ss;
            ss << *this;
            return ss.str();
        }
    };

    template <typename T>
    bool approxEqual(const Vec2<T> &lhs, const Vec2<T> &rhs, const T &thres = THRES)
    {
        bool res = true;
        for (size_t dim = 0; dim < 2; ++dim)
            res &= abs(lhs[0] - rhs[0]) < thres;
        return res;
    }

    template <typename T>
    Vec2<T> abs(const Vec2<T> &v)
    {
        Vec2<T> res = v;
        for (size_t dim = 0; dim < 2; ++dim)
            res[dim] = std::abs(v[dim]);
        return res;
    }
    template <typename T>
    Vec2<T> sgn(const Vec2<T> &v)
    {
        Vec2<T> res;
        for (size_t dim = 0; dim < 2; ++dim)
            res[dim] = sgn<T>(v[dim]);
        return res;
    }
    template <typename T>
    Vec2<T> isWhole(const Vec2<T> &vec, const T &thres = THRES)
    {
        Vec2<T> res;
        for (size_t dim = 0; dim < 2; ++dim)
            res[dim] = std::abs(vec[dim] - std::round(vec[dim])) < thres;
        return res;
    }

    template <typename T>
    Vec2<T> round(const Vec2<T> &vec)
    {
        Vec2<T> res;
        for (size_t dim = 0; dim < 2; ++dim)
            res[dim] = std::round(vec[dim]);
        return res;
    }

    template <typename T, typename U>
    U det(const Vec2<T> &lhs, const Vec2<T> &rhs)
    {
        return U(lhs[0]) * U(rhs[1]) - U(lhs[1]) * U(rhs[0]);
    }
    inline long_t det(const V2 &lhs, const V2 &rhs) { return det<int_t, long_t>(lhs, rhs); }

    template <typename T, typename U>
    U dot(const Vec2<T> &lhs, const Vec2<T> &rhs)
    {
        return U(lhs[0]) * U(rhs[0]) + U(lhs[1]) * U(rhs[1]);
    }
    inline long_t dot(const V2 &lhs, const V2 &rhs) { return dot<int_t, long_t>(lhs, rhs); }

    template <typename T, typename U>
    U normSq(const Vec2<T> &vec1, const Vec2<T> &vec2)
    {
        Vec2<T> diff = Vec2<T>(vec1) - Vec2<T>(vec2);
        return diff[0] * diff[0] + diff[1] * diff[1];
    }

    template <typename T, typename U>
    U norm(const Vec2<T> &vec1, const Vec2<T> &vec2)
    {
        return std::sqrt(normSq<T, U>(vec1, vec2));
    }
    inline float_t norm(const V2 &lhs, const V2 &rhs) { return norm<int_t, float_t>(lhs, rhs); }

    template <bool is_strict = false>
    std::pair<long_t, bool> isLeft(const V2 &v_left, const V2 &v_right)
    {
        long_t dtr = det(v_left, v_right);
        if constexpr (is_strict)
            return std::make_pair(dtr, dtr < 0);
        else
            return std::make_pair(dtr, dtr <= 0);
    }
    template <bool is_strict = false>
    std::pair<long_t, bool> dtrLeftOrRight(const Side &side, const V2 &side_left_v_left, const V2 &side_left_v_right)
    {
        if (side == Side::L)
            return isLeft<is_strict>(side_left_v_left, side_left_v_right);
        else
            return isLeft<is_strict>(side_left_v_right, side_left_v_left);
    }
    template <bool is_strict = false>
    bool isLeftOrRight(const Side &side, const V2 &side_left_v_left, const V2 &side_left_v_right)
    {
        return dtrLeftOrRight<is_strict>(side, side_left_v_left, side_left_v_right).second;
    }

    template <typename T, typename U>
    U norm(const std::vector<Vec2<T>> &path)
    {
        if (path.size() <= 1)
            return NAN;

        U cost = 0;
        for (auto coord = path.begin() + 1; coord != path.end(); ++coord)
            cost += norm<T, U>(*(coord - 1), *coord);
        return cost;
    }

    template <typename T>
    float_t norm(const std::vector<Vec2<T>> &path) { return norm<T, float_t>(path); }
}