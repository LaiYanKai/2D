#include "P2D/P2D.hpp"
#include <unordered_map>

#pragma once
namespace P2D::T2
{
    using crnkey_t = uint32_t;
    inline crnkey_t getCrnKey(const mapkey_t &mkey, const dir_idx_t &di_occ) noexcept { return (crnkey_t(mkey) << 3) | di_occ; }

    class Node;
    class Corner
    {

    public:
        V2 coord = {0, 0};
        Node *node = nullptr;
        Corner *nb_left = nullptr, *nb_right = nullptr;
        mapkey_t mkey = 0;
        dir_idx_t di = 0;
        bool convex = false;

        // full constructor
        Corner(const V2 &coord, Node *const &node, Corner *const &crn_l, Corner *const &crn_r, const mapkey_t &mkey, const dir_idx_t &di, const bool &convex)
            : coord(coord), node(node), nb_left(nb_left), nb_right(nb_right), mkey(mkey), di(di), convex(convex) {}
        // constructor from trace
        Corner(const V2 &coord, const mapkey_t &mkey, const dir_idx_t &di, const bool &convex) : coord(coord), mkey(mkey), di(di), convex(convex) {}
        // default constructor
        Corner() {}

        inline Corner *&neighbor(const Side &side) { return side == Side::L ? nb_left : nb_right; }
        inline Corner *const &neighbor(const Side &side) const { return side == Side::L ? nb_left : nb_right; }
        inline dir_idx_t edgeDi(const Side &side) const
        {
            return this->convex ? addDirIdx(this->di, side == Side::L ? 1 : 7) : addDirIdx<true>(this->di, side == Side::L ? 3 : 5);
        }
        inline V2 edgeVec(const Side &side) const { return dirIdxToDir(this->edgeDi(side)); }

        friend std::ostream &operator<<(std::ostream &out, const Corner &crn)
        {
            if (crn.di == 0)
                out << "#";
            else
                out << (crn.convex ? "+" : "-"); // << int(crn.di_occ) ;
            out << crn.coord;                    // << "|" << std::setw(10) << std::setfill('0') << crn.mkey << std::setfill(' ');
            return out;
        }
    };

    inline const Corner null_crn = Corner();
    inline std::ostream &operator<<(std::ostream &out, const Corner *const &crn)
    {
        if (crn == nullptr)
            out << "    NA    ";
        else
            out << *crn;
        return out;
    }

    class Corners
    {
    private:
        std::unordered_map<crnkey_t, Corner> _data;

    public:
        inline std::pair<Corner *, bool> tryEmplace(const Corner &crn) // does not copy adjacent corners
        {
            crnkey_t ckey = getCrnKey(crn.mkey, crn.di);
            auto p = _data.try_emplace(ckey, crn.coord, crn.mkey, crn.di, crn.convex);
            return std::make_pair(&(p.first->second), p.second);
        }
        inline void clear() { _data.clear(); }
    };
}