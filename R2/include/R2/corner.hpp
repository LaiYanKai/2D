#include "P2D/P2D.hpp"
#include "types.hpp"
#include <unordered_map>

#pragma once
namespace P2D::R2
{
    using crnkey_t = mapkey_t;
    inline crnkey_t getCrnKey(const mapkey_t &mapkey, const dir_idx_t &di_occ) noexcept
    {
        return (mapkey << 3) | di_occ;
    }

    class Node;
    struct Corner
    {
        V2 coord = {0,0};
        Node *node = nullptr;
        Corner *left = nullptr, *right = nullptr;
        mapkey_t mkey = -1;
        bool is_convex = false;
        dir_idx_t di_occ = 0;

        Corner(const bool &is_convex, const mapkey_t &mkey, const dir_idx_t &di_occ, const V2 &coord)
            : coord(coord), mkey(mkey), is_convex(is_convex), di_occ(di_occ) {}
        Corner() {}

        inline Corner *&trace(const Side &side) { return side == Side::L ? left : right; }
        inline Corner *const &trace(const Side &side) const { return side == Side::L ? left : right; }
        inline dir_idx_t edgeDi(const Side &side) const
        {
            return this->is_convex ? addDirIdx<true>(this->di_occ, side == Side::L ? 1 : 7) : addDirIdx<true>(this->di_occ, side == Side::L ? 3 : 5);
        }
        inline V2 edgeVec(const Side &side) const { return dirIdxToDir<V2>(this->edgeDi(side)); }
    };
    std::ostream &operator<<(std::ostream &out, const Corner &crn);
    std::ostream &operator<<(std::ostream &out, const Corner *const &crn);

    class Corners
    {
    private:
        std::unordered_map<crnkey_t, Corner> _data;

    public:
        inline std::pair<Corner *, bool> tryEmplace(const Corner &crn) // does not copy adjacent corners
        {
            crnkey_t ckey = getCrnKey(crn.mkey, crn.di_occ);
            auto p = _data.try_emplace(ckey, crn.is_convex, crn.mkey, crn.di_occ, crn.coord);
            return std::make_pair(&(p.first->second), p.second);
        }
        inline void clear() { _data.clear(); }
    };
}