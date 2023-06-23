#include "P2D/P2D.hpp"
#include "corner.hpp"
#include "types.hpp"
#include <memory>
#include <unordered_map>

#pragma once
namespace P2D::R2
{
    using rkey_t = uint64_t; // must be twice the length of mkey_t. two ckeys together. Therefore, not more than 23000 in length for each side.
    rkey_t getRayKey(const mapkey_t &src_mkey, const dir_idx_t &src_diocc, const mapkey_t &tgt_mkey, const dir_idx_t &tgt_diocc);

    // ==================================== Collision ====================================

    // Stores collision information
    class Collision
    {
    public:
        Corner *const crn_left, *const crn_right;

        Collision(Corner *const &crn_left, Corner *const &crn_right) : crn_left(crn_left), crn_right(crn_right) {}
        Corner *const &crn(const Side &side) const { return side == Side::L ? crn_left : crn_right; }
    };

    // ==================================== Ray ====================================
    class Ray
    {
    public:
        Corner *const crn_src, *const crn_tgt;
        const V2 vec; // tgtFromSrc
        const float_t length;
        Vis vis = Vis::Unknown;
        std::unique_ptr<Collision> collision = nullptr;

        Ray(Corner *const &crn_src, Corner *const &crn_tgt);

        inline Corner *const &crn(const TreeDir &tdir) const { return tdir == TreeDir::Src ? crn_src : crn_tgt; }
        inline const V2 &coord(const TreeDir &tdir) const { return this->crn(tdir)->coord; }
        inline void setCollision(Corner *const &crn_l, Corner *const &crn_r)
        {
            assert(this->collision == nullptr);
            this->collision = std::make_unique<Collision>(crn_l, crn_r);
        }

        std::string repr(const int type = 0) const;
    };
    std::string get_addr(const Ray *const &ray);
    std::ostream &operator<<(std::ostream &out, Ray const &ray);
    std::ostream &operator<<(std::ostream &out, const Ray *const &ray);

    // ==================================== Rays ====================================

    class Rays
    {
    private:
        std::unordered_map<rkey_t, Ray> data;

    public:
        inline std::pair<Ray *, bool> tryEmplace(Corner *const &crn_src, Corner *const &crn_tgt)
        {
            rkey_t rkey = getRayKey(crn_src->mkey, crn_src->di_occ, crn_tgt->mkey, crn_tgt->di_occ);
            auto p = data.try_emplace(rkey, crn_src, crn_tgt);
            return std::make_pair(&(p.first->second), p.second);
        }
        // key must be found
        inline Ray *at(Corner *const &crn_src, Corner *const &crn_tgt)
        {
            rkey_t rkey = getRayKey(crn_src->mkey, crn_src->di_occ, crn_tgt->mkey, crn_tgt->di_occ);
            return &data.at(rkey); // will raise exception if cannot be found
        }
        inline void clear() { data.clear(); }
    };
}