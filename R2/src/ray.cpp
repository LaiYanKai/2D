#include "R2/ray.hpp"

namespace P2D::R2
{

    Ray::Ray(Corner *const &crn_src, Corner *const &crn_tgt)
        : crn_src(crn_src), crn_tgt(crn_tgt), vec(crn_tgt->coord - crn_src->coord), length(vec.norm()),
          vis(Vis::Unknown), collision(nullptr)
    {
        static_assert(TreeDir::Src == 0 && TreeDir::Tgt == 1);
    }

    rkey_t getRayKey(const mapkey_t &src_mkey, const dir_idx_t &src_diocc, const mapkey_t &tgt_mkey, const dir_idx_t &tgt_diocc)
    {
        constexpr rkey_t MAPKEY_WIDTH = sizeof(mapkey_t) * 8;
        crnkey_t src_ckey = getCrnKey(src_mkey, src_diocc);
        crnkey_t tgt_ckey = getCrnKey(tgt_mkey, tgt_diocc);
        return (rkey_t(src_ckey) << MAPKEY_WIDTH) | rkey_t(tgt_ckey);
    }

    /* void getMapKeys(const bkey_t &lkey, mapkey_t &src_key, mapkey_t &tgt_key)
    {
        constexpr bkey_t MAPKEY_WIDTH = sizeof(mapkey_t) * 8;
        constexpr bkey_t MAPKEY_MASK = (bkey_t(1) << MAPKEY_WIDTH) - 1;
        tgt_key = lkey & MAPKEY_MASK;
        src_key = (lkey >> MAPKEY_WIDTH);
    } */

    std::string Ray::repr(const int type) const
    {
        std::stringstream ss;

        { // all types
            ss << "<";
            ss << get_addr(this);
            ss << ",";
            ss << vis;
            // ss << "|" << vec;
            ss << this->crn_src;
            ss << "; ";
            ss << this->crn_tgt;
            ss << ">";
        }

        if (type > 0)
        {
            ss << "CrnLR(";
            if (this->collision == nullptr)
                ss << "    NA    ";
            else
                ss << this->collision->crn_left;

            ss << "; ";
            if (this->collision == nullptr)
                ss << "    NA    ";
            else
                ss << this->collision->crn_right;
            ss << "), ";

            ss << "SrcTgt(";
            ss << this->crn_src;
            ss << "; ";
            ss << this->crn_tgt;
            ss << ")";
        }

        return ss.str();
    }

    std::string get_addr(const Ray *const &ray) { return "Ry" + P2D::get_addr((void *)ray); }

    std::ostream &operator<<(std::ostream &out, Ray const &ray)
    {
        out << ray.repr(1);
        return out;
    }
    std::ostream &operator<<(std::ostream &out, const Ray *const &ray)
    {
        if (ray == nullptr)
            out << "     RayNA     ";
        else
            out << *ray;
        return out;
    }
}