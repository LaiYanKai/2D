#include "R2/node.hpp"
#include "R2/query.hpp"

namespace P2D::R2
{
    // std::ostream &operator<<(std::ostream &out, const ProjInfo &pinfo)
    // {
    //     out << "VProj(" << pinfo.v_tgtFromCur << "), ";
    //     out << "CrnL(" << pinfo.crn_left << "), ";
    //     out << "CrnR(" << pinfo.crn_right << ")";
    //     return out;
    // }
    // std::ostream &operator<<(std::ostream &out, const ProjInfo *const &pinfo)
    // {
    //     out << *pinfo;
    //     return out;
    // }

    std::string Node::repr(const int &type) const
    {
        std::stringstream ss;
        if (type == 0)
        {
            ss << this->crn;
            ss << " <numUnits:";
            ss << this->units.size();
            ss << ", minG$";
            ss << to_string(this->cost_src_min);
            ss << ">";
        }
        return ss.str();
    }

    std::string get_addr(const Node *const &n) { return "ND" + P2D::get_addr((void *)n); }

    std::ostream &operator<<(std::ostream &out, Node const &node)
    {
        out << node.repr(0);
        return out;
    }
    std::ostream &operator<<(std::ostream &out, const Node *const &node)
    {
        out << *node;
        return out;
    }
}