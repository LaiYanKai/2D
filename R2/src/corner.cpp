#include "R2/corner.hpp"
namespace P2D::R2
{
    std::ostream &operator<<(std::ostream &out, const Corner &crn)
    {
        if (crn.di_occ == 0)
            out << "#";
        else
            out << (crn.is_convex ? "+" : "-"); // << int(crn.di_occ) ;

        // out << "'";
        out << crn.coord; // << "|" << std::setw(10) << std::setfill('0') << crn.mkey << std::setfill(' ');
        return out;
    }
    std::ostream &operator<<(std::ostream &out, const Corner *const &crn)
    {
        if (crn == nullptr)
            out << "    NA    ";
        else
            out << *crn;
        return out;
    }
}