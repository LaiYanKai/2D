#include "R2/traceinfo.hpp"

namespace P2D::R2
{

    std::string TraceInfo::repr(const int &type) const
    {
        std::stringstream ss;
        if (type != 0)
        {
            ss << "<cfr:";
            ss << this->v_curFromRel;
            ss << ",";
            ss << "prog:";
            ss << this->v_prog;
            ss << "> U{ ";
            ss << this->u;
            ss << " }";
        }
        return ss.str();
    }

    std::string get_addr(const TraceInfo *const &ti) { return "TI" + P2D::get_addr((void *)ti); }

    std::ostream &operator<<(std::ostream &out, TraceInfo const &ti)
    {
        out << ti.repr(1);
        return out;
    }
    std::ostream &operator<<(std::ostream &out, const TraceInfo *const &ti)
    {
        if (ti == nullptr)
            out << "      TraceInfoNA      ";
        else
            out << *ti;
        return out;
    }

}
