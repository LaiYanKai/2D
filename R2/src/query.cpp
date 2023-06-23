#include "R2/query.hpp"
#include "R2/types.hpp"

namespace P2D::R2
{
    std::string get_addr(const Query *const &q) { return "QR" + P2D::get_addr((void *)q); }

    std::ostream &operator<<(std::ostream &out, const Query &query)
    {

        out << "<";
        out << query.type;
        out << ", ";
        out << get_addr(&query);
        out << ", f$" << to_string(query.f);
        out << ">";
        out << " USrc( ";
        out << query.u_src->repr(1);
        out << " ), UTgt( ";
        out << query.u_tgt->repr(1);
        out << ")";

        return out;
    }

    std::ostream &operator<<(std::ostream &out, const Query *const &query)
    {
        out << *query;
        return out;
    }
}