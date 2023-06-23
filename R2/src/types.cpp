#include "R2/types.hpp"
namespace P2D::R2
{
    TreeDir operator!(const TreeDir &tdir) { return tdir == TreeDir::Src ? TreeDir::Tgt : TreeDir::Src; }
    ListDir operator!(const ListDir &ldir) { return ldir == ListDir::Back ? ListDir::Front : ListDir::Back; }

    std::ostream &operator<<(std::ostream &out, const Vis &vis)
    {
        switch (vis)
        {
        case Vis::Unknown:
            out << "U";
            break;
        case Vis::No:
            out << "N";
            break;
        case Vis::Yes:
            out << "Y";
            break;
        default:
            out << "?";
            break;
        }
        return out;
    }
    std::ostream &operator<<(std::ostream &out, const TreeDir &tdir)
    {
        switch (tdir)
        {
        case TreeDir::Src:
            out << "S";
            break;
        case TreeDir::Tgt:
            out << "T";
            break;
        default:
            out << "?";
            break;
        }
        return out;
    }
    std::ostream &operator<<(std::ostream &out, const QueryType &qtype)
    {
        switch (qtype)
        {
        case QueryType::Cast:
            out << "QC";
            break;
        case QueryType::Trace:
            out << "QT";
            break;
        default:
            out << "Q?";
            break;
        }
        return out;
    }
    std::ostream &operator<<(std::ostream &out, const UnitType &utype)
    {
        switch (utype)
        {
        case UnitType::PS:
            out << "Ps";
            break;
        case UnitType::EU:
            out << "Eu";
            break;
        case UnitType::EY:
            out << "Ey";
            break;
        case UnitType::SU:
            out << "Su";
            break;
        case UnitType::SY:
            out << "Sy";
            break;
        case UnitType::TU:
            out << "Tu";
            break;
        case UnitType::TY:
            out << "Ty";
            break;
        case UnitType::TD:
            out << "Td";
            break;
        case UnitType::TR:
            out << "Tr";
            break;
        default:
            out << "??";
            break;
        }
        return out;
    }
}