#include "R2/unit.hpp"
#include "R2/node.hpp"

namespace P2D::R2
{
    Corner *const &Unit::crn() const { return node->crn; }

    void Unit::erase(const EraseState &estate) { this->node->units.erase(this, estate); }

    Unit *Unit::unzip(const UnitType &utype, const std::vector<Unit *> &srcs, const std::vector<Unit *> &tgts,
                      Ray *const &ray_left, Ray *const &ray_right, const float_t &cost_src, const float_t &cost_tgt)
    {
        return this->node->units.emplace(this->side_tgt, utype, srcs, tgts, ray_left, ray_right, cost_src, cost_tgt);
    }
    Unit *Unit::unzipToSrc(const UnitType &utype, const std::vector<Unit *> &srcs, const std::vector<Unit *> &tgts, Ray *const &ray_left, Ray *const &ray_right, const float_t &cost_src)
    {
        return unzip(utype, srcs, tgts, ray_left, ray_right, cost_src, INF);
    }
    Unit *Unit::unzipToTgt(const UnitType &utype, const std::vector<Unit *> &srcs, const std::vector<Unit *> &tgts, Ray *const &ray_left, Ray *const &ray_right, const float_t &cost_tgt)
    {
        return unzip(utype, srcs, tgts, ray_left, ray_right, INF, cost_tgt);
    }

    // Creates a Unit and inserts it into this container, and links it to its srcs and tgts
    Unit *UnitsContainer::emplace(const Side &side_tgt, const UnitType &utype, const std::vector<Unit *> &srcs, const std::vector<Unit *> &tgts,
                                  Ray *const &ray_left, Ray *const &ray_right, const float_t &cost_src, const float_t &cost_tgt)
    {
        Unit *u = Unit::create(node, side_tgt, utype, srcs, tgts, cost_src, cost_tgt, ray_left, ray_right);
        this->insert(u);
        _dbg11("Node{" << node << "}");
        return u;
    }

    std::string Unit::repr(const int &type) const
    {
        std::stringstream ss;
        ss << this->type << this->side_tgt << ":" << this->crn();
        ss << " <";
        ss << get_addr(this);
        if (type != 0)
        {
            ss << ",";
            ss << this->srcs.size() << "," << this->tgts.size();
            ss << ">";
            ss << " Rays(";
            ss << (this->ray_left == nullptr ? "<     NA    >" : this->ray_left->repr(0));
            ss << (this->ray_right == nullptr ? "<     NA    >" : this->ray_right->repr(0));
            ss << ")";
        }
        else
        {
            ss << ">";
        }
        return ss.str();
    }

    std::string get_addr(const Unit *const &u) { return "UN" + P2D::get_addr((void *)u); }

    std::ostream &operator<<(std::ostream &out, Unit const &unit)
    {
        out << unit.repr(1);
        out << ", <g$";
        out << to_string(unit.cost_src);
        out << ", h$";
        out << to_string(unit.cost_tgt);
        out << ">";

        for (const TreeDir &tdir : {TreeDir::Src, TreeDir::Tgt})
        {
            if (tdir == Src)
                out << ", Srcs[";
            else
                out << ", Tgts[";

            out << unit.rels(tdir).size() << "]{";

            for (Unit *u_rel : unit.rels(tdir))
            {
                if (u_rel == nullptr)
                    out << "     NA     ; ";
                else
                    out << u_rel->repr(0) << "; ";
            }
            out << "}";
        }
        return out;
    }
    std::ostream &operator<<(std::ostream &out, const Unit *const &unit)
    {
        if (unit == nullptr)
            out << "      UnitNA      ";
        else
            out << *unit;
        return out;
    }

    void UnitsContainer::clear()
    {
        while (this->_front != nullptr)
        {
            assert(&(this->_front->node->units) == this);
            Unit *unit_n = this->_front->next;
            delete this->_front;
            this->_front = unit_n;
        }
        this->_size = 0;
    }
    void UnitsContainer::erase(Unit *const &unit, const EraseState &estate)
    {
        assert(&(unit->node->units) == this); // unit must belong to this container
        assert(this->inContainer(unit));

        if (this->_front == unit)
        {
            assert(unit->prev == nullptr);
            this->_front = unit->next;
        }
        else
        {
            assert(unit->prev != nullptr);
            unit->prev->next = unit->next;
        }

        if (unit->next != nullptr)
            unit->next->prev = unit->prev;

        unit->prev = nullptr; // unnecessary, but for completeness
        unit->next = nullptr; // unnecessary, but for completeness

        if (estate == EraseState::Del)
        {
            // _dbg11("-- Deleted U{" << unit->repr(1) << "}");
            delete unit;
        }

        --this->_size;
    }
}
