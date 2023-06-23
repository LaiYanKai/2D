#include "types.hpp"
#include "corner.hpp"
#include "ray.hpp"
#include <array>

#pragma once
namespace P2D::R2
{
    class Unit;
    std::string get_addr(const Unit *const &u);

    class Node;
    class Query;
    class Unit
    {
    public:
        std::vector<Unit *> srcs = {}, tgts = {};
        std::vector<Query *> queries = {};
        Ray *ray_left = nullptr, *ray_right = nullptr;
        Node *const node;
        Unit *prev = nullptr, *next = nullptr;
        float_t cost_src = INF, cost_tgt = INF;
        UnitType type;
        const Side side_tgt;

        // creates a new Unit and links to srcs and tgts. Does not belong to an owning container.
        static inline Unit *create(Node *const &node, const Side &side_tgt, const UnitType &utype,
                                   const std::vector<Unit *> &srcs, const std::vector<Unit *> &tgts,
                                   const float_t &cost_src, const float_t &cost_tgt,
                                   Ray *const &ray_left, Ray *const &ray_right)
        {
            Unit *u = new Unit(srcs, tgts, ray_left, ray_right, node, nullptr, nullptr, cost_src, cost_tgt, utype, side_tgt);
            for (auto &src : srcs)
                if (src != nullptr)
                    src->addTgt(u);
            for (auto &tgt : tgts)
                if (tgt != nullptr)
                    tgt->addSrc(u);
            return u;
        }

        // creates Src Type and links srcs and tgts
        static inline Unit *createSrcType(Node *const &node, const Side &side_tgt, const UnitType &utype,
                                          const std::vector<Unit *> &srcs, const std::vector<Unit *> &tgts,
                                          const float_t &cost_src, Ray *const &ray_left, Ray *const &ray_right)
        {
            return Unit::create(node, side_tgt, utype, srcs, tgts, cost_src, INF, ray_left, ray_right);
        }

        // creates Tgt type and links srcs and tgts
        static inline Unit *createTgtType(Node *const &node, const Side &side_tgt, const UnitType &utype,
                                          const std::vector<Unit *> &srcs, const std::vector<Unit *> &tgts,
                                          const float_t &cost_tgt, Ray *const &ray_left, Ray *const &ray_right)
        {
            return Unit::create(node, side_tgt, utype, srcs, tgts, INF, cost_tgt, ray_left, ray_right);
        }

#if P2D_DEBUG
        static inline int _cnt = 0;
#endif
        Unit(const std::vector<Unit *> &srcs, const std::vector<Unit *> &tgts,
             Ray *const &ray_left, Ray *const &ray_right,
             Node *const &node, Unit *const &prev, Unit *const &next,
             const float_t &cost_src, const float_t &cost_tgt, const UnitType &type, const Side &side_tgt)
            : srcs(srcs), tgts(tgts),
              ray_left(ray_left), ray_right(ray_right), node(node), prev(prev), next(next),
              cost_src(cost_src), cost_tgt(cost_tgt), type(type), side_tgt(side_tgt)
        {
            _instances<Unit>(true);
            assert(node != nullptr);
            _dbg11("-- Create " << get_addr(this));
        }
        Unit(Node *const &node, const Side &side_tgt) : node(node), side_tgt(side_tgt)
        {
            _instances<Unit>(true);
            assert(node != nullptr);
            _dbg11("-- Create " << get_addr(this));
        }
        Unit &operator=(const Unit &) = delete; // Disallow copying
        Unit(const Unit &) = delete;
        ~Unit()
        {
            _instances<Unit>(false);
            _dbg11("-- Delete " << get_addr(this));
        }

        // contains assert statements, does not return anything
        inline void isValid()
        {
#if P2D_DEBUG
            if (this->isS())
            {
                // if (this->crn()->di_occ == 0)
                // assert(this->isSY());
                // assert(this->srcs.size() >= 0);
                assert(this->srcs.empty() == false);
                assert(this->tgts.size() >= 1);
                assert(this->cost_src < INF);
                assert(this->cost_tgt == INF);
            }
            else if (this->isT())
            {
                if (this->crn()->di_occ == 0) // if goal
                {
                    assert(this->isTY());
                    assert(this->tgts.size() == 0);
                }
                else if (this->isTY())
                    assert(this->tgts.size() == 1);
                else
                    assert(this->tgts.size() >= 1); // PS and TU
                assert(this->srcs.size() >= 1);
                assert(this->cost_src == INF);
                assert(this->cost_tgt < INF);
            }
#endif
        }

        Corner *const &crn() const;
        const V2 &coord() const { return this->crn()->coord; }
        inline Side side(const TreeDir &tdir) const { return tdir == TreeDir::Tgt ? side_tgt : !side_tgt; }
        inline dir_idx_t edgeDi(const TreeDir &tdir) const { return this->crn()->edgeDi(this->side(tdir)); }
        inline V2 edgeVec(const TreeDir &tdir) const { return this->crn()->edgeVec(this->side(tdir)); }
        void erase(const EraseState &estate);

        inline Unit *const &unitList(const ListDir &ldir) const { return ldir == ListDir::Back ? this->next : this->prev; }
        inline Unit *&unitList(const ListDir &ldir) { return ldir == ListDir::Back ? this->next : this->prev; }

        inline const std::vector<Unit *> &rels(const TreeDir &tdir) const { return tdir == TreeDir::Src ? this->srcs : this->tgts; }
        inline std::vector<Unit *> &rels(const TreeDir &tdir) { return tdir == TreeDir::Src ? this->srcs : this->tgts; }

        inline const float_t &cost(const TreeDir &tdir) const { return tdir == TreeDir::Src ? this->cost_src : this->cost_tgt; }
        inline float_t &cost(const TreeDir &tdir) { return tdir == TreeDir::Src ? this->cost_src : this->cost_tgt; }

        inline Unit *const &rel(const TreeDir &tdir) const
        {
            assert(this->rels(tdir).size() == 1);
            return this->rels(tdir).front();
        }
        inline Unit *&rel(const TreeDir &tdir)
        {
            assert(this->rels(tdir).size() == 1);
            return this->rels(tdir).front();
        }
        inline Unit *const &src() const { return this->rel(Src); }
        inline Unit *&src() { return this->rel(Src); }
        inline Unit *const &tgt() const { return this->rel(Tgt); }
        inline Unit *&tgt() { return this->rel(Tgt); }

        inline size_t numRels(const TreeDir &tdir) const { return tdir == TreeDir::Src ? this->numSrcs() : this->numTgts(); }
        inline size_t numSrcs() const { return this->srcs.size(); }
        inline size_t numTgts() const { return this->tgts.size(); }

        inline bool isPS() const { return this->type == UnitType::PS; }
        inline bool isEU() const { return this->type == UnitType::EU; }
        inline bool isEY() const { return this->type == UnitType::EY; }
        inline bool isSU() const { return this->type == UnitType::SU; }
        inline bool isSY() const { return this->type == UnitType::SY; }
        inline bool isTU() const { return this->type == UnitType::TU; }
        inline bool isTY() const { return this->type == UnitType::TY; }
        inline bool isTD() const { return this->type == UnitType::TD; }
        inline bool isTR() const { return this->type == UnitType::TR; }
        inline bool isS() const { return this->isSU() || this->isSY(); }
        inline bool isT() const { return this->isTU() || this->isTY() || this->isTD(); }
        inline bool isE() const { return this->isEU() || this->isEY(); }
        inline bool isSrcU() const { return this->isSU() || this->isEU(); }
        inline bool isSrcY() const { return this->isSY() || this->isEY(); }

        inline Ray *const &ray(const Side &side) const { return side == Side::L ? ray_left : ray_right; }
        inline Ray *&ray(const Side &side) { return side == Side::L ? ray_left : ray_right; }

        // replaces a src (u_from) in srcs to u_to
        inline void replaceSrc(Unit *const &u_from, Unit *const &u_to) { this->replaceRel(Src, u_from, u_to); }
        // replaces a tgt(u_from in srcs to u_to)
        inline void replaceTgt(Unit *const &u_from, Unit *const &u_to) { this->replaceRel(Tgt, u_from, u_to); }
        // replaces a rel(u_from) with a new rel(u_to) in this unit's (tdir) rels
        inline void replaceRel(const TreeDir &tdir, Unit *const &u_from, Unit *const &u_to)
        {
            for (Unit *&u : this->rels(tdir))
            {
                if (u == u_from)
                {
                    u = u_to;
                    return;
                }
            }
            assert(false);
        }

        // replaces a src(u_from) with a new src(u_to) in this unit's src rels.
        // Then, unlinks the old src(u_from) from this unit by removing this unit from old src(u_from)'s Tgt rels
        // and links the new src(u_to) to this unit by adding this unit to new src(u_to)'s Tgt rels
        inline void replaceSrcDeep(Unit *const &u_from, Unit *const &u_to) { this->replaceRelDeep(Src, u_from, u_to); }
        // replaces a tgt(u_from) with a new tgt(u_to) in this unit's Src rels.
        // Then, unlinks the old tgt(u_from) from this unit by removing this unit from old tgt(u_from)'s Src rels
        // and links the new tgt(u_to) to this unit by adding this unit to new tgt(u_to)'s Src rels
        inline void replaceTgtDeep(Unit *const &u_from, Unit *const &u_to) { this->replaceRelDeep(Tgt, u_from, u_to); }
        // replaces a rel(u_from) with a new rel(u_to) in this unit's (tdir) rels.
        // Then, unlinks the old rel(u_from) from this unit by removing this unit from old rel(u_from)'s (!tdir) rels
        // and links the new rel(u_to) to this unit by adding this unit to new rel(u_to)'s (!tdir) rels
        inline void replaceRelDeep(const TreeDir &tdir, Unit *const &u_from, Unit *const &u_to)
        {
            assert(u_from != nullptr);
            assert(u_to != nullptr);

            this->replaceRel(tdir, u_from, u_to);
            u_from->removeRel(!tdir, this);
            u_to->addRel(!tdir, this);
        }

        // links by adding u_src into this unit's srcs, and this unit into u_srcs's tgts
        inline void linkSrc(Unit *const &u_src) { this->linkRel(Src, u_src); }
        // links by adding u_tgt into this unit's tgts, and this unit into u_tgt's srcs
        inline void linkTgt(Unit *const &u_tgt) { this->linkRel(Tgt, u_tgt); }
        // links by adding u_rel into this unit's (tdir) rels, and this unit into u_rel's (!tdir) rels
        inline void linkRel(const TreeDir &tdir, Unit *const &u_rel)
        {
            assert(u_rel != nullptr);
            assert(this->hasRel(tdir, u_rel) == false);
            assert(u_rel->hasRel(!tdir, this) == false);

            this->addRel(tdir, u_rel);
            u_rel->addRel(!tdir, this);
        }

        inline void linkSrcs(const std::vector<Unit *> &u_srcs) { this->linkRels(Src, u_srcs); }
        inline void linkTgts(const std::vector<Unit *> &u_tgts) { this->linkRels(Tgt, u_tgts); }
        inline void linkRels(const TreeDir &tdir, const std::vector<Unit *> &u_rels)
        {
            assert(&u_rels != &(this->rels(tdir))); // avoid invalidation when looping over u_rels
            for (Unit *const &u_rel : u_rels)
            {
                assert(&u_rels != &(u_rel->rels(!tdir)));   // avoid invalidation when looping over u_rels
                assert(this->hasRel(tdir, u_rel) == false); // rel not added to this before

                u_rel->addRel(!tdir, this); // add this to rel
            }

            auto &rels = this->rels(tdir);
            rels.insert(rels.end(), u_rels.begin(), u_rels.end());
        }

        // unlinks by removing this unit from u_src's tgts, and u_src from this unit's srcs
        inline void unlinkSrc(Unit *const &u_src) { this->unlinkRel(Src, u_src); }
        // unlinks by removing this unit from u_tgt's srcs, and u_tgt from this unit's tgts
        inline void unlinkTgt(Unit *const &u_tgt) { this->unlinkRel(Tgt, u_tgt); }
        // unlinks by removing this unit from u_rel's (!tdir) rels, and u_rel from this unit's (tdir) rels
        inline void unlinkRel(const TreeDir &tdir, Unit *const &u_rel)
        {
            assert(u_rel != nullptr);

            this->removeRel(tdir, u_rel);
            u_rel->removeRel(!tdir, this);
        }

        // One-sided unlinks u_src from this unit's srcs.
        inline void removeSrc(Unit *const &u_src) { this->removeRel(Src, u_src); }
        // One-sided unlinks u_tgt from this unit's tgts.
        inline void removeTgt(Unit *const &u_tgt) { this->removeRel(Tgt, u_tgt); }
        // One-sided unlinks u_rel from this unit's (tdir) rels.
        inline void removeRel(const TreeDir &tdir, Unit *const &u_rel)
        {
            auto &units_rel = this->rels(tdir);
            for (auto it_u_rel = units_rel.begin(); it_u_rel != units_rel.end(); ++it_u_rel)
            {
                if (*it_u_rel == u_rel)
                {
                    _dbg11("--- " << get_addr(this) << ": Removed " << tdir << " " << get_addr(u_rel));
                    units_rel.erase(it_u_rel);
                    return;
                }
            }
            assert(false); // not found
        }

        // returns true if u_rel exists in this unit's (tdir) rels
        inline bool hasRel(const TreeDir &tdir, Unit *const &u_rel) const
        {
            for (auto &u_ : this->rels(tdir))
                if (u_ == u_rel)
                    return true;
            return false;
        }
        // returns true if u_src exists in this unit's srcs
        inline bool hasSrc(Unit *const &u_src) const { return this->hasRel(Src, u_src); }
        // returns true if u_tgt exists in this unit's tgts
        inline bool hasTgt(Unit *const &u_tgt) const { return this->hasRel(Tgt, u_tgt); }

        inline bool hasSrc() const { return this->hasRel(TreeDir::Src); }
        inline bool hasTgt() const { return this->hasRel(TreeDir::Tgt); }
        inline bool hasRel(const TreeDir &tdir) const { return this->rels(tdir).empty() == false; }

        // One-sided link by adding u_src into this unit's srcs.
        inline void addSrc(Unit *const &u_src) { this->addRel(Src, u_src); }
        // One-sided link by adding u_tgt into this unit's tgts.
        inline void addTgt(Unit *const &u_tgt) { this->addRel(Tgt, u_tgt); }
        // One-sided link by adding u_rel into this unit's (tdir) rels.
        inline void addRel(const TreeDir &tdir, Unit *const &u_rel)
        {
            assert((u_rel != nullptr && this->hasRel(tdir, u_rel) == false) || u_rel == nullptr); // must not already point to u_rel, and u_rel is nullptr
            _dbg11("--- " << get_addr(this) << ": Added " << tdir << " " << get_addr(u_rel));
            this->rels(tdir).push_back(u_rel);
        }

        inline bool hasQuery(Query *const &q) const
        {
            for (Query *const &_q : this->queries)
                if (_q == q)
                    return true;
            return false;
        }
        // removes query from unit's queries. Does not remove unit from query
        inline void removeQuery(Query *const &q)
        {
            auto it_q = this->queries.begin();
            for (; *(it_q) != q; ++it_q)
                assert(it_q != this->queries.end()); // query must be found in unit's queries
            this->queries.erase(it_q);
        }

        // creates a copy of this unit (ignoring queries), and links srcs and tgts
        Unit *unzip(const UnitType &utype, const std::vector<Unit *> &srcs, const std::vector<Unit *> &tgts, Ray *const &ray_left, Ray *const &ray_right, const float_t &cost_src, const float_t &cost_tgt);
        // creates a copy of this unit (ignoring queries), and links srcs and tgts
        Unit *unzipToSrc(const UnitType &utype, const std::vector<Unit *> &srcs, const std::vector<Unit *> &tgts, Ray *const &ray_left, Ray *const &ray_right, const float_t &cost_src);
        // creates a copy of this unit (ignoring queries), and links srcs and tgts
        Unit *unzipToTgt(const UnitType &utype, const std::vector<Unit *> &srcs, const std::vector<Unit *> &tgts, Ray *const &ray_left, Ray *const &ray_right, const float_t &cost_tgt);

        // copies srcs and tgts
        inline void convToTgtType(const UnitType &utype, const float_t &cost_tgt)
        {
            this->cost_src = INF;
            this->cost_tgt = cost_tgt;
            this->type = utype;
            // this->ray_left = nullptr; // preserve
            // this->ray_right = nullptr; // preserve
        }
        inline void convToSrcType(const UnitType &utype, const float_t &cost_src, Ray *const &ray_left, Ray *const &ray_right)
        {
            this->cost_tgt = INF;
            this->cost_src = cost_src;
            this->type = utype;
            this->ray_left = ray_left;
            this->ray_right = ray_right;
        }

        // returns min cost from tdir rels. // does not set the min cost
        inline float_t findCostSrc() const { return findCost(TreeDir::Src); }
        inline float_t findCostTgt() const { return findCost(TreeDir::Tgt); }
        inline float_t findCost(const TreeDir &tdir) const
        {
            float_t cost_rel = INF;
            for (Unit *const &u_rel : rels(tdir))
            {
                if (u_rel == nullptr)
                    continue;
                float_t c = u_rel->cost(tdir) + norm(u_rel->coord(), coord());
                if (c < cost_rel)
                    cost_rel = c;
            }
            return cost_rel;
        }

        // compact notation
        std::string repr(const int &type = 0) const;
    };
    std::ostream &operator<<(std::ostream &out, Unit const &unit);
    std::ostream &operator<<(std::ostream &out, const Unit *const &unit);

    class UnitsContainer
    {
    private:
        Node *const node;
        Unit *_front;
        int _size;

    public:
        UnitsContainer(Node *const &node) : node(node), _front(nullptr), _size(0)
        {
            static_assert(ListDir::Front == 0 && ListDir::Back == 1);
        }
        UnitsContainer &operator=(const UnitsContainer &) = delete; // Disallow copying
        UnitsContainer(const UnitsContainer &) = delete;
        ~UnitsContainer() { this->clear(); }

        Unit *const &front() const { return this->_front; }
        inline const int &size() const { return this->_size; }
        inline bool empty() const
        {
            assert((this->_size == 0) == (this->_front == nullptr));
            return this->_size == 0;
        }

        // erases this unit from unit_next, unit_prev and unit_container.
        // does not set the unit_next and prev to nullptr
        void erase(Unit *const &unit, const EraseState &estate = EraseState::Del);

        // deletes all units and set size to zero.
        void clear();

        // inserts to front of list. Modifies next and prev, and DOES NOT MODIFY unit's parent and child.
        void insert(Unit *const &unit)
        {
            assert(unit->node == this->node);

            if (this->_front == nullptr)
            { // nothing in this container
                this->_front = unit;
                assert(unit->next == nullptr);
                assert(unit->prev == nullptr);
            }
            else
            {
                assert(this->_front != nullptr);
                unit->next = this->_front;
                unit->prev = nullptr;
                assert(this->_front->prev == nullptr);
                this->_front->prev = unit;
                this->_front = unit;
            }

            ++this->_size;
        }

        // Creates a Unit and inserts it into this container, and links it to its srcs and tgts
        Unit *emplace(const Side &side_tgt, const UnitType &utype, const std::vector<Unit *> &srcs, const std::vector<Unit *> &tgts,
                      Ray *const &ray_left, Ray *const &ray_right, const float_t &cost_src, const float_t &cost_tgt);

        // Creates a Src Type Unit, inserts it into this container, and links it to its srcs and tgts
        inline Unit *emplaceSrcType(const Side &side_tgt, const UnitType &utype, const std::vector<Unit *> &srcs, const std::vector<Unit *> &tgts,
                                    Ray *const &ray_left, Ray *const &ray_right, const float_t &cost_src)
        {
            Unit *u = Unit::createSrcType(this->node, side_tgt, utype, srcs, tgts, cost_src, ray_left, ray_right);
            this->insert(u);
            return u;
        }

        // Creates a Tgt Type Unit, inserts it into this container, and links it to its srcs and tgts
        inline Unit *emplaceTgtType(const Side &side_tgt, const UnitType &utype, const std::vector<Unit *> &srcs,
                                    const std::vector<Unit *> &tgts, Ray *const &ray_left, Ray *const &ray_right, const float_t &cost_tgt)
        {
            Unit *u = Unit::createTgtType(this->node, side_tgt, utype, srcs, tgts, cost_tgt, ray_left, ray_right);
            this->insert(u);
            return u;
        }
        bool inContainer(Unit *const &unit)
        {
            Unit *u = this->_front;
            while (u != nullptr)
            {
                Unit *u_next = u->next;
                if (u == unit)
                    return true;
                u = u_next;
            }
            return false;
        }
    };
}
