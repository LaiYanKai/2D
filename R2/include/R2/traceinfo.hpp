#include "types.hpp"
#include "unit.hpp"

#pragma once
namespace P2D::R2
{
    class TraceInfo;
    std::string get_addr(const TraceInfo *const &ti);

    enum TraceState
    {
        Normal,
        Edge,
        Encountered,
        Reverse,
    };

    class TraceInfo
    {
    public:
#if P2D_DEBUG
        static inline int _cnt = 0;
#endif
        V2 v_curFromRel, v_prog;
        Unit *u;
        Unit *u_spec = nullptr;
        TraceInfo *next, *prev;
        int cnt_prog = 0;
        TraceState state = TraceState::Normal;

        TraceInfo(Unit *const &u, const V2 &v_curFromRel, const V2 &v_prog, TraceInfo *const &next, TraceInfo *const &prev)
            : v_curFromRel(v_curFromRel), v_prog(v_prog), u(u), next(next), prev(prev)
        {
            _instances<TraceInfo>(true);
            _dbg11("-- Create " << get_addr(this));
        }
        ~TraceInfo()
        {
            _instances<TraceInfo>(false);
            _dbg11("-- Delete " << get_addr(this));
        }
        std::string repr(const int &type) const;
        inline TraceInfo *&list(const ListDir &ldir) { return ldir == ListDir::Front ? this->prev : this->next; }
    };
    std::ostream &operator<<(std::ostream &out, TraceInfo const &ti);
    std::ostream &operator<<(std::ostream &out, const TraceInfo *const &ti);

    class TraceInfoContainer
    {
    private:
        TraceInfo *_front, *_back;
        size_t _size = 0;

    public:
        TraceInfoContainer() : _front(nullptr), _back(nullptr), _size(0)
        {
            static_assert(ListDir::Front == 0 && ListDir::Back == 1);
        }
        TraceInfoContainer &operator=(const TraceInfoContainer &) = delete; // Disallow copying
        TraceInfoContainer(const TraceInfoContainer &) = delete;
        ~TraceInfoContainer() { this->clear(EraseState::Del); }

        inline size_t size() const { return this->_size; }
        inline TraceInfo *const &front() const { return this->_front; }
        inline TraceInfo *const &back() const { return this->_back; }
        inline TraceInfo *&listEnd(const ListDir &ldir) { return ldir == ListDir::Front ? this->_front : this->_back; }

        inline bool empty() const
        {
            assert((this->_front == nullptr) == (this->_back == nullptr));
            assert((this->_front == nullptr) == (this->_size == 0));
            return this->_front == nullptr;
        }

        // erases this unit from unit_next, unit_prev and unit_container.
        // does not set the unit_next and prev to nullptr
        void erase(TraceInfo *const &ti, const EraseState &estate = EraseState::Del)
        {
            assert(this->inContainer(ti));

            for (const ListDir &ldir : {ListDir::Front, ListDir::Back})
            {
                TraceInfo *ti_fwd = ti->list(ldir);
                TraceInfo *ti_rev = ti->list(!ldir);
                TraceInfo *&ti_end = this->listEnd(ldir);

                if (ti_fwd != nullptr)
                {
                    assert(ti != ti_end);
                    ti_fwd->list(!ldir) = ti_rev;
                }
                else
                {
                    assert(ti == ti_end);
                    ti_end = ti_rev;
                }
            }

            ti->prev = nullptr;
            ti->next = nullptr;

            --this->_size;

            if (estate == EraseState::Del)
                delete ti;
        }

        // deletes all units and set size to zero.
        void clear(const EraseState &estate = EraseState::Del)
        {
            if (estate == EraseState::Del)
            {
                while (this->_front != nullptr)
                {
                    TraceInfo *ti = this->_front->next;
                    delete this->_front;
                    this->_front = ti;
                }
            }
            else
                this->_front = nullptr;
            this->_back = nullptr;

            this->_size = 0;
        }

        // inserts to front of list. Modifies next and prev
        void insert(const ListDir &ldir, TraceInfo *const &ti)
        {
            if (this->empty())
            {
                this->_front = ti;
                this->_back = ti;
                assert(ti->prev == nullptr);
                assert(ti->next == nullptr);
            }
            else
            {
                TraceInfo *ti_end = this->listEnd(ldir);
                ti->list(!ldir) = ti_end;
                ti->list(ldir) = nullptr;
                assert(ti_end->list(ldir) == nullptr);
                ti_end->list(ldir) = ti;
                this->listEnd(ldir) = ti;
            }
            ++this->_size;
        }

        // Creates a TraceInfo and inserts it into this container
        inline TraceInfo *emplace(const ListDir &ldir, Unit *const &u, const V2 &v_curFromRel, const V2 &v_prog)
        {
            TraceInfo *ti = new TraceInfo(u, v_curFromRel, v_prog, nullptr, nullptr);
            this->insert(ldir, ti);
            return ti;
        }

        inline TraceInfo *pop(const ListDir &ldir)
        {
            if (this->empty())
                return nullptr;

            TraceInfo *&ti_end = this->listEnd(ldir);
            TraceInfo *ti = ti_end;
            TraceInfo *ti_rev = ti->list(!ldir);

            assert(ti->list(ldir) == nullptr);

            ti_end = ti_rev;
            ti->list(!ldir) = nullptr;

            assert(ti_rev->list(ldir) == ti);
            ti_rev->list(ldir) = nullptr;

            --this->_size;

            return ti;
        }

        // moves all ti in ti_ctn to this container, at ldir of this container.
        // returns the !ldir ti end of ti_ctn. This ti is moved into this container.
        TraceInfo *splice(const ListDir &ldir, TraceInfoContainer &ti_ctn)
        {
            assert(&ti_ctn != this);
            if (ti_ctn.empty())
                return nullptr;

            TraceInfo *ti_ctn_rev = ti_ctn.listEnd(!ldir);
            TraceInfo *ti_ctn_fwd = ti_ctn.listEnd(ldir);

            this->_size += ti_ctn.size();
            ti_ctn.clear(EraseState::Keep);
            TraceInfo *ti_end = this->listEnd(ldir);

            assert(ti_end->list(ldir) == nullptr);
            ti_end->list(ldir) = ti_ctn_rev;

            assert(ti_ctn_rev->list(!ldir) == nullptr);
            ti_ctn_rev->list(!ldir) = ti_end;

            this->listEnd(ldir) = ti_ctn_fwd;
            assert(ti_ctn_fwd->list(ldir) == nullptr);

            return ti_ctn_rev;
        }

        // moves all ti in ti_ctn to this container, at ldir of this container.
        void splice(const ListDir &ldir, TraceInfoContainer &ti_ctn, TraceInfo *const &ti)
        {
            ti_ctn.erase(ti, EraseState::Keep);
            this->insert(ldir, ti);
        }

        bool inContainer(TraceInfo *const &ti)
        {
            TraceInfo *_ti = this->_front;
            while (_ti != nullptr)
            {
                TraceInfo *_ti_next = _ti->next;
                if (_ti == ti)
                    return true;
                _ti = _ti_next;
            }
            return false;
        }
    };
}
