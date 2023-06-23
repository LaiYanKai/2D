#include "types.hpp"
#include "ray.hpp"
#include "unit.hpp"
#include "node.hpp"

#include <list>

#pragma once
namespace P2D::R2
{
    std::string get_addr(const Query *const &q);

    // owns batch tree, if any
    class Query
    {
    public:
#if P2D_DEBUG
        static inline int _cnt = 0;
#endif
        Query *next = nullptr, *prev = nullptr, *next_open = nullptr, *prev_open = nullptr;
        Unit *u_src = nullptr, *u_tgt = nullptr;
        float_t f = INF;
        QueryType type;
        Query(const QueryType &type, const float_t &f, Unit *const &u_src, Unit *const &u_tgt,
              Query *const &prev, Query *const &next,
              Query *const &prev_open, Query *const &next_open)
            : next(next), prev(prev), next_open(next_open), prev_open(prev_open), u_src(u_src), u_tgt(u_tgt), f(f), type(type)
        {
            _instances<Query>(true);
            _dbg11("-- Created " << get_addr(this));
        }
        Query(const float_t &f) : f(f)
        {
            _instances<Query>(true);
            _dbg11("-- Created " << get_addr(this));
        }
        Query &operator=(const Query &) = delete; // Disallow copying
        Query(const Query &) = delete;
        ~Query()
        {
            _instances<Query>(false);
            _dbg11("-- Deleted " << get_addr(this));
        }

        Query *const &queryList(const ListDir &ldir) const { return ldir == ListDir::Back ? next : prev; }
        Query *&queryList(const ListDir &ldir) { return ldir == ListDir::Back ? next : prev; }
        Query *const &queryOpen(const ListDir &ldir) const { return ldir == ListDir::Back ? next_open : prev_open; }
        Query *&queryOpen(const ListDir &ldir) { return ldir == ListDir::Back ? next_open : prev_open; }
    };
    std::ostream &operator<<(std::ostream &out, const Query &query);
    std::ostream &operator<<(std::ostream &out, const Query *const &query);

    // ======================= QueriesContainer ============================

    // owns Query
    class QueriesContainer
    {
    private:
        Query *_head;

    public:
        QueriesContainer() : _head(nullptr){};
        QueriesContainer &operator=(const QueriesContainer &) = delete; // Disallow copying
        QueriesContainer(const QueriesContainer &) = delete;
        ~QueriesContainer() { this->clear(); }

        bool empty() const { return this->_head == nullptr; }

        Query *emplace(const QueryType &qtype, const float_t &f, Unit *const &u_src, Unit *const &u_tgt)
        {
            Query *q = new Query(qtype, f, u_src, u_tgt, nullptr, nullptr, nullptr, this->_head);

            // insert into list
            if (this->empty() == false)
            {
                assert(this->_head->prev == nullptr);
                this->_head->prev = q;
            }
            assert(q->prev == nullptr);
            q->next = this->_head;
            this->_head = q;

            return this->_head;
        }

        // deletes the Query
        void erase(Query *const &query)
        {
            Query *&q_prev = query->prev;
            Query *&q_next = query->next;

            if (q_prev == nullptr)
            {
                assert(this->_head == query);
                this->_head = q_next;
            }
            else
                q_prev->next = q_next;

            if (q_next != nullptr)
                q_next->prev = q_prev;

            delete query;
        }

        void clear()
        {
            while (this->empty() == false)
            {
                Query *q_n = this->_head->next;
                delete this->_head;
                this->_head = q_n;
            }
        }
    };

}