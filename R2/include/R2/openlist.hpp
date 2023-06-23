#include "P2D/P2D.hpp"
#include "types.hpp"
#include "query.hpp"
#include <list>
#include <vector>

#pragma once
namespace P2D::R2
{
    class OpenList
    {
    private:
        Query *_head;

    public:
        OpenList() : _head(nullptr) {}
        OpenList &operator=(const OpenList &) = delete; // Disallow copying
        OpenList(const OpenList &) = delete;

        inline bool empty() const { return this->_head == nullptr; }

        // Use only when queries are deleted by QueriesContainer, as this method simply sets open_head to nullptr.
        // Does not delete any queries, and does not modify pointers in Query's queryOpen().
        inline void clear() { this->_head = nullptr; }

        // removes query from the list and returns it. nullptr is returned if there is nothing in the list (head is nullptr)
        Query *poll()
        {
            Query *q = this->_head;
            if (this->empty() == false)
            {
                assert(q->prev_open == nullptr);
                Query *q_next = q->next_open;
                if (q_next != nullptr)
                {
                    assert(q_next->prev_open == q);
                    q_next->prev_open = nullptr;
                    q->next_open = nullptr; // required for unqueue checks
                }
                this->_head = q_next;
            }
            return q;
        }
        // queues or requeues using query->f and insert sort (from the start).
        void unqueue(Query *const &query)
        {
            Query *&q_prev = query->prev_open;
            Query *&q_next = query->next_open;
            if (q_prev == nullptr && q_next == nullptr && this->_head != query)
            {   // not queued before
                return;
            }
            if (this->_head == query)
            {
                assert(q_prev == nullptr);
                this->_head = q_next;
            }
            else
            {
                assert(q_prev != nullptr);
                q_prev->next_open = q_next;
            }

            if (q_next != nullptr)
                q_next->prev_open = q_prev;

            // for debug purposes
            query->prev_open = nullptr;
            query->next_open = nullptr;
        }
        void queue(Query *const &query)
        {
            assert(query->f < INF);

            Query *q_prev = nullptr;
            Query *q_next = this->_head;
            while (q_next != nullptr && approxGe(query->f, q_next->f)) // query->f >= q_next->f
            {
                q_prev = q_next;
                q_next = q_next->next_open;
            }

            // join q_prev and query
            if (q_prev != nullptr)
                q_prev->next_open = query;
            else
                this->_head = query;
            query->prev_open = q_prev;

            // join q_next and query
            if (q_next != nullptr)
                q_next->prev_open = query;
            query->next_open = q_next;

            _dbg11("Queued : { " << query << " }");
        }
    };

}