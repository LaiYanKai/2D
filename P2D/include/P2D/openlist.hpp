#include "types.hpp"

#pragma once
namespace P2D
{
    // doubly linked list  that does not own T* objects.
    // T* must have members "openlist_prev" and "openlist_next", and "f", initialised to nullptrs
    template <typename T>
    class OpenList
    {
    private:
        T *_front = nullptr;

    public:
        OpenList() : _front(nullptr) {}
        OpenList &operator=(const OpenList &) = delete; // Disallow copying
        OpenList(const OpenList &) = delete;
        ~OpenList() {}

        bool empty() const { return this->_front == nullptr; }
        void clear() { this->_front = nullptr; }

        // Removes the first element in the open list, returns nullptr if none.
        T *poll()
        {
            T *ele = this->_front;
            if (this->empty() == false)
            {
                assert(ele->openlist_prev == nullptr);
                T *ele_next = ele->openlist_next;
                if (ele_next != nullptr)
                {
                    assert(ele_next->openlist_prev == ele);
                    ele_next->openlist_prev = nullptr;
                    ele->openlist_next = nullptr; // required for unqueue checks
                }
                this->_front = ele_next;
            }
            return ele;
        }

        // Removes the element from the openlist. Undefined behavior if element does not belong to the openlist
        void unqueue(T *const &ele)
        {
            T *&ele_prev = ele->openlist_prev;
            T *&ele_next = ele->openlist_next;
            if (ele_prev == nullptr && ele_next == nullptr && this->_front != ele) // not queued before
                return;

            if (this->_front == ele)
            {
                assert(ele_prev == nullptr);
                this->_front = ele_next;
            }
            else
            {
                assert(ele_prev != nullptr);
                ele_prev->openlist_next = ele_next;
            }

            if (ele_next != nullptr)
                ele_next->openlist_prev = ele_prev;

            // for debug purposes
            ele->openlist_prev = nullptr;
            ele->openlist_next = nullptr;
        }

        // Queues element by insert sort
        void queue(T *const &ele)
        {
            assert(ele->openlist_next == nullptr && ele->openlist_prev == nullptr); // must not already be queued
            assert(ele->f < INF);

            T *ele_prev = nullptr;
            T *ele_next = this->_front;
            while (ele_next != nullptr && approxGe(ele->f, ele_next->f) == true) // ele->f >= ele_next->f
            {
                ele_prev = ele_next;
                ele_next = ele_next->openlist_next;
            }

            // join ele_prev and ele
            if (ele_prev != nullptr)
                ele_prev->openlist_next = ele;
            else
                this->_front = ele;
            ele->openlist_prev = ele_prev;

            // join ele_next and ele
            if (ele_next != nullptr)
                ele_next->openlist_prev = ele;
            ele->openlist_next = ele_next;
        }
    };
}