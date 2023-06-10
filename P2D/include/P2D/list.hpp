
#include "types.hpp"

#pragma once
namespace P2D
{

    template <typename T> // T must have pointers "next" and "prev" that points to other T elements in the list. Both should be initialised to nullptr
    class AbstractList
    {
    private:
        T *_front = nullptr, *_back = nullptr;
        size_t _size = 0;

    public:
        // returns the pointer to the front (ldir == Prev) or back (ldir == Next) element
        static T *&lend(AbstractList<T> &ctn, const ListDir &ldir) { return ldir == ListDir::Prev ? ctn.front() : ctn.back(); }
        // returns the pointer to next or prev element
        static T *&eend(T *const &ele, const ListDir &ldir) { return ldir == ListDir::Prev ? ele->prev : ele->next; }

        AbstractList() : _front(nullptr), _back(nullptr), _size(0) {}
        AbstractList &operator=(const AbstractList &) = delete; // Disallow copying
        AbstractList(const AbstractList &) = delete;
        ~AbstractList() { this->clear(Erasure::Del); }

        inline size_t size() const { return this->_size; }
        inline T *const &front() const { return this->_front; }
        inline T *const &back() const { return this->_back; }

        // returns true if the container is empty
        inline bool empty() const
        {
            assert((this->_front == nullptr) == (this->_back == nullptr));
            assert((this->_front == nullptr) == (this->_size == 0));
            return this->_front == nullptr;
        }

        // returns true if ele exists in the container
        inline bool has(T *const &ele) const
        {
            T *e = this->_front;
            for (T *e = this->_front; e != nullptr; e = e->next)
                if (e == ele)
                    return true;
            return false;
        }

        // deletes all elements set size to zero.
        void clear(const Erasure &estate = Erasure::Del)
        {
            if (estate == Erasure::Del)
            {
                while (this->_front != nullptr)
                {
                    T *ele = this->_front->next;
                    delete this->_front;
                    this->_front = ele;
                }
            }
            else
                this->_front = nullptr;
            this->_back = nullptr;
            this->_size = 0;
        }

        // erases ele from the list, deleting it if erase == Erasure::Del
        void erase(T *const &ele, const Erasure &erase = Erasure::Del)
        {
            assert(this->inContainer(ele));

            for (const ListDir &ldir : {ListDir::Prev, ListDir::Next})
            {
                T *ele_fwd = eend(ele, ldir);
                T *ele_rev = eend(ele, !ldir);
                T *&ele_end = lend(*this, ldir);

                if (ele_fwd != nullptr)
                { // adjust the sibling's pointers if the sibling exists
                    assert(ele != ele_end);
                    eend(ele_fwd, !ldir) = ele_rev;
                }
                else
                { // adjust the list end if the sibling doesn't exist (the ele is the list end)
                    assert(ele == ele_end);
                    ele_end = ele_rev;
                }
            }

            // adjust the current points
            ele->prev = nullptr;
            ele->next = nullptr;

            --this->_size;

            if (erase == Erasure::Del)
                delete ele;
        }

        // inserts to front (ldir == Prev) or back (ldir == Next) of container.
        void insert(const ListDir &ldir, T *const &ele)
        {
            if (this->empty())
            {
                this->_front = ele;
                this->_back = ele;
                assert(ele->prev == nullptr); // avoid ele from accidentally deleting other elements. Use splice if moving from one list to another.
                assert(ele->next == nullptr); // avoid ele from accidentally deleting other elements. Use splice if moving from one list to another.
                ele->prev = nullptr;
                ele->next = nullptr;
            }
            else
            {
                T *ele_end = lend(*this, ldir);
                eend(ele, !ldir) = ele_end;
                eend(ele, ldir) = nullptr;
                eend(ele_end, ldir) = ele;
                lend(*this, ldir) = ele;
            }
            ++this->_size;
        }

        inline T *pop(const ListDir &ldir)
        {
            if (this->empty())
                return nullptr;

            T *&ele_end = lend(*this, ldir);
            T *ele = ele_end;
            T *ele_rev = eend(ele, !ldir);

            ele_end = ele_rev;
            assert(eend(ele_rev, ldir) == ele);
            eend(ele_rev, ldir) = nullptr;

            assert(eend(ele, ldir) == nullptr);
            eend(ele, !ldir) = nullptr;

            --this->_size;
            return ele;
        }

        // moves all ele in ele_ctn to this container, at ldir of this container.
        // returns the !ldir ele end of ele_ctn. This ele is moved into this container.
        T *splice(const ListDir &ldir, AbstractList &ele_ctn)
        {
            assert(&ele_ctn != this);
            if (ele_ctn.empty())
                return nullptr;

            T *ele_ctn_fwd = lend(ele_ctn, ldir);
            T *ele_ctn_rev = lend(ele_ctn, !ldir);

            this->_size += ele_ctn.size();
            ele_ctn.clear(Erasure::Keep);
            if (this->empty())
            {
                lend(*this, ldir) = ele_ctn_fwd;
                lend(*this, !ldir) = ele_ctn_rev;
            }
            else
            {
                T *ele_end = lend(*this, ldir);

                assert(eend(ele_end, ldir) == nullptr);
                eend(ele_end, ldir) = ele_ctn_rev;

                assert(eend(ele_ctn_rev, !ldir) == nullptr);
                eend(ele_ctn_rev, !ldir) = ele_end;

                assert(lend(*this, ldir) == nullptr);
                lend(*this, ldir) = ele_ctn_fwd;
            }
            return ele_ctn_rev;
        }

        // moves ele in ele_ctn to this container, at ldir of this container.
        void splice(const ListDir &ldir, AbstractList &ele_ctn, T *const &ele)
        {
            ele_ctn.erase(ele, Erasure::Keep);
            this->insert(ldir, ele);
        }
    };
}