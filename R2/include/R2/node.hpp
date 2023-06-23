#include "P2D/P2D.hpp"
#include "types.hpp"
#include "corner.hpp"
#include "unit.hpp"
#include <vector>
#include <array>
#include <assert.h>
#include <unordered_map>
#include <forward_list>

#pragma once
namespace P2D::R2
{
    class Query;
    class Node
    {
    public:
        UnitsContainer units;
        // rays that have los from this node.
        std::vector<Ray *> rays;
        Corner *const crn;
        float_t cost_src_min = INF;

#if P2D_DEBUG
        static inline int _cnt = 0;
#endif

        Node(Corner *const &crn) : units(this), crn(crn)
        {
            _instances<Node>(true);
        }
        ~Node()
        {
            _instances<Node>(false);
            units.clear();
            crn->node = nullptr; // crn does not own node
        }
        const V2 &coord() const { return crn->coord; }

        std::string repr(const int &type = 0) const;
    };
    std::string get_addr(const Node *const &n);
    std::ostream &operator<<(std::ostream &out, const Node &node);
    std::ostream &operator<<(std::ostream &out, const Node *const &node);

    class Nodes
    {
    private:
        // std::unordered_map<crnkey_t, Node> _data;
        std::forward_list<Node> _data;

    public:
        inline Node *emplace(Corner *const &crn)
        {
            assert(crn->node == nullptr);
            return &(this->_data.emplace_front(crn));
        }
        inline void clear() { this->_data.clear(); }
    };
}