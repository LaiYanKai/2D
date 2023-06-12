#include <forward_list>
#include "P2D/P2D.hpp"

#pragma once
namespace P2D::VG2
{
    struct Node;
    struct Corner
    {
        std::vector<Corner *> neighbors;
        V2 coord = {0, 0};
        Node *node = nullptr;
        mapkey_t key = 0;

        Corner(const mapkey_t &key, const V2 &coord) : neighbors(), coord(coord), key(key){};
    };

    struct Node
    {
        Corner *const crn = nullptr;
        Node *parent = nullptr, *openlist_next = nullptr, *openlist_prev = nullptr;
        float_t f = INF, g = INF, h = INF;
        bool is_visited = false;
        Node(Corner *const &crn, Node *const &parent, const float_t &g, const float_t &h) : crn(crn), parent(parent), f(g + h), g(g), h(h) {}
    };

    class Nodes
    {
    private:
        std::forward_list<Node> _data;

    public:
        inline Node *emplace(Corner *const &crn, Node *const &parent, const float_t &g, const float_t &h)
        {
            Node &node = _data.emplace_front(crn, parent, g, h);
            node.crn->node = &node;
            return &node;
        }
        inline void clear()
        { // remove node from corner
            for (auto &node_ : _data)
                node_.crn->node = nullptr;
            _data.clear();
        }
        inline bool empty() const { return _data.empty(); }
    };
}