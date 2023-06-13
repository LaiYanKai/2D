#include <vector>
#include <forward_list>
#include "P2D/P2D.hpp"

#pragma once
namespace P2D::TS2
{
    struct Node
    {
        const V2 coord = {0, 0};
        Node *parent = nullptr, *openlist_next = nullptr, *openlist_prev = nullptr;
        float_t f = INF, g = INF, h = INF;
        mapkey_t key = 0;
        Node(const mapkey_t &key, const V2 &coord, Node *const &parent, const float_t &g, const float_t &h) : coord(coord), parent(parent), f(g + h), g(g), h(h), key(key) {}
    };

    class Nodes
    {
    private:
        std::vector<Node *> _data;
        std::forward_list<Node> _owner;

    public:
        inline void setup(const size_t &num_nodes) { _data.resize(num_nodes, nullptr); }
        // read-only. Use emplace to write
        Node *const &operator[](const mapkey_t &key) { return _data[key]; }
        inline Node *const &emplace(const mapkey_t &key, const V2 &coord, Node *const &parent, const float_t &g, const float_t &h)
        {
            _data[key] = &(_owner.emplace_front(key, coord, parent, g, h));
            return _data[key];
        }
        // does not resize the array
        inline void erase()
        { // remove node from corner
            for (const Node &node : _owner)
                _data[node.key] = nullptr;
            _owner.clear();
        }
        inline bool empty() const { return _data.empty(); }
    };
}