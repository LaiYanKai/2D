#include <vector>
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
        bool is_visited = false;
        Node(const mapkey_t &key, const V2 &coord, Node *const &parent, const float_t &g, const float_t &h) : coord(coord), parent(parent), f(g + h), g(g), h(h), key(key) {}
    };

    class Nodes
    {
    private:
        std::vector<Node *> _data;

    public:
        inline void resize(const size_t &num_nodes) { _data.resize(num_nodes, nullptr); }
        inline Node *const &emplace(const mapkey_t &key, const V2 &coord, Node *const &parent, const float_t &g, const float_t &h)
        {
            if (_data[key] == nullptr)
                _data[key] = new Node(key, coord, parent, g, h);
            return _data[key];
        }
        // does not resize the array
        inline void erase()
        { // remove node from corner
            for (Node *&node : _data)
            {
                if (node != nullptr)
                    delete node;
                node = nullptr;
            }
        }
        inline bool empty() const { return _data.empty(); }
    };
}