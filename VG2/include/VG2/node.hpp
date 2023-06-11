#include <forward_list>
#include "P2D/P2D.hpp"

#pragma once
namespace P2D::VG2
{
    struct Corner
    {
        mapkey_t key = 0;
        V2 coord = {0, 0};
        std::vector<Corner *> neighbors;
        Corner(const mapkey_t &key, const V2 &coord) : key(key), coord(coord), neighbors(){};
    };

    struct Node
    {
        Corner *const crn = nullptr;
        Node *parent = nullptr, *openlist_next = nullptr, *openlist_prev = nullptr;
        float_t f = INF, g = INF, h = INF;
        Node(Corner *const &crn, Node *const &parent, const float_t &g, const float_t &h) : crn(crn), parent(parent), f(g + h), g(g), h(h) {}
    };

    class Nodes
    {
    private:
        std::forward_list<Node> _data;

    public:
        inline Node *emplace(Corner *const &crn, Node *const &parent, const float_t &g, const float_t &h) { return &(_data.emplace_front(crn, parent, g, h)); }
        inline void clear() { _data.clear(); }
        inline bool empty() const { return _data.empty(); }
    };
}