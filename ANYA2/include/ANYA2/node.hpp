#include <vector>
#include <unordered_map>
#include <forward_list>
#include "P2D/P2D.hpp"

#pragma once
namespace P2D::ANYA2
{
    enum class NodeType
    {
        Flat = 0,
        Cone = 1
    };

    struct Node;
    struct Corner
    {
        V2 coord = {0, 0};
        std::forward_list<Node> nodes; // nodes are not deleted until end of the algo
        float_t min_g = INF;
        mapkey_t key = 0;

        Corner(const mapkey_t &key, const V2 &coord) : coord(coord), key(key){};

        inline Node *const &emplace_node(Corner *const &crn, Node *const &parent, const float_t &g, const float_t &h, const NodeType &type, const V2 &v_left, const V2 &v_right, const int_t &dx)
        {
            return &nodes.emplace_front(crn, parent, g, h, type, v_left, v_right, dx);
        }
    };
    struct Node
    {
        // root coordinate
        V2 v_left = {0, 0}, v_right = {0, 0};
        Corner *const crn = nullptr; 
        Node *parent = nullptr, *openlist_next = nullptr, *openlist_prev = nullptr;
        float_t f = INF, g = INF, h = INF;
        int_t dx;
        // root key
        NodeType type = NodeType::Flat;

        Node(Corner *const &crn, Node *const &parent, const float_t &g, const float_t &h, const NodeType &type, const V2 &v_left, const V2 &v_right, const int_t &dx)
            : v_left(v_left), v_right(v_right), crn(crn), parent(parent), f(g + h), g(g), h(h), dx(dx), type(type) {}
    };

    class Corners
    {
    private:
        std::unordered_map<mapkey_t, Corner> _data;

    public:
        inline Corner *try_emplace(const mapkey_t &key, const V2 &coord)
        {
            return &(_data.try_emplace(key, key, coord).first->second);
        }
        inline void clear() { _data.clear(); }
        inline bool empty() const { return _data.empty(); }
    };
}