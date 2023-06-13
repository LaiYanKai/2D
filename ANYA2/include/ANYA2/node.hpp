#include <vector>
#include <unordered_map>
#include "P2D/P2D.hpp"

#pragma once
namespace P2D::ANYA2
{
    enum class NodeType
    {
        Point = 0,
        Flat = 1,
        Cone = 2
    };
    struct Node
    {
        // root coordinate
        const V2 coord = {0, 0};
        Node *parent = nullptr, *openlist_next = nullptr, *openlist_prev = nullptr;
        float_t f = INF, g = INF, h = INF;
        dir_idx_t di_vertical, di_horizontal; // always expand from the ray side
        V2 v_from = {0, 0}, v_to = {0, 0};
        // root key
        const mapkey_t key = 0;
        NodeType type = NodeType::Point;

        Node(const NodeType &type, const mapkey_t &key, const V2 &coord, Node *const &parent, const float_t &g, const float_t &h)
            : coord(coord), parent(parent), f(g + h), g(g), h(h), key(key), type(type) {}
    };

    class Nodes
    {
    private:
        std::unordered_map<mapkey_t, Node> _data;

    public:
        inline Node *const &try_emplace(const NodeType &type, const mapkey_t &key, const V2 &coord, Node *const &parent, const float_t &g, const float_t &h)
        {
            return &(_data.try_emplace(key, key, coord, parent, g, h).first->second);
        }
        inline void clear() { _data.clear(); }
        inline bool empty() const { return _data.empty(); }
    };
}