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

    // "min" is the end where the y coordinate is smaller.
    struct Interval
    {
        V2 p_min = {0, 0}, p_max = {0, 0};
        mapkey_t k_min = -1, k_max = -1;

        Interval(const mapkey_t &k_min, const V2 &p_min, const mapkey_t &k_max, const V2 &p_max)
            : p_min(p_min), p_max(p_max), k_min(k_min), k_max(k_max)
        {
            static_assert(-1 == mapkey_t(-1)); // if -1 is used
        }
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
    struct Div
    {
        // floored y value (guaranteed to be away from zero if quotient is negative)
        int_t floored;
        // dx *ray.y / ray.x

        // dy = dx * ray.y / ray.x = floored + remainder / ray.x
        // int_t dividend;
    };
    struct Node
    {
        // root coordinate
        V2 v_min = {0, 0}, v_max = {0, 0};
        Corner *const crn = nullptr;
        Node *parent = nullptr, *openlist_next = nullptr, *openlist_prev = nullptr;
        float_t f = INF, g = INF, h = INF;
        int_t dx;
        // root key
        NodeType type = NodeType::Flat;

        Node(Corner *const &crn, Node *const &parent, const float_t &g, const float_t &h, const NodeType &type, const V2 &v_min, const V2 &v_max, const int_t &dx)
            : v_min(v_min), v_max(v_max), crn(crn), parent(parent), f(g + h), g(g), h(h), dx(dx), type(type) {}
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