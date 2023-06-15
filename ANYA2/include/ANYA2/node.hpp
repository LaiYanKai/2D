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

    struct Interval
    {
        V2 vert_pos = {-1, -1}, vert_neg = {-1, -1}, diff_pos, diff_neg;
        Interval(const int_t &x_neg, const int_t &y_neg, const V2 &root_coord) : vert_neg(x_neg, y_neg), diff_neg(vert_neg - root_coord) {}
    };

    struct Node;
    struct Corner
    {
        V2 coord = {0, 0};
        std::forward_list<Node> nodes; // nodes are not deleted until end of the algo
        float_t min_g = INF;
        mapkey_t key = 0;

        Corner(const mapkey_t &key, const V2 &coord) : coord(coord), key(key){};

        inline Node *const &emplaceNode(Node *const &parent, const float_t &g, const NodeType &type, const int_t &dx)
        {
            return &nodes.emplace_front(this, parent, g, type, dx);
        }
    };
    struct Node
    {
        V2 ray_pos = {0, 0}, ray_neg = {0, 0};
        Corner *const crn = nullptr;
        Node *parent = nullptr, *openlist_next = nullptr, *openlist_prev = nullptr;
        float_t f = INF, g = INF, h = INF;
        int_t dx = 0;

        // root key
        NodeType type = NodeType::Flat;

        Node(Corner *const &crn, Node *const &parent, const float_t &g, const NodeType &type, const int_t &dx)
            : crn(crn), parent(parent), g(g), dx(dx), type(type) {}
    };

    struct Boundary
    {
        const V2 &ray;
        V2 pv_cur, pv_bound;
        mapkey_t kv_cur, kv_bound, kc_bound;
        const int_t sgn_y;
        const int_t ray_dir; // if positive, there is a tail. if zero, ray has no y component. if negative, ray has no tail

        // does not calculate next interval
        Boundary(const V2 &ray, const int_t &sgn_y) : ray(ray), sgn_y(sgn_y), ray_dir(ray.y * sgn_y) {}
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