#include <vector>
#include <unordered_map>
#include <forward_list>
#include <ostream>
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
        // Interval(const int_t &x_neg, const int_t &y_neg, const V2 &root_coord) : vert_neg(x_neg, y_neg), diff_neg(vert_neg - root_coord) {}
        Interval(const V2 &vert_neg, const V2 &diff_neg) : vert_neg(vert_neg), diff_neg(diff_neg) {}
    };

    struct Node;
    struct Corner
    {
        V2 coord = {0, 0};
        std::forward_list<Node> nodes; // nodes are not deleted until end of the algo
        float_t min_g = INF;
        mapkey_t key = 0;

        Corner(const mapkey_t &key, const V2 &coord) : coord(coord), key(key){};

        inline Node *emplaceNode(Node *const &parent, const float_t &g, const NodeType &type, const int_t &dx)
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

        friend std::ostream &operator<<(std::ostream &out, Node const &node)
        {
            const V2 &root = node.crn->coord;
            out << "<N";
            out << get_addr((void *)&node);
            out << "|";
            out << (node.type == NodeType::Flat ? "F" : "C");
            out << ":";
            out << root << ">";

            out << ", dx(" << node.dx << ")";

            if (node.parent == nullptr)
                out << ", par(    NA     )";
            else
                out << ", par(" << node.parent->crn->coord << ")";

            V2f pv_neg = V2f(node.dx, float_t(node.dx) * node.ray_neg.y / node.ray_neg.x) + V2f(root);
            V2f pv_pos = V2f(node.dx, float_t(node.dx) * node.ray_pos.y / node.ray_pos.x) + V2f(root);
            out << ", itv[calc](" << pv_neg << "; " << pv_pos << ")";

            out << ", ray(" << node.ray_neg;
            out << ";" << node.ray_pos << ")";

            out << ", F$(" << node.f << ")";
            out << ", G$(" << node.g << ")";
            out << ", H$(" << node.h << ")";
            return out;
        }
    };
    inline std::ostream &operator<<(std::ostream &out, const Node *const &node)
    {
        if (node == nullptr)
            out << "    NANode     ";
        else
            out << *node;
        return out;
    }

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