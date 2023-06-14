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
    struct Ray
    {
        V2 vec = {0, 0};        // the vector of the ray from the root
        mapkey_t key = -1;      // the vertex ({dx, dy} + root) key at the quotient of (dx * ray.y /ray.x)
        int_t dy = 0;           // the quotient for the integer division: (dx * ray.y / ray.x)
        bool remainder = false; // indicate if a remainder exists for the integer division: (dx * ray.y / ray.x)

        Ray() {}
        Ray(const V2 ray, const mapkey_t &key, const int_t &dy, const bool &remainder)
            : vec(vec), key(key), dy(dy), remainder(remainder)
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

        inline Node *const &emplaceNode(Node *const &parent, const float_t &g, const float_t &h, const NodeType &type, const int_t &dx)
        {
            return &nodes.emplace_front(this, parent, g, h, type, dx);
        }
    };
    struct Node
    {
        Ray ray_pos, ray_neg;
        Corner *const crn = nullptr;
        Node *parent = nullptr, *openlist_next = nullptr, *openlist_prev = nullptr;
        float_t f = INF, g = INF, h = INF;
        int_t dx = 0;

        // root key
        NodeType type = NodeType::Flat;

        Node(Corner *const &crn, Node *const &parent, const float_t &g, const float_t &h, const NodeType &type, const int_t &dx)
            : crn(crn), parent(parent), f(g + h), g(g), h(h), dx(dx), type(type) {}
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