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
        V2 neg_ray = {0,0}, pos_ray = { 0,0};
        Interval(const V2 &neg_ray, const V2 &pos_ray) : neg_ray(neg_ray), pos_ray(pos_ray) {}
    };

    struct Node;
    struct Corner
    {
        V2 coord = {0, 0};
        std::forward_list<Node> nodes; // nodes are not deleted until end of the algo
        float_t min_g = INF;
        mapkey_t key = 0;

        Corner(const mapkey_t &key, const V2 &coord) : coord(coord), key(key){};
        inline Node *emplaceNode(Node *const &parent, const float_t &g, const NodeType &type, const int_t &dx) { return &nodes.emplace_front(this, parent, g, type, dx); }
    };

    struct Node
    {
        V2 pos_ray = {0, 0}, neg_ray = {0, 0};
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

            V2f pv_neg = V2f(node.dx, float_t(node.dx) * node.neg_ray.y / node.neg_ray.x) + V2f(root);
            V2f pv_pos = V2f(node.dx, float_t(node.dx) * node.pos_ray.y / node.pos_ray.x) + V2f(root);
            out << ", itv[calc](" << pv_neg << "; " << pv_pos << ")";

            out << ", ray(" << node.neg_ray;
            out << ";" << node.pos_ray << ")";

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

    struct Cone
    {
        Node *const node;
        Cone(Node *const &node) : node(node) { assert(node->dx != 0); }
        inline const V2 &root() const { return node->crn->coord; }
        inline const int_t &dx() const { return node->dx; }
        inline int_t &dx() { return node->dx; }
        inline int_t sgnX() const
        {
            assert(this->dx() != 0);
            return this->dx() > 0 ? 1 : -1;
        }
        inline int_t dxNext() const { return dx() + sgnX(); }
        inline int_t x() const { return root().x + dx(); }
        inline int_t xNext() const { return root().x + dxNext(); }
        inline const V2 &negRay() const { return node->neg_ray; }
        inline V2 &negRay() { return node->neg_ray; }
        inline const V2 &posRay() const { return node->neg_ray; }
        inline V2 &posRay() { return node->pos_ray; }
        inline const V2 &ray(const int_t &sgn_y) const { return sgn_y < 0 ? negRay() : posRay(); }
        inline V2 &ray(const int_t &sgn_y) { return sgn_y < 0 ? negRay() : posRay(); }

        inline int_t rayY(const int_t &dx, V2 const &ray) const { return dx * ray.y / ray.x; }
        inline float_t rayYf(const int_t &dx, V2 const &ray) const { return float_t(dx) * ray.y / ray.x; }
        inline int_t negRayCurY() const { return rayY(dx(), negRay()); }
        inline int_t posRayCurY() const { return rayY(dx(), posRay()); }
        inline int_t rayCurY(const int_t &sgn_y) const { return sgn_y < 0 ? negRayCurY() : posRayCurY(); }
        inline int_t negRayNextY() const { return rayY(dxNext(), negRay()); }
        inline int_t posRayNextY() const { return rayY(dxNext(), posRay()); }
        inline int_t rayNextY(const int_t &sgn_y) const { return sgn_y < 0 ? negRayNextY() : posRayNextY(); }

        inline V2 negRayNextCoord() const { return V2(xNext(), negRayNextY()); }
        inline V2 posRayNextCoord() const { return V2(xNext(), posRayNextY()); }
        inline V2 rayNextCoord(const int_t &sgn_y) const { return sgn_y < 0 ? negRayNextCoord() : posRayNextCoord(); }
        inline V2 negRayCurCoord() const { return V2(x(), negRayCurY()); }
        inline V2 posRayCurCoord() const { return V2(x(), posRayCurY()); }
        inline V2 rayCurCoord(const int_t &sgn_y) const { return sgn_y < 0 ? negRayCurCoord() : posRayCurCoord(); }

        inline bool negRayHasTail() const { return negRay().y < 0; }
        inline bool posRayHasTail() const { return posRay().y > 0; }
        inline bool rayHasTail(const int_t &sgn_y) const { return sgn_y < 0 ? negRayHasTail() : posRayHasTail(); }
        inline bool negRayIsPerp() const { return negRay().y == 0; }
        inline bool posRayIsPerp() const { return posRay().y == 0; }
        inline bool rayIsPerp(const int_t &sgn_y) const { return sgn_y < 0 ? negRayIsPerp() : posRayIsPerp(); }
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