#include "P2D/P2D.hpp"

#pragma once
namespace P2D::VG2
{
    // struct Corner
    // {
    //     mapkey_t key;
    //     V2 coord;
    //     std::vector<Corner *> neighbors;
    //     Corner(mapkey_t key, V2 coord) : key(key), coord(coord), neighbors(){};
    // };
    // struct Node
    // {
    //     bool visited;
    //     float_t f, g, h;
    //     Corner *crn;
    //     Node *parent, *next, *prev;
    //     Node(Corner *crn, float_t h, std::list<Node *>::iterator open_) : visited(false), f(h), g(INF), h(h), crn(crn), parent(nullptr), open_(open_) {}
    // };
    // class OpenList
    // {
    // private:
    //     std::list<Node *> pq;

    // public:
    //     void push(Node *node);
    //     bool empty();
    //     Node *poll();
    //     void erase(std::list<Node *>::iterator open_);
    //     std::list<Node *>::iterator end();
    // };

    // void OpenList::push(Node *new_node)
    // {
    //     for (auto node_ = pq.begin(); node_ != pq.end(); ++node_)
    //     {
    //         if (new_node->f < (*node_)->f)
    //         {
    //             new_node->open_ = pq.emplace(node_, new_node);
    //             return;
    //         }
    //     }
    //     pq.emplace_back(new_node);
    //     new_node->open_ = std::prev(end());
    // }
    // bool OpenList::empty()
    // {
    //     return pq.empty();
    // }
    // Node *OpenList::poll()
    // {
    //     Node *node = pq.front();
    //     pq.pop_front();
    //     node->open_ = end();
    //     return node;
    // }
    // void OpenList::erase(std::list<Node *>::iterator open_)
    // {
    //     pq.erase(open_);
    // }
    // std::list<Node *>::iterator OpenList::end()
    // {
    //     return pq.end();
    // }
}