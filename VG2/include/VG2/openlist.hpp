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
}