#include "P2D/P2D.hpp"
#include "node.hpp"

#pragma once
namespace P2D::TS2
{
    template <bool diag_block = true>
    class TS2
    {
    private:
        Nodes nodes;
        Grid *const grid;
        OpenList<Node> open_list;
        Los<diag_block> los;

    public:
        TS2(Grid *const &grid) : grid(grid), los(grid) { setup(); }
        TS2 &operator=(const TS2 &) = delete; // Disallow copying
        TS2(const TS2 &) = delete;
        ~TS2() {}

        // use when map data changes
        void setup()
        {
            const V2 &size_vert = grid->getSize<false>();
            nodes.setup(size_vert.x * size_vert.y);
        }

        std::vector<V2> run(const V2 &p_start, const V2 &p_goal)
        {
            mapkey_t k_start = grid->coordToKey<false>(p_start);
            mapkey_t k_goal = grid->coordToKey<false>(p_goal);

            // ===== Create start and goal nodes =======
            Node *const &node_goal = nodes.emplace(k_goal, p_goal, nullptr, INF, 0);
            Node *node_start = nodes.emplace(k_start, p_start, nullptr, 0, norm(p_start, p_goal));
            node_start->parent = node_start;
            Node *node = node_start;
            open_list.queue(node);

            std::vector<V2> path = {};
            while (true)
            {
                // ====== Poll Node ======
                node = open_list.poll();

                // ====== Openlist empty ======
                if (node == nullptr)
                    break; // no path found;

                // ====== Path found ======
                if (node == node_goal)
                {
                    do
                    {
                        path.push_back(node->coord);
                        node = node->parent;
                    } while (node != node_start);
                    path.push_back(p_start);
                    break;
                }

                // --------- Can't  check is_visited due to optimality issues ------------

                // ====== Get window of surrounding cells ======
                char window = 0;
                for (dir_idx_t di = 7; di > 0; di -= 2)
                {
                    window <<= 1;
                    mapkey_t cell_key = grid->addKeyToRelKey(node->key, grid->getCellRelKey(di, node->coord.x));
                    V2 cell_coord = node->coord + grid->getCellRelCoord(di);
                    window |= !(grid->isAccessible(cell_key, cell_coord)); // 1 means out-of-map or occupied.
                }

                // ------ Skip if vertex on checkerboard corner -------
                if constexpr (diag_block == true)
                    if (window == 0b0101 || window == 0b1010)
                        continue; // diagonally blocked, remaining two vertices are cheaper to reach from node's parent than detouring around current position

                // ====== Check Neighbors ======
                for (dir_idx_t di = 0; di < 8; ++di)
                {
                    bool vertex_blocked;
                    char mask;
                    if (isOrdinal(di))
                        mask = 1 << (di / 2);
                    else
                    { // cardinal
                        if (di == 0)
                            mask = 0b1001;
                        else if (di == 2)
                            mask = 0b0011;
                        else if (di == 4)
                            mask = 0b0110;
                        else
                            mask = 0b1100;
                    }
                    vertex_blocked = (window & mask) > 0;
                    if (vertex_blocked)
                        continue; // out of map or blocked by occupied cells

                    // ----- Get Neighbor node -----
                    mapkey_t nb_key = grid->addKeyToRelKey(node->key, grid->getRelKey<false>(di));
                    Node *node_nb = nodes[nb_key];
                    if (node_nb == nullptr)
                    {
                        V2 nb_coord = node->coord + grid->getRelCoord<false>(di);
                        node_nb = nodes.emplace(nb_key, nb_coord, nullptr, INF, norm(nb_coord, p_goal));
                    }

                    // ------ Determine potential parent of neighbor by testing LOS ------
                    Node *node_par = node->parent;
                    if (los.template cast(node_nb->key, node_nb->coord, node_par->key, node_par->coord) == false)
                        node_par = node; // neighbor is not visible to parent of expanded node

                    // ------ Determine parent definitely by testing against G-cost -------
                    float_t test_g = node_par->g + norm(node_nb->coord, node_par->coord);
                    if (approxGt(node_nb->g, test_g) == true)
                    {                               // is cheapest
                        open_list.unqueue(node_nb); // remove from ol, if any
                        node_nb->g = test_g;
                        node_nb->parent = node_par;
                        node_nb->f = node_nb->g + node_nb->h;
                        open_list.queue(node_nb);
                    }
                }
            }

            // ==== Remove nodes  ====
            open_list.clear();
            nodes.erase();

            return path;
        }
    };
}
