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
        TS2(Grid *const &grid, const std::filesystem::path fp_vg) : grid(grid), los(grid) { setup(); }
        TS2 &operator=(const TS2 &) = delete; // Disallow copying
        TS2(const TS2 &) = delete;
        ~TS2() {}

        void setup()
        {
            const V2 &size_vert = grid->getSize<false>();
            nodes.resize(size_vert.x * size_vert.y);
        }

        std::vector<V2> run(const V2 &p_start, const V2 &p_goal)
        {
            mapkey_t k_start = grid->coordToKey<false>(p_start);
            mapkey_t k_goal = grid->coordToKey<false>(p_goal);

            // ===== Create start and goal nodes =======
            Node *const &node_goal = nodes.emplace(k_goal, p_goal, nullptr, INF, 0);
            Node *node = nodes.emplace(k_start, p_start, nullptr, 0, norm(p_start, p_goal));
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
                    } while (node != nullptr);
                    break;
                }

                // ====== Skip if already expanded ======
                if (node->is_visited == true)
                    continue; // is visited. continue;
                node->is_visited = true;

                // ====== Queue neighbors if possible ======
                for (dir_idx_t di = 0; di < 8; ++di)
                {
                    bool can_access;
                    if (isOrdinal(di) == true)
                    {
                        mapkey_t cell_key = grid->addKeyToRelKey(node->key, grid->getCellRelKey(di, node->coord.x));
                        V2 cell_coord = node->coord + grid->getCellRelCoord(di);
                        can_access = grid->isAccessible(cell_key, cell_coord);
                    }
                    else
                    {
                        char window = 0;
                        for (const dir_idx &rel_di : {1, 3, 5, 7})
                        {
                            window <<= 1;
                            dir_idx_t di_rotated = addDirIdx(di, rel_di);
                            mapkey_t cell_key = grid->addKeyToRelKey(node->key, grid->getCellRelKey(di_rotated, node->coord.x));
                            V2 cell_coord = node->coord + grid->getCellRelCoord(di_rotated);
                            window |= grid->isAccessible(cell_key, cell_coord);
                        }
                        switch (window)
                        {
                        case 0b0110:
                        case 0b0010:
                        case 0b0100:
                        case 0b0000:
                            can_access = false;
                            break;
                        case 0b0101:
                        default:
                            can_access = true;
                        }
                    }

                    Node *&node_nb = crn_nb->node;
                    float_t test_g = node->g + norm(crn_nb->coord, crn->coord);

                    if (node_nb == nullptr) // emplace directly if no nodes at crn
                    {
                        node_nb = nodes.emplace(crn_nb, node, test_g, norm(crn_nb->coord, p_goal));
                        open_list.queue(node_nb);
                    }
                    else if (node_nb != nullptr)
                    { // test for g cost if a node already exists at crn
                        if (node_nb->is_visited == false && approxGt(node_nb->g, test_g) == true)
                        {                               // not visited and cheapest
                            open_list.unqueue(node_nb); // remove from ol
                            node_nb->g = test_g;
                            node_nb->parent = node;
                            node_nb->f = node_nb->g + node_nb->h;
                            open_list.queue(node_nb);
                        }
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
