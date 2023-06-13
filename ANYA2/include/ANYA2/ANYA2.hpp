#include "P2D/P2D.hpp"
#include "node.hpp"

#pragma once
namespace P2D::ANYA2
{
    template <bool diag_block = true>
    class ANYA2
    {
    private:
        Nodes nodes;
        Grid *const grid;
        OpenList<Node> open_list;
        Los<diag_block> los;

    public:
        ANYA2(Grid *const &grid) : grid(grid), los(grid) { setup(); }
        ANYA2 &operator=(const ANYA2 &) = delete; // Disallow copying
        ANYA2(const ANYA2 &) = delete;
        ~ANYA2() {}

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
            // open_list.queue(node);

            std::vector<V2> path = {};
            while (true)
            {
                // ====== Poll Node ======
                node = open_list.poll();

                // ====== Openlist empty ======
                if (node == nullptr)
                    break; // no path found;

                // // ====== Path found ======
                // if (node == node_goal)
                // {
                //     do
                //     {
                //         path.push_back(node->coord);
                //         node = node->parent;
                //     } while (node != node_start);
                //     path.push_back(p_start);
                //     break;
                // }

                
            }

            // ==== Remove nodes  ====
            open_list.clear();
            nodes.erase();

            return path;
        }
    };
}
