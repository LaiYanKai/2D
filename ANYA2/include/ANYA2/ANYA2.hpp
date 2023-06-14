#include "P2D/P2D.hpp"
#include "node.hpp"

#pragma once
namespace P2D::ANYA2
{
    template <bool diag_block = true>
    class ANYA2
    {
    private:
        Corners crns;
        Grid *const grid;
        OpenList<Node> open_list;
        Los<diag_block> los;

        void expandCone(Node *const &node)
        {
            _dbgtitle("[Cone] Expand Cone Node { " << node << " }");
            _dbginc;

            for (const int_t dy : {-1, 1})
            {
                const V2 &vec_y = dy == -1 ? node->v_min : node->v_max;
                if (vec_y.x == node->dx)
                { // i.e. there can be a turning point on the min y side.
                    V2 cell_coord;
                    mapkey_t cell_key;

                    // check +dx, +dy cell
                    dir_idx_t di = dirToDirIdx(node->dx, dy);
                    const V2 vert_coord = vec_y + node->crn->coord;
                    const mapkey_t vert_key = grid->coordToKey<false>(vert_coord);
                    grid->getCellKeyAndCoord(di, vert_key, vert_coord, cell_key, cell_coord);
                    if (grid->isAccessible(cell_key, cell_coord) == true)
                    { // +dx, +dy cell is free

                        // check -dx, +dy cell
                        di = dirToDirIdx(-node->dx, dy);
                        grid->getCellKeyAndCoord(di, vert_key, vert_coord, cell_key, cell_coord);
                        if (grid->isAccessible(cell_key, cell_coord) == false)
                        { // -dx, +dy cell is occupied

                            bool create_flat = true;
                            if constexpr (diag_block == true)
                            { // check +dx, -dy cell
                                di = dirToDirIdx(node->dx, -dy);
                                grid->getCellKeyAndCoord(di, vert_key, vert_coord, cell_key, cell_coord);
                                // create flat if +dx, +y cell is free
                                create_flat = grid->isAccessible(cell_key, cell_coord) == true;
                            }

                            if (create_flat == true)
                            {
                                // createFlatNode();
                                // set new vmax to this vmin
                                // set new vmin to 0; // search in -y when expanding flat.
                                // set new root to vert_coord, vert_key
                                // set new dx to this dx

                            } // create flat
                        }     // -dx, +dy cell is occupied
                    }         // +dx +dy cell is free
                }             // has turning point if +dx is same as vec_y.x
            }

            _dbgdec;
        }

    public:
        ANYA2(Grid *const &grid) : grid(grid), los(grid) { setup(); }
        ANYA2 &operator=(const ANYA2 &) = delete; // Disallow copying
        ANYA2(const ANYA2 &) = delete;
        ~ANYA2() {}

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
            nodes.clear();

            return path;
        }
    };
}
