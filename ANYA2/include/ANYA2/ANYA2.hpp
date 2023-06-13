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
            const dir_idx_t di_f = node->dx > 0 ? 0 : 4;
            const dir_idx_t di_fl = addDirIdx(di_f, 1);
            const dir_idx_t di_bl = addDirIdx(di_f, 3);
            const dir_idx_t di_br = addDirIdx(di_f, 5);
            const dir_idx_t di_fr = addDirIdx(di_f, 7);

            // =============== get left and right vertices (within and including on both rays)=====================
            V2 p_left = {node->crn->coord.x + node->dx, 0}, p_right = p_left;
            assert(node->dx != 0);
            if (node->dx > 0)
            { // left for +x is +y (floor), right is -y (ceil)
                // normal floor from division
                p_left.y = (node->dx * node->v_left.y / node->v_left.x) + node->crn->coord.y;
                // to get ceil
                int_t numer = (node->dx * node->v_right.y);
                int_t denom = node->v_right.x;
                p_right.y = numer / denom;
                if (p_right.y * denom != numer) // flooring occurred
                    ++p_right.y;                // +1 for ceil
                p_right.y += node->crn->coord.y;
            }
            else
            { // left for -x is -y (ceil), right is +y (floor)
                // normal floor from division
                p_right.y = (node->dx * node->v_right.y / node->v_right.x) + node->crn->coord.y;
                // to get ceil
                int_t numer = (node->dx * node->v_left.y);
                int_t denom = node->v_left.x;
                p_left.y = numer / denom;
                if (p_left.y * denom != numer) // flooring occurred
                    ++p_left.y;                // +1 for ceil
                p_left.y += node->crn->coord.y;
            }
            mapkey_t k_left = grid->coordToKey<false>(p_left);
            mapkey_t k_right = grid->coordToKey<false>(p_right);
            _dbg11("[Cone] LeftVertex(" << p_left << ") RightVertex(" << p_right << ") ");

            // =============== Find turning points on Left and Right vertices =====================
            std::vector<std::pair<mapkey_t, V2>> intervals;
            bool scanning_accessible;
            mapkey_t cell_key;
            V2 cell_coord;
            char window = 0;
            for (const Side &side : {Side::R, Side::L}) // order bcos we always scan from left to right
            {
                const dir_idx_t &di_front = side == Side::L ? di_fl : di_fr;
                const dir_idx_t &di_back = side == Side::L ? di_bl : di_br;
                const mapkey_t &vert_key = side == Side::L ? k_left : k_right;
                const V2 &vert_coord = side == Side::L ? p_left : p_right;

                grid->getCellKeyAndCoord(di_front, vert_key, vert_coord, cell_key, cell_coord);
                const bool access_front = grid->isAccessible(cell_key, cell_coord);
                grid->getCellKeyAndCoord(di_back, vert_key, vert_coord, cell_key, cell_coord);
                const bool access_back = grid->isAccessible(cell_key, cell_coord);

                if (access_front == true)
                { // FS cell is accessible (front_left if side == L, etc.)
                    if (access_back == true)
                    {   // BS cell is accessible (back_left if side == L, etc.)

                    }
                    else
                    {   // turning point if not diag block
                        if constexpr (diag_block)
                        {
                            const dir_idx_t &di_front_rev = side == Side::L ? di_fr : di_fl;
                            grid->getCellKeyAndCoord(di_front_rev, vert_key, vert_coord, cell_key, cell_coord);
                            bool access_front_rev = grid->isAccessible(cell_key, cell_coord);
                            if (access_front_rev == true)
                            {

                            }
                        }

                    }
                }
                else
                { // FS cell not accessibles
                }
            }

            grid->getCellKeyAndCoord(di_fl, k_left, p_left, cell_key, cell_coord);
            if (grid->isAccessible(cell_key, cell_coord))
            { // FL cell is accessible
                scanning_accessible = true;
                grid->getCellKeyAndCoord(di_bl, k_left, p_left, cell_key, cell_coord);
                if (grid->isAccessible(cell_key, cell_coord))
                { // BL cell is accessible (no turning point)
                }
                else
                { // BL cell not accessible
                    if constexpr (diag_block)
                    {
                        grid->getCellKeyAndCoord(di_fr, k_left, p_left, cell_key, cell_coord);
                        if (grid->isAccessible(cell_key, cell_coord))
                        { // FL, FR cell accessible; BL not accessible;
                        }
                    }
                    else
                    {
                    }
                }
            }
            else
            { // FL cell not accessible (no turning point)
                scanning_accessible = false;
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
