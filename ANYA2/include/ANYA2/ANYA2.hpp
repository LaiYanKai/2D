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

        inline bool expandCone(Node *const &node, const V2 &p_goal, std::vector<V2> &path)
        {
            _dbgtitle("[Cone] Expand Cone Node { " << node << " }");
            _dbginc;
            assert(node->dx != 0);
            // requires rays and dx to be resolved for current interval at dx.

            // ====================== Check if node's expanded interval intersects goal =============================
            if (node->dx + node->crn->coord.x == p_goal.x)
            {
                V2 y_gfr = p_goal.y - node->crn->coord.y; // y of vector goalFromRoot
                bool found_goal;
                const Ray &ray_neg = node->ray_neg;
                if (ray_neg.vec.y <= 0)               // ray is going to -y or no change in y from root
                    found_goal = y_gfr >= ray_neg.dy; // the result of quotient (dx * ray.y / ray.x) is in +y direction of or on true value
                else                                  // ray is going to +y from root
                    found_goal = y_gfr > ray_neg.dy;  // the result of quotient is in -y direction of true value

                const Ray &ray_pos = node->ray_pos;
                if (ray_pos.y >= 0)                    // ray is going to +y or no change in y from root
                    found_goal &= y_gfr <= ray_pos.dy; // result of quotient is in -y direction of or on true value
                else                                   // ray is going to -y from root
                    found_goal &= y_gfr < ray_pos.dy;  // the result of quotient is in +y direction of true value

                if (found_goal == true)
                { // terminate and find path if goal intersects interval
                    assert(path.empty() == true);
                    path.push_back(p_goal);
                    Node *n = node;
                    do
                    {
                        path.push_back(n->crn->coord);
                        n = n->parent;
                    } while (n != nullptr);
                    return true;
                }
            }

            // ====================== Add Turning Points on both (+y and -y) sides ======================
            bool created_flat_nodes = false;
            for (const int_t sgn_y : {-1, 1})
            {
                const Ray &ray = sgn_y < 0 ? node->ray_neg : node->ray_pos;
                if (ray.remainder == false &&)
                { // i.e. there can be a turning point on this side.
                    V2 cell_coord;
                    mapkey_t cell_key;

                    // check +dx, +dy cell
                    dir_idx_t di = dirToDirIdx(node->dx, dy);
                    const V2 vert_coord = ray + node->crn->coord;
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
                                created_flat_nodes = true;
                                // createFlatNode();
                                // set new vmax to this vmin
                                // set new vmin to 0; // search in -y when expanding flat.
                                // set new root to vert_coord, vert_key
                                // set new dx to this dx

                            } // create flat
                        }     // -dx, +dy cell is occupied
                    }         // +dx +dy cell is free
                }             // has turning point if +dx is same as ray.x
            }

            // ====================== Determine starting (-y side) and ending cell (+y side) ======================
            mapkey_t kc_min, kc_max;
            V2 pc_min, pc_max;
            for (const int_t &dy : {-1, 1})
            {
                const V2 &ray = dy < 0 ? node->v_min : node->v_max;
                V2 &pc = dy < 0 ? pc_min : pc_max;
                mapkey_t &kc = dy < 0 ? kc_min : kc_max;

                _dbg11("[Cone:" << (dy < 0 ? "MinY" : "MaxY") << "] Ray is (" << ray << ")");
                if (dy < 0 == ray.y < 0)
                { // the starting -y cell depends on the next row
                    const V2 vert_coord = node->crn->coord + V2{dx_next, dx_next * ray.y / ray.x};
                    const mapkey_t vert_key = grid->coordToKey<false>(vert_coord);
                    // kc is in -dx, +y direction (does not matter for max)
                    dir_idx_t di = dirToDirIdx(-node->dx, 1);
                    grid->getCellKeyAndCoord(di, vert_coord, vert_key, kc, pc);
                    if (dy < 0)
                        _dbg11("[Cone:MinY] First Cell (" << pc << ") depends on NEXT row floored vertex(" << vert_coord << ")");
                    else
                        _dbg11("[Cone:MaxY] Last Cell (" << pc << ") depends on NEXT row floored vertex(" << vert_coord << ")");
                }
                else
                { // the starting -y cell depends on the current row
                    const V2 vert_coord = node->crn->coord + V2{node->dx, node->dx * ray.y / ray.x};
                    const mapkey_t vert_key = grid->coordToKey<false>(vert_coord);
                    // kc_min is in +dx, +y direction
                    dir_idx_t di = dirToDirIdx(node->dx, 1);
                    grid->getCellKeyAndCoord(di, vert_coord, vert_key, kc, pc);
                    if (dy < 0)
                        _dbg11("[Cone:MinY] First Cell (" << pc << ") depends on CURRENT row floored vertex(" << vert_coord << ")");
                    else
                        _dbg11("[Cone:MaxY] Last Cell (" << pc << ") depends on CURRENT row floored vertex(" << vert_coord << ")");
                }
            }

            // ================== Find intervals by scanning from first cell to last cell ==========================
            std::vector<Interval> intervals;
            {
                bool scan_accessible = grid->isAccessible(kc_min, pc_min);
                if (scan_accessible == true) // first cell is free, min side of interval is bounded by minray (v_min) of node.
                    intervals.emplace_back(-1, {0, 0}, -1, {0, 0});
                const mapkey_t &rkc = grid->getRelKey<true>(2);                                       // +y direction
                const mapkey_t rkv_min = grid->getVertexRelKey(dirToDirIdx(-node->dx, -1), pc_min.x); // relative vertex key in -dx, -y direction from cell
                const V2 rpv_min = grid->getVertexRelCoord(dirToDirIdx(-node->dx, -1));               // relative vertex coord in -dx, -y direction from cell

                V2 pc = pc_min;
                mapkey_t kc = kc_min;
                while (true)
                {
                    // get next cell in +y direction
                    kc = grid->addKeyToRelKey(kc, rkc);
                    if (++pc.y > pc_max.y)
                        break; // exit if outside of the interval

                    if (scan_accessible == true && grid->isAccessible(kc, pc) == false)
                    { // found the first inaccessible cell (out-of-map, occupied) after scanning free interval
                        // flip state
                        scan_accessible = false;

                        // fill the unfilled interval
                        assert(intervals.empty() == false && intervals.back().k_max == -1); // there must be an unfilled interval
                        Interval &interval = intervals.back();
                        interval.k_max = grid->addKeyToRelKey(kc, rkv_min);
                        interval.p_max = pc + rpv_min;
                    }
                    else if (scan_accessible = false && grid->isAccessible(kc, pc) == true)
                    { // found the first accessible cell after scanning inaccessible interval
                        // flip state
                        scan_accessible = true;

                        // create new interval at current vertex
                        assert(intervals.empty() == true || intervals.back().k_max >= 0); // interval must be filled or no interval at all
                        intervals.emplace_back(
                            grid->addKeyToRelKey(kc, rkv_min),
                            pc + rpv_min,
                            -1, {0, 0});
                    }
                }
            }

            // ================== Create Intervals ==========================
            if (intervals.empty() == true)
            { // regardless of flat nodes being created
                _dbg11("[Cone] Next row is inaccessible. Stop expansion.");
                // delete node and all applicable parents if possible
                _dbgdec;
                return true; // terminate
            }                // no intervals
            else if (created_flat_nodes == false && intervals.size() == 1)
            {
                _dbgtitle("[Cone] Intermediate Pruning: Continue to next row as there is only one successor");
                _dbginc;
                node->dx += node->dx > 0 ? 1 : -1;
                const Interval &interval = intervals.back();
                if (interval.k_min >= 0)
                {
                    node->v_min = interval.p_min - node->crn->coord;
                    _dbg11("[Cone] Min Ray changed to ( " << node->v_min << " )");
                }
                else
                    _dbg11("[Cone] Min Ray is unchanged");

                if (interval.k_max >= 0)
                {
                    node->v_max = interval.p_max - node->Crn->coord;
                    _dbg11("[Cone] Max Ray changed to ( " << node->v_max << " )")
                }
                else
                    _dbg11("[Cone] Max Ray is unchanged");

                _dbgdec;
                _dbgdec;
                return false;
            } // no flat nodes created and no other successors
            else
            {
                _dbgtitle("[Cone] More than one successor:");
                _dbginc;
                assert(intervals.size() > 1 || created_flat_nodes == true); // one interval and created flat nodes, or multiple intervals (regardless of flat nodes creation)

                bool used_current_node = false;
                for (const Interval &interval : intervals)
                {
                    _dbg11("[Cone] ---- Interval from VertexMin( " << interval.p_min << " ) to VertexMax( " << interval.p_max << " ) ----");

                    V2 new_v_min = interval.k_min >= 0 ? interval.p_min - node->crn->coord : node->v_min;
                    V2 new_v_max = interval.k_max >= 0 ? interval.p_max - node->crn->coord : node->v_max;
                    _dbg11("[Cone] New VMin( " << new_v_min << " ), new VMax( " << new_v_max << " )");

                    if (det(new_v_min, new_v_max) == 0)
                    { // is parallel, no node can be formed
                        _dbg11("[Cone] VMin and VMax is parallel, no node can be formed.");
                        continue;
                    }

                    if (used_current_node == true)
                    {
                        _dbg11("[Cone] Create new cone node at root")
                    }
                }

                _dbgdec;
                _dbgdec;
                return true;
            } // more than one successor (cone node or flat node)
            assert(false);
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
