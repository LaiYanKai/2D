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

        inline void updateHCost(Node *const &node, V2 p_goal) const
        {
            // flip around the root if dx != v_goalFromRoot.x

            if (node->type == NodeType::Cone)
            {

                int_t interval_x = node->crn->coord.x + node->dx;
                int_t dx_gfi = p_goal.x - interval_x;
                if (dx_gfi * sgn(node->dx) < 0)
                    p_goal.x = interval_x - dx_gfi; // flip around the interval if "below" the interval

                assert(sgn(node->dx) * det(node->ray_pos, node->ray_neg) < 0);

                int_t v_gfr = p_goal - node->crn->coord;
                if (sgn(node->dx) * det(v_gfr, node->ray_pos) < 0)
                { // h cost has to detour around node->ray_pos
                    V2f vert_pos(node->dx, float_t(node->dx) * node->ray_pos.y / node->ray_pos.x);
                    vert_pos += node->crn->coord;
                    node->h = norm<float_t>(p_goal, vert_pos) + norm<float_t>(vert_pos, node->crn->coord);
                }
                else if (sgn(node->dx) * det(node->ray_neg, v_gfr) > 0)
                { // h cost has to detour around node->ray_neg
                    V2f vert_neg(node->dx, float_t(node->dx) * node->ray_neg.y / node->ray_neg.x);
                    vert_neg += node->crn->coord;
                    node->h = norm<float_t>(p_goal, vert_neg) + norm<float_t>(vert_neg, node->crn->coord);
                }
                else // calculate h-cost directly
                    node->h = norm(p_goal, node->crn->coord);
            }    // cone type
            else // flat type
                node->h = norm(p_goal, node->crn->coord);
            node->f = node->g + node->h;
        }

        inline void expandFlat(Node *const &node, const V2 &p_goal, std::vector<V2> &path)
        { // expand until cannot proceed anymore is reached
            _dbgtitle("[Flat] Node { " << node << " }");
            _dbginc;
            mapkey_t kv = node->crn->key;
            const V2 &root = node->crn->coord;
            int_t y = root.y;
            int_t x = root.x;
            const int_t sgn_y = node->ray_neg == 0 ? -1 : 1;
            dir_idx_t di = dirIdxToDir(0, sgn_y);
            int_t last_y = grid->getBoundary<false>(di);
            mapkey_t rkv = grid->getRelKey<false>(di);
            mapkey_t rkc = grid->getRelKey<true>(di);
            di = dirIdxToDir(1, sgn_y);
            mapkey_t kc_above = grid->addKeyToRelKey(kv, grid->getCellRelKey(di, x));
            di = dirIdxToDir(-1, sgn_y);
            mapkey_t kc_below = grid->addKeyToRelKey(kv, grid->getCellRelKey(di, x));

            Corner *crn_above = nullptr, *crn_below = nullptr;
            bool is_oc_above_prev = false;
            bool is_oc_below_prev = false;
            if (node->dx > 0)
            {
                crn_above = node->crn;
                is_oc_below_prev = true;
            }
            else
            {
                crn_below = node->crn;
                is_oc_above_prev = true;
            }

            // move in sgn_y by one unit
            bool to_stop = false;
            do
            {
                y += sgn_y;
                bool is_oc_above, is_oc_below;
                if (y == p_goal.y)
                { // goal found
                    assert(path.empty() == true);
                    path.push_back(p_goal);
                    Node *n = node;
                    do
                    {
                        path.push_back(n->crn->coord);
                        n = n->parent;
                    } while (n != nullptr);
                    _dbgdec;
                    return;
                }
                else if (y == last_y)
                {
                    is_oc_above = true;
                    is_oc_below = true;
                    to_stop = true;
                }
                else
                {
                    kc_above = grid->addKeyToRelKey(kc_above, rkc);
                    kc_below = grid->addKeyToRelKey(kc_below, rkc);
                    is_oc_above = grid->isOc(kc_above);
                    is_oc_below = grid->isOc(kc_below);
                }

                for (const int_t &sgn_x : {-1, 1})
                {
                    bool &is_oc = sgn_x > 0 ? is_oc_above : is_oc_below;
                    bool &is_oc_prev = sgn_x > 0 ? is_oc_above_prev : is_oc_below_prev;
                    Corner *&crn = sgn_x > 0 ? crn_above : crn_below;

                    if (is_oc == true && is_oc_prev == false)
                    {
                        assert(crn != nullptr);

                        if (node->type == NodeType::Flat)
                        { // have not converted the flat node to cone node
                            assert(sgn(node->dx) == sgn_x);
                            Ray &ray_to = sgn_y > 0 ? node->ray_pos : node->ray_neg;
                            ray_to = V2(sgn_x, y - crn->coord.y);

                            node->type = NodeType::Cone;
                            if (sgn_x * det(node->ray_pos, node->ray_neg) <= 0)
                            {
                                _dbg11("[Flat] Cannot queue this flat node as a cone node because the next row is inaccessible");
                            }
                            else
                            {
                                updateHCost(node, p_goal);
                                open_list.queue(node);
                                _dbg11("[Flat] <<<<<< [QUEUE] New Cone node {" << node << "}");
                            }
                        }
                        else
                        {
                            Node *new_cone = crn_above->emplaceNode(node, crn_above->min_g, NodeType::Cone, sgn_x);
                            if (sgn_y > 0)
                            {
                                new_cone->ray_neg = V2(sgn_x, 0);
                                new_cone->ray_pos = V2(sgn_x, y - crn->coord.y);
                            }
                            else
                            {
                                new_cone->ray_neg = V2(sgn_x, y - crn->coord.y);
                                new_cone->ray_pos = V2(sgn_x, 0);
                            }
                            updateHCost(new_cone, p_goal);
                            open_list.queue(new_cone);
                            _dbg11("[Flat] <<<<<< [QUEUE] New Cone node {" << new_cone << "}");
                        }
                        crn = nullptr;
                        is_oc_prev = is_oc;
                    }
                    else if (is_oc == false && is_oc_prev == true)
                    {
                        assert(crn == nullptr);
                        V2 pv(x, y);
                        kv = grid->coordtoKey<false>(pv);
                        crn = crns.try_emplace(kv, pv);
                        float_t new_g = node->g + y - root.y;
                        if (new_crn->min_g < new_g)
                        { // new_g is > minimum g at crn. stop flat node expansion
                            _dbg11("[Flat] Prepare to stop Flat Node search: New G$(" << new_g << ") from flat node root is >= crn min G$(" << new_crn->min_g << ")");
                            crn = nullptr;
                            to_stop = true;
                        }
                        else
                        {
                            crn->min_g = new_g; // update min g cost
                        }
                        is_oc_prev = is_oc;
                    }
                }

            } while (to_stop == false || crn_above != nullptr || crn_below != nullptr);
        }

        inline bool expandCone(Node *const &node, const V2 &p_goal, std::vector<V2> &path)
        {
            _dbgtitle("[Cone] Expand Cone Node { " << node << " }");
            _dbginc;
            assert(node->dx != 0);
            const V2 &root = node->crn->coord;

            // requires rays and dx to be resolved for current interval at dx.
            // ====================== Check if node's expanded interval intersects goal =============================
            if (node->dx + root.x == p_goal.x)
            {
                V2 y_gfr = p_goal.y - root.y; // y of vector goalFromRoot
                bool found_goal;
                const Ray &ray_neg = node->ray_neg;
                if (ray_neg.dy <= 0)                  // ray is going to -y or no change in y from root
                    found_goal = y_gfr >= ray_neg.dy; // the result of quotient (dx * ray.y / ray.x) is in +y direction of or on true value
                else                                  // ray is going to +y from root
                    found_goal = y_gfr > ray_neg.dy;  // the result of quotient is in -y direction of true value

                const Ray &ray_pos = node->ray_pos;
                if (ray_pos.dy >= 0)                   // ray is going to +y or no change in y from root
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
                    _dbgdec;
                    return true;
                }
            }

            // ======== Check if the next interval is out of map =============
            const int_t &dx = node->dx;
            const int_t x = node->dx + root.x;

            if (x == 0 || x == grid->getSize<true>().x)
            {
                _dbg11("[Cone] Next Interval is out of map ");
                _dbgdec;
                return true;
            }
            const int_t dx_next = node->dx + (node->dx < 0 ? -1 : 1);
            const int_t x_next = dx_next + root.x;

            // ====================== Add Turning Points on both (+y and -y) sides ======================
            // get the y rounded-to-zero (r2z) vertices near the edge of the current and next intervals
            Boundary boundary_neg(node->ray_neg, -1);
            Boundary boundary_pos(node->ray_pos, 1);

            bool has_flat_successors = false;
            for (Boundary &b : {boundary_neg, boundary_pos})
            {
                b.pv_cur = V2(dx, dx * b.ray.y / b.ray.x) + root;
                b.kv_cur = grid->coordToKey<false>(b.pv_cur);

                if (b.ray_dir >= 0)
                { // (ray is perpendicular or has tail interval)
                    V2 pc;
                    mapkey_t kc;

                    // check +dx, +sgn_y cell
                    dir_idx_t di = dirToDirIdx(dx, b.sgn_y);
                    grid->getCellKeyAndCoord(di, b.kv_cur, b.pv_cur, kc, pc);
                    if (grid->isAccessible(kc, pc) == true)
                    { // +dx, +sgn_y cell is free

                        // check -dx, +sgn_y cell
                        di = dirToDirIdx(-dx, sgn_y);
                        grid->getCellKeyAndCoord(di, b.kv_cur, b.pv_cur, kc, pc);
                        if (grid->isAccessible(kc, pc) == false)
                        { // -dx, +sgn_y cell is occupied

                            bool create_flat = true;
                            if constexpr (diag_block == true)
                            { // check +dx, -sgn_y cell
                                di = dirToDirIdx(dx, -sgn_y);
                                grid->getCellKeyAndCoord(di, b.kv_cur, b.pv_cur, kc, pc);
                                // create flat if +dx, +y cell is free
                                create_flat = grid->isAccessible(kc, pc) == true;
                            }

                            if (create_flat == true)
                            {
                                has_flat_successors = true;
                                Corner *new_crn = crns.try_emplace(b.kv_cur, b.pv_cur);
                                float_t new_g = node->g + norm(b.pv_cur, root);
                                if (new_crn->min_g > new_g)
                                {
                                    new_crn->min_g = new_g;
                                    Node *new_node = node->crn->emplaceNode(node, new_g, NodeType::Flat, sgn(dx));
                                    if (b.sgn_y < 0)
                                    {
                                        assert(new_node->ray_neg == 0);
                                        new_node->ray_pos = b.pv_cur - root;
                                    }
                                    else
                                    {
                                        assert(new_node->ray_pos == 0);
                                        new_node->ray_neg = b.pv_cur - root;
                                    }
                                    updateHCost(new_node, p_goal);
                                    open_list.queue(new_node);
                                    _dbg11("[Cone] <<<<<< [QUEUE] new flat node at neg {" << new_node << "}");
                                }
                                else
                                {
                                    _dbg11("[Cone] Flat node cannot be created at Vert(" << b.pv_cur << ") bcos new G$(" << new_g << ") >= cur G$(" << new_crn->min_g << ")");
                                }

                            } // create flat
                        }     // -dx, +sgn_y cell is occupied
                    }         // +dx +sgn_y cell is free
                }             // has turning point if +dx is same as ray.x
            }

            // ====================== Determine first and last cell to check for cell row after interval =======================
            {
                Boundary &b = boundary_neg;
                if (b.ray_dir > 0)
                { // has tail
                    // determine first cell by taking y of next row
                    int_t y_next = dx_next * b.ray->vec.y / b.ray->vec.x + root.y; // r2z coordinate (r2z is in +y direction)
                    --y_next;                                                      // to get the corresponding vertex at +dx +y of cell
                    const int_t y_map_bound = 0;
                    b.pv_bound = V2(b.pv_cur.x, (y_next < y_map_bound) ? y_map_bound : y_next);
                    b.kv_bound = grid->coordToKey<false>(b.pv_bound);
                }
                else
                { // perpendicular or no tail. determine first cell from current row
                    b.pv_bound = b.pv_cur;
                    b.kv_bound = b.kv_cur;
                }
                dir_idx_t di = dirToDirIdx(dx, 1);
                b.kc_bound = grid->addKeyToRelKey(b.kv_bound, grid->getCellRelKey(di, b.pv_bound.x));

                _dbg11("[Cone] First vertex(" << b.pv_bound << ")");
            }
            {
                Boundary &b = boundary_pos;
                assert(b.kv_cur == grid->coordToKey(b.pv_cur));

                if (b.ray_dir > 0)
                { // has tail
                    // determine first cell by taking y of next row
                    int_t y_next = dx_next * b.ray->vec.y / b.ray->vec.x + root.y; // r2z coordinate (r2z is in -y direction)
                    ++y_next;                                                      // so that the boundary lies outside of the sec
                    const int_t y_map_bound = grid->getBoundary<false>(2);
                    b.pv_bound = V2(b.pv_cur.x, (y_next > y_map_bound) ? y_map_bound : y_next);
                    b.kv_bound = grid->coordToKey<false>(b.pv_bound);
                }
                else
                {                          // perpendicular or no tail. determine first cell from current row
                    b.pv_bound = b.pv_cur; // lies outside of angular sec
                    b.kv_bound = b.kv_cur; // lies outside of sec
                }

                _dbg11("[Cone] Last vertex(" << b.pv_bound << ")");
            }

            assert(boundary_pos.pv_bound.y >= boundary_neg.pv_bound.y); // cannot be opposite under current assumptions

            // ---------- Scan cell interval --------------
            std::vector<Interval> intervals;
            {
                const int_t &y_last = boundary_pos.pv_bound.y;
                int_t y = boundary_neg.pv_bound.y;
                mapkey_t kc = grid->boundary_neg.kc_bound;
                const mapkey_t &rkc = grid->getRelKey<true>(2);
                bool scan_oc = true;
                while (1)
                {
                    if (scan_oc == true && grid->isOc(kc) == false)
                    { // found first free cell after scanning occupied cells. create new interval
                        assert(intervals.empty() == true || intervals.back().vert_pos.y >= 0);
                        intervals.emplace_back(x, y, root);
                        scan_oc = false;
                    }
                    else if (scan_oc == false && grid->isOc(kc) == true)
                    { // found first oc cell after scanning free cells. fill last interval
                        assert(intervals.empty() == false && intervals.back().vert_pos.y < 0);
                        intervals.back().vert_pos = V2(x, y);
                        intervals.back().diff_pos = V2(x, y) - root;
                        scan_oc = true;
                    }
                    ++y;
                    if (y >= last_y)
                        break;
                    kc = grid->addKeyToRelKey(kc, rkc);
                }
                if (intervals.back().vert_pos.y < 0) // unfilled
                {
                    intervals.back().vert_pos = V2(x, last_y);
                    intervals.back().diff_pos = V2(x, last_y) - root;
                }
            }

            // ================= Delete Intervals that lie outside the angular sector ==========================
            if (boundary_neg.ray_dir > 0)
            { // has tail at negative side
                for (auto itv_ = intervals.begin(); itv_ != intervals.end();)
                {
                    assert(itv_->diff_pos.x == dx);
                    if (sgn(dx) * det(node->ray_neg, itv_->diff_pos) <= 0) // interval lies beyond neg ray of node
                        itv_ = intervals.erase(itv_);
                    else
                        ++itv_;
                }
            }
            if (boundary_pos.ray_dir > 0)
            { // has tail at positive side
                for (auto itv_ = intervals.begin(); itv_ != intervals.end();)
                {
                    assert(itv_->diff_neg.x == dx);
                    if (sgn(dx) * det(itv_->diff_neg, node->ray_pos) <= 0)
                        itv_ = intervals.erase(itv_);
                    else
                        ++itv_;
                }
            }

            // ================= Process Intervals ==========================
            bool has_cone_successors = false;
            for (auto itv_ = intervals.begin(); itv_ != intervals.end();)
            {
                V2 &vert_neg = itv_->vert_neg;
                V2 &diff_neg = itv_->diff_neg;
                V2 &vert_pos = itv_->vert_pos;
                V2 &diff_pos = itv_->diff_pos;

                // ---- Adjust negative end's vert and diff from current row if applicable ----
                if (diff_neg.y < 0)
                {
                    diff_neg.x = dx_next;
                    vert_neg.x = x_next;
                    assert(node->ray_neg.y < 0);
                    if (sgn(dx) * det(node->ray_neg, diff_neg) <= 0)
                    { // interval neg vertex lies outside of node's negative ray
                        diff_neg = node->ray_neg;
                        vert_neg.y = root.y + dx_next * diff_neg.y / diff_neg.x;
                    }
                }
                else if (diff_neg.y == 0)
                {
                    diff_neg.x = dx_next;
                    vert_neg.x = x_next;
                }

                // ---- Adjust positive end's vert and diff from current row if applicable ----
                if (diff_pos.y > 0)
                {
                    diff_pos.x = dx_next;
                    vert_pos.x = x_next;
                    assert(node->ray_pos.vec.y > 0);
                    if (sgn(dx) * det(diff_pos, node->ray_pos) <= 0)
                    { // interval pos vertex lies outside of node's positive ray
                        diff_pos = node->ray_pos;
                        vert_pos.y = root.y + dx_next * diff_pos.y / diff_pos.x;
                    }
                }
                else if (diff_pos.y == 0)
                {
                    diff_pos.x = dx_next;
                    vert_pos.x = x_next;
                }

                // ---- Decide if cone node can be generated, and if the current node can continue expanding in this interval ----
                if (diff_neg.y > 0)
                { // cone node exists at negative side of interval
                    // test g-cost to see if cheaper to reach node
                    const V2 &new_root = vert_neg;
                    mapkey_t new_root_key = grid->coordToKey<false>(new_root);
                    Corner *new_crn = crns.try_emplace(new_root_key, new_root);
                    float_t new_g = node->g + norm(new_root, root);
                    if (new_crn->min_g > new_g)
                    {
                        new_crn->min_g = new_g;
                        has_cone_successors = true;
                        Node *new_node = node->crn->emplaceNode(node, new_g, NodeType::Cone, sgn(dx));
                        new_node->ray_neg = V2(new_node->dx, 0);

                        // adjust ray_pos
                        new_node->ray_pos = diff_neg;
                        int_t new_y = new_root.y + new_node->dx * diff_neg.y / diff_neg.x;
                        assert(vert_pos.x == x_next);
                        if (new_y > vert_pos.y) // this is okay bcos vert_pos is not adjusted by the node's ray_pos (bcos ray_pos can't cross ray_neg), so this condition is due to obstacle at vert_pos
                            new_node->ray_pos = V2(new_node->dx, vert_pos.y - new_root.y);

                        // queue
                        updateHCost(new_node, p_goal);
                        open_list.queue(new_node);
                        _dbg11("[Cone] <<<<<< [QUEUE] new cone node at neg {" << new_node << "}");
                    }
                    else
                    {
                        _dbg11("[Cone] Cone node cannot be created at VertNeg(" << vert_neg << ") bcos new G$(" << new_g << ") >= cur G$(" << new_crn->min_g << ")");
                    }

                    if (sgn(dx) * det(diff_neg, diff_pos) <= 0)
                    {
                        _dbg11("[Cone] No continuation from current root at interval");
                        itv_ = intervals.erase(itv_);
                    }
                    else
                    {
                        _dbg11("[Cone] Can continue from current root at interval");
                        // adjust vert_neg
                        vert_neg.x = x_next;
                        vert_neg.y = dx_next * diff_neg.y / diff_neg.x + root.y;
                        ++itv_;
                    }
                }
                else if (diff_pos.y < 0)
                { // cone node exists at positive side of interval

                    // test g-cost to see if cheaper to reach node
                    const V2 &new_root = vert_pos;
                    mapkey_t new_root_key = grid->coordToKey<false>(new_root);
                    Corner *new_crn = crns.try_emplace(new_root_key, new_root);
                    float_t new_g = node->g + norm(new_root, root);
                    if (new_crn->min_g > new_g)
                    {
                        new_crn->min_g = new_g;
                        has_cone_successors = true;
                        Node *new_node = node->crn->emplaceNode(node, new_g, NodeType::Cone, sgn(dx));
                        new_node->ray_pos = V2(new_node->dx, 0);

                        // adjust ray_neg
                        new_node->ray_neg = diff_pos;
                        int_t new_y = new_root.y + new_node->dx * diff_pos.y / diff_pos.x;
                        assert(vert_neg.x == x_next);
                        if (new_y < vert_neg.y) // this is okay bcos vert_pos is not adjusted by the node's ray_pos (bcos ray_pos can't cross ray_neg), so this condition is due to obstacle at vert_pos
                            new_node->ray_pos = V2(new_node->dx, vert_neg.y - new_root.y);

                        // queue
                        updateHCost(new_node, p_goal);
                        open_list.queue(new_node);
                        _dbg11("[Cone] <<<<<< [QUEUE] new cone node at pos {" << new_node << "}");
                    }
                    else
                    {
                        _dbg11("[Cone] Cone node cannot be created at VertPos(" << vert_pos << ") bcos new G$(" << new_g << ") >= cur G$(" << new_crn->min_g << ")");
                    }

                    if (sgn(dx) * det(diff_neg, diff_pos) <= 0)
                    {
                        _dbg11("[Cone] No continuation from current root at interval");
                        itv_ = intervals.erase(itv_);
                    }
                    else
                    {
                        _dbg11("[Cone] Can continue from current root at interval");
                        // adjust vert_pos
                        vert_pos.x = x_next;
                        vert_pos.y = dx_next * diff_pos.y / diff_pos.x + root.y;
                        ++itv_;
                    }
                }
                else
                { // no cone nodes
                    // continue as normal
                    _dbg11("[Cone] Continue at normal interval");
                    ++itv_;
                }
            }

            // ==================  Check if next row can be expanded by current root ==========================
            if (intervals.empty() == true)
            { // regardless of flat nodes being created
                _dbg11("[Cone] Next row is inaccessible by current root. Stop expansion.");
                // delete node and all applicable parents if possible
                _dbgdec;
                return true; // terminate
            }                // no intervals
            else if (has_flat_successors == false && has_cone_successors == false && intervals.size() == 1)
            {
                _dbgtitle("[Cone] Intermediate Pruning: Continue to next row as there is only one successor");
                _dbginc;
                Interval &interval = intervals.back();
                node->dx = dx_next;
                node->ray_neg = interval.diff_neg;
                node->ray_pos = interval.diff_pos;

                _dbgdec;
                _dbgdec;
                return false;
            } // no flat nodes created and no other successors
            else
            {
                _dbgtitle("[Cone] More than one successor:");
                _dbginc;
                assert(intervals.size() > 1 || has_flat_successors == true); // one interval and created flat nodes, or multiple intervals (regardless of flat nodes creation)

                bool used_current_node = false;
                for (const Interval &interval : intervals)
                {
                    _dbg11("[Cone] ---- Interval from VertexNeg( " << interval.vert_neg << " ) DiffNeg(" << interval.diff_neg << ")");
                    _dbg11("[Cone]                 to VertexPos( " << interval.vert_pos << " ) DiffPos(" << interval.diff_pos << ")");

                    if (used_current_node == true)
                    {
                        Node *new_node = node->crn->emplaceNode(node->parent, node->g, NodeType::Cone, dx_next);
                        new_node->dx = dx_next;
                        new_node->ray_neg = interval.diff_neg;
                        new_node->ray_pos = interval.diff_pos;
                        updateHCost(new_node, p_goal);
                        open_list.queue(new_node);
                        _dbg11("[Cone] <<<<<< [QUEUE] new node at root {" << new_node << "}");
                    }
                    else
                    {
                        node->dx = dx_next;
                        node->ray_neg = interval.diff_neg;
                        node->ray_pos = interval.diff_pos;
                        updateHCost(node, p_goal);
                        open_list.queue(node);
                        _dbg11("[Cone] <<<<<< [QUEUE] node at root {" << node << "}");
                    }
                }

                _dbgdec;
                _dbgdec;
                return true;
            } // more than one successor (cone node or flat node)
            assert(false);
        }

    public:
        ANYA2(Grid *const &grid) : grid(grid), los(grid)
        {
            setup();
        }
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
