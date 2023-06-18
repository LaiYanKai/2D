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

        inline void foundGoal(Node *node, std::vector<V2> &path, const V2 &p_goal) const
        {
            _dbgtitle("Found Goal!");
            _dbginc;
            assert(path.empty() == true);
            path.push_back(p_goal);
            _dbg10("Path { " << p_goal);
            do
            {
                path.push_back(node->crn->coord);
                _dbg00("; " << node->crn->coord);
                node = node->parent;
            } while (node != nullptr);
            _dbg01(" }");
            _dbgdec;
            return;
        }

        inline void updateHCost(Node *const &node, V2 p_goal) const
        {
            // flip around the root if dx != v_goalFromRoot.x

            if (node->type == NodeType::Cone)
            {

                int_t interval_x = node->crn->coord.x + node->dx;
                int_t dx_gfi = p_goal.x - interval_x;
                if (dx_gfi * sgn(node->dx) < 0)
                {
                    p_goal.x = interval_x - dx_gfi; // flip around the interval if "below" the interval
                    _dbg11("[HCost] Reflect goal around interval to (" << p_goal << ")");
                }

                assert(sgn(node->dx) * det(node->pos_ray, node->neg_ray) < 0);

                const V2 &root = node->crn->coord;
                V2 v_gfr = p_goal - root;
                if (sgn(node->dx) * det(v_gfr, node->pos_ray) < 0)
                { // h cost has to detour around node->pos_ray
                    V2f rootf(root);
                    V2f vert_pos(node->dx, float_t(node->dx) * node->pos_ray.y / node->pos_ray.x);
                    vert_pos += rootf;
                    node->h = norm(V2f(p_goal), vert_pos) + norm(vert_pos, rootf);
                    _dbg11("[HCost] Detour around +Ray (" << node->pos_ray << ") at vertex coordinate(" << vert_pos << ")");
                }
                else if (sgn(node->dx) * det(node->neg_ray, v_gfr) < 0)
                { // h cost has to detour around node->neg_ray
                    V2f rootf(root);
                    V2f vert_neg(node->dx, float_t(node->dx) * node->neg_ray.y / node->neg_ray.x);
                    vert_neg += rootf;
                    node->h = norm(V2f(p_goal), vert_neg) + norm(vert_neg, rootf);
                    _dbg11("[HCost] Detour around -Ray (" << node->neg_ray << ") at vertex coordinate(" << vert_neg << ")");
                }
                else // calculate h-cost directly
                    node->h = norm(p_goal, node->crn->coord);
            }    // cone type
            else // flat type
                node->h = norm(p_goal, node->crn->coord);
            node->f = node->g + node->h;
        }

        template <bool from_start>
        inline bool expandFlat(Node *const &node, const V2 &p_goal, std::vector<V2> &path)
        { // expand until cannot proceed anymore is reached
            const V2 &root = node->crn->coord;
            int_t y = root.y;

            _dbgtitle("[Flat:" << y << "] Node { " << node << " }");
            _dbginc;
            mapkey_t kv = node->crn->key;
            int_t x = root.x;
            const int_t sgn_y = node->neg_ray == 0 ? -1 : 1;
            dir_idx_t di = dirToDirIdx(0, sgn_y);
            int_t last_y = grid->getBoundary<false>(di);
            mapkey_t rcell_key = grid->getRelKey<true>(di);
            di = dirToDirIdx(1, sgn_y);
            mapkey_t cell_key_above = grid->addKeyToRelKey(kv, grid->getCellRelKey(di, x));
            di = dirToDirIdx(-1, sgn_y);
            mapkey_t cell_key_below = grid->addKeyToRelKey(kv, grid->getCellRelKey(di, x));

            Corner *crn_above = nullptr, *crn_below = nullptr;
            bool is_oc_above_prev = false;
            bool is_oc_below_prev = false;
            bool reused_node = false;
            if constexpr (from_start == true)
            { // for start case
                crn_above = node->crn;
                crn_below = node->crn;
                reused_node = true; // always create new nodes in case the rays get updated wrongly
            }
            else
            {
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
            }

            // move in sgn_y by one unit
            bool stopped_above = false, stopped_below = false;
            do
            {
                y += sgn_y;
                bool is_oc_above, is_oc_below;
                if (y == p_goal.y && x == p_goal.x)
                { // goal found
                    foundGoal(node, path, p_goal);
                    _dbgdec;
                    return true;
                }
                else if (y == last_y)
                {
                    is_oc_above = true;
                    is_oc_below = true;
                    stopped_above = true;
                    stopped_below = true;
                }
                else
                {
                    if (stopped_above == false)
                    {
                        cell_key_above = grid->addKeyToRelKey(cell_key_above, rcell_key);
                        is_oc_above = grid->isOc(cell_key_above);
                    }
                    if (stopped_below == false)
                    {
                        cell_key_below = grid->addKeyToRelKey(cell_key_below, rcell_key);
                        is_oc_below = grid->isOc(cell_key_below);
                    }

                    if constexpr (diag_block)
                    { // check diagonal blocking
                        if (is_oc_above != is_oc_below && is_oc_above_prev != is_oc_above && is_oc_below_prev != is_oc_below)
                        { // checkerboard
                            _dbg11("[Flat:" << y << "] Encountered checkerboard corner. Stop expansion");
                            is_oc_above = true;
                            is_oc_below = true;
                            stopped_above = true;
                            stopped_below = true;
                        }
                    }
                }

                for (const int_t &sgn_x : {-1, 1})
                {
                    bool &is_oc = sgn_x > 0 ? is_oc_above : is_oc_below;
                    bool &is_oc_prev = sgn_x > 0 ? is_oc_above_prev : is_oc_below_prev;
                    bool &stopped = sgn_x > 0 ? stopped_above : stopped_below;
                    Corner *&crn = sgn_x > 0 ? crn_above : crn_below;

                    if (is_oc == true && is_oc_prev == false)
                    { // expansion is going from free  cell to  occupied cell
                        assert(crn != nullptr);

                        if (reused_node == false && crn == node->crn)
                        { // have not converted the flat node to cone node

                            assert(from_start == true || sgn(node->dx) == sgn_x);
                            V2 &ray_to = sgn_y > 0 ? node->pos_ray : node->neg_ray;
                            ray_to = V2(sgn_x, y - crn->coord.y);

                            node->type = NodeType::Cone;
                            if (sgn_x * det(node->neg_ray, node->pos_ray) <= 0)
                                _dbg11("[Flat:" << y << "] Cannot queue cone node with root at parent node because the next row is inaccessible");
                            else
                            {
                                updateHCost(node, p_goal);
                                open_list.queue(node);
                                _dbg11("[Flat:" << y << "] <<<<<< [QUEUE] Reused flat node as cone node {" << node << "}");
                            }
                            reused_node = true; // mark it bcos no longer possible to have next cone node at root (for non-start case. No issues with start case)
                        }
                        else
                        {
                            Node *new_cone;
                            if constexpr (from_start == true)
                                new_cone = crn->emplaceNode(crn == node->crn ? nullptr : node, 0, NodeType::Cone, sgn_x); // avoid pointing to start flat node if cone node is on the start point
                            else
                                new_cone = crn->emplaceNode(node, crn->min_g, NodeType::Cone, sgn_x);

                            if (sgn_y > 0)
                            {
                                new_cone->neg_ray = V2(sgn_x, 0);
                                new_cone->pos_ray = V2(sgn_x, y - crn->coord.y);
                            }
                            else
                            {
                                new_cone->neg_ray = V2(sgn_x, y - crn->coord.y);
                                new_cone->pos_ray = V2(sgn_x, 0);
                            }
                            updateHCost(new_cone, p_goal);
                            open_list.queue(new_cone);
                            _dbg11("[Flat:" << y << "] <<<<<< [QUEUE] New Cone node {" << new_cone << "}");
                        }
                        crn = nullptr;
                        is_oc_prev = is_oc;
                    }
                    else if (is_oc == false && is_oc_prev == true)
                    { // expansion is going from occupied cell to free cell
                        assert(crn == nullptr);
                        V2 pv(x, y);
                        kv = grid->coordToKey<false>(pv);
                        crn = crns.try_emplace(kv, pv);
                        float_t new_g = node->g + y - root.y;
                        if (crn->min_g < new_g)
                        { // new_g is > minimum g at crn. stop flat node expansion
                            _dbg11("[Flat:" << y << "] Prepare to stop Flat Node search: New G$(" << new_g << ") from flat node root is >= crn min G$(" << crn->min_g << ")");
                            crn = nullptr;
                            stopped = true;
                        }
                        else
                        {
                            _dbg11("[Flat:" << y << "] New cheapest-$G node can be created at root (" << x << "," << y << ") with dx(" << sgn_x << ")");
                            crn->min_g = new_g; // update min g cost
                        }
                        is_oc_prev = is_oc;
                    }
                }

            } while (crn_above != nullptr || crn_below != nullptr);
            _dbgdec;
            return false;
        }

        inline bool _expandConeFoundGoal(Cone &cone, const V2 &p_goal, std::vector<V2> &path) const
        {
            if (cone.x() == p_goal.x)
            {
                bool found_goal;
                if (cone.negRayHasTail() == true || cone.negRayIsPerp() == true) // ray is going to -y or no change in y from root
                    found_goal = p_goal.y >= b_neg.y_cur;                        // the result of quotient (dx * ray.y / ray.x) is in +y direction of or on true value
                else                                                             // ray is going to +y from root
                    found_goal = p_goal.y > b_neg.y_cur;                         // the result of quotient is in -y direction of true value

                if (cone.posRayHasTail() == true || cone.posRayIsPerp() == true) // ray is going to +y or no change in y from root
                    found_goal &= p_goal.y <= b_pos.y_cur;                       // result of quotient is in -y direction of or on true value
                else                                                             // ray is going to -y from root
                    found_goal &= p_goal.y < b_pos.y_cur;                        // the result of quotient is in +y direction of true value

                if (found_goal == true)
                { // terminate and find path if goal intersects interval
                    foundGoal(cone.node, path, p_goal);
                    return true;
                }
            }
            return false;
        }

        inline bool _expandConeFindFlatSuccessors(Cone &cone, const V2 &p_goal, std::vector<V2> &path)
        {
            for (const int_t &sgn_y : {-1, 1})
            {
                if (cone.rayIsPerp(sgn_y) == true || cone.rayHasTail(sgn_y) == true)
                {                                                  // (ray is perpendicular or has tail interval)
                    const V2 vert_coord = cone.rayCurCoord(sgn_y); // r2z vertex at cur row

                    // check +dx, +sgn_y cell
                    dir_idx_t di = dirToDirIdx(dx, sgn_y);
                    V2 cell_coord = vert_coord + grid->getCellRelCoord(di);
                    if (grid->isAccessible(cell_coord) == true)
                    { // +dx, +sgn_y cell is free

                        // check -dx, +sgn_y cell
                        di = dirToDirIdx(-dx, sgn_y);
                        cell_coord = vert_coord + grid->getCellRelCoord(di);
                        if (grid->isAccessible(cell_coord) == false)
                        { // -dx, +sgn_y cell is occupied

                            // Determine if flat successor can be created (check diagonal block):
                            bool create_flat = true;
                            if constexpr (diag_block == true)
                            { // check +dx, -sgn_y cell
                                di = dirToDirIdx(dx, -sgn_y);
                                cell_coord = vert_coord + grid->getCellRelCoord(di);
                                // create flat if +dx, +y cell is free
                                create_flat = grid->isAccessible(cell_coord) == true;
                            }

                            // Create flat successor if can be created:
                            if (create_flat == true)
                            {
                                Corner *new_crn = crns.try_emplace(vert_key, vert_coord);
                                float_t new_g = node->g + norm(vert_coord, cone.root());
                                if (new_crn->min_g > new_g)
                                {
                                    new_crn->min_g = new_g;
                                    Node *new_node = new_crn->emplaceNode(node, new_g, NodeType::Flat, sgn_dx);
                                    if (sgn_y < 0)
                                    {
                                        assert(new_node->neg_ray == 0);
                                        new_node->pos_ray = vert_coord - cone.root();
                                    }
                                    else
                                    {
                                        assert(new_node->pos_ray == 0);
                                        new_node->neg_ray = vert_coord - cone.root();
                                    }
                                    updateHCost(new_node, p_goal);
                                    open_list.queue(new_node);
                                    _dbg11("[Cone] <<<<<< [QUEUE] new flat node at neg {" << new_node << "}");
                                    return true;
                                }
                                else
                                {
                                    _dbg11("[Cone] Flat node cannot be created at Vert(" << vert_coord << ") bcos new G$(" << new_g << ") >= cur G$(" << new_crn->min_g << ")");
                                }
                            } // create flat
                        }     // -dx, +sgn_y cell is occupied
                    }         // +dx +sgn_y cell is free
                }             // has turning point if +dx is same as ray.x
            }
            return false;
        }
        inline bool expandCone(Node *const &node, const V2 &p_goal, std::vector<V2> &path)
        {
            _dbgtitle("[Cone] Expand Cone Node { " << node << " }");
            _dbginc;

            // Cone class stores the node and performs simple calculations. -O3 should cache all repeated calcs.
            Cone cone(node);

            while (1)
            {
                _dbgtitle("[Cone:" << node->dx << "] { " << node << " }");
                _dbginc;

                // ====================== Check if node's expanded interval intersects goal =============================
                if (_expandConeFindFlatSuccessors(cone, p_goal, path))
                {
                    _dbgdec;
                    _dbgdec;
                    return true;
                }

                // ======== Check if the next interval is out of map =============
                if (x == 0 || x == grid->getSize<true>().x)
                {
                    _dbg11("[Cone] Next Interval is out of map ");
                    _dbgdec;
                    break;
                }

                // ====================== Add Turning Points on both (+y and -y) sides ======================
                bool has_flat_successors = _expandConeFlatSuccessors(b_neg, b_pos, node);

                // ====================== Determine first and last cell to check for cell row after interval =======================
                int_t y_begin;
                if (cone.negRayHasTail() == true)
                {
                    y_begin = cone.negRayNextY() - 1;           // so that the cell is always in +dx +y direction of vertex
                    int_t y_bound = grid->getBoundary<true>(6); // should be zero
                    if (y_begin < y_bound)
                        y_begin = y_bound;
                }
                else // neg ray has no tail
                    y_begin = cone.negRayCurY();
                int_t y_end;
                if (cone.posRayHasTail() == true)
                {
                    y_end = cone.posRayNextY() + 1; // to exclude this last vertex
                    int_t y_bound = grid->getBoundary<true>(2);
                    if (y_end > y_bound)
                        y_end = y_bound;
                }
                else // pos ray has no tail
                    y_end = cone.posRayCurY();
                _dbg11("[Cone] Scan from YBegin(incl., vert)(" << y_begin << ") to YEnd(not incl., vert)(" << y_end << ")");
                assert(y_end > y_begin); // cannot be opposite under current assumptions

                dir_idx_t di = dirToDirIdx(cone.dx(), 1);
                V2 vert_coord(cone.x(), y_begin);
                mapkey_t cell_key = grid->addKeyToRelKey(grid->coordToKey<false>(vert_coord), grid->getCellRelKey(di));
                mapkey_t rel_cell_key = grid->getRelKey<true>(2);

                std::vector<Interval> intervals;
                bool scanning_oc = false;
                // insert interval starting from cone's -ray
                if (grid->isOc(cell_key) == false)
                    intervals.emplace_back(cone.negRay(), V2(0, 0));

                while (true)
                {
                    ++vert_coord.y;
                    if (vert_coord.y >= y_end)
                        break;
                    cell_key = grid->addKeyToRelKey(cell_key, rel_cell_key);

                    // occupancy of cell in +dx, +y direction
                    const bool is_oc = grid->isOc(cell_key);

                    if (scanning_oc == true && is_oc == false)
                    { // found first free cell after scanning occupied cells. create new interval
                        // [x] ---(+y)---> [ ]
                        assert(intervals.empty() == true || intervals.back().pos_ray != 0);
                        const V2 new_neg_ray = vert_coord - cone.root();
                        scanning_oc = false;

                        if (new_neg_ray.y <= 0) // ray can point to next row
                            new_neg_ray.x += cone.sgnX();

                        // if beyond pos_ray, stop scan, don't insert interval
                        if (cone.sgnX() * det(new_neg_ray, cone.posRay()) <= 0)
                            break; // later cells will not result in any nodes.
                        else
                            intervals.emplace_back(new_neg_ray, V2(0, 0)); // if not beyond pos_ray, insert interval
                    }
                    else if (scanning_oc == false && is_oc == true)
                    { // found first oc cell after scanning free cells. fill last interval
                        // [ ] ---(+y)---> [x]
                        assert(intervals.empty() == false && intervals.back().pos_ray == 0);
                        const V2 new_pos_ray = vert_coord - cone.root();
                        scanning_oc = true;

                        if (new_pos_ray.y >= 0) // can point to next row
                            new_pos_ray += cone.sgnX();

                        // test if beyond ray neg, if it is,  delete interval
                        if (cone.sgnX() * det(cone.negRay(), new_pos_ray) <= 0)
                            intervals.pop_back();
                        else
                            intervals.back().pos_ray = new_pos_ray;
                    }
                }
                // fill the ray if there is remaining pos_ray
                if (intervals.empty() == false && intervals.back().pos_ray == 0)
                    intervals.back().pos_ray = cone.posRay();

                // ---------- Scan cell interval --------------
                std::vector<Interval> intervals;
                {
                    const int_t &last_y = b_pos.pv_bound.y;
                    int_t y = b_neg.pv_bound.y;
                    mapkey_t cell_key = b_neg.cell_key_bound;
                    const mapkey_t &rcell_key = grid->getRelKey<true>(2);
                    bool scan_oc = grid->isOc(cell_key);
                    // place interval for starting from free
                    if (scan_oc == false)
                        intervals.emplace_back(V2(x, y), b_neg.ray);

                    while (1)
                    {
                        ++y;
                        if (y >= last_y)
                            break;
                        cell_key = grid->addKeyToRelKey(cell_key, rcell_key);

                        if (scan_oc == true && is_oc == false)
                        { // found first free cell after scanning occupied cells. create new interval
                            assert(intervals.empty() == true || intervals.back().vert_pos.y >= 0);
                            const V2 vert_neg(x, y);
                            intervals.emplace_back(vert_neg, vert_neg - root);
                            scan_oc = false;

                            // test if beyond pos_ray, if it is, stop scan, don't emplace back

                            // upgrade
                        }
                        else if (scan_oc == false && is_oc == true)
                        { // found first oc cell after scanning free cells. fill last interval
                            assert(intervals.empty() == false && intervals.back().vert_pos.y < 0);
                            intervals.back().vert_pos = V2(x, y);
                            intervals.back().diff_pos = V2(x, y) - root;
                            scan_oc = true;

                            // test if beyond ray neg, if it is,  delete interval
                        }
                    }
                    if (intervals.empty() == false && intervals.back().vert_pos.y < 0) // unfilled
                    {
                        intervals.back().vert_pos = V2(x, last_y);
                        intervals.back().diff_pos = b_pos.ray; // V2(x, last_y) - root;
                    }
                }

                // ================= Delete Intervals that lie outside the angular sector ==========================
                if (b_neg.ray_dir > 0)
                { // has tail at negative side
                    for (auto itv_ = intervals.begin(); itv_ != intervals.end();)
                    {
                        assert(sgn(itv_->diff_pos.x) == sgn(dx));
                        if (sgn(dx) * det(node->neg_ray, itv_->diff_pos) <= 0) // interval lies beyond neg ray of node
                            itv_ = intervals.erase(itv_);
                        else
                            ++itv_;
                    }
                }
                if (b_pos.ray_dir > 0)
                { // has tail at positive side
                    for (auto itv_ = intervals.begin(); itv_ != intervals.end();)
                    {
                        assert(sgn(itv_->diff_neg.x) == sgn(dx));
                        if (sgn(dx) * det(itv_->diff_neg, node->pos_ray) <= 0)
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
                        assert(node->neg_ray.y < 0);
                        if (sgn(dx) * det(node->neg_ray, diff_neg) <= 0)
                        { // interval neg vertex lies outside of node's negative ray
                            diff_neg = node->neg_ray;
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
                        assert(node->pos_ray.y > 0);
                        if (sgn(dx) * det(diff_pos, node->pos_ray) <= 0)
                        { // interval pos vertex lies outside of node's positive ray
                            diff_pos = node->pos_ray;
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
                            Node *new_node = new_crn->emplaceNode(node, new_g, NodeType::Cone, sgn(dx));
                            new_node->neg_ray = V2(new_node->dx, 0);

                            // adjust pos_ray
                            new_node->pos_ray = diff_neg;
                            int_t new_y = new_root.y + new_node->dx * diff_neg.y / diff_neg.x;
                            assert(vert_pos.x == x_next);
                            if (new_y > vert_pos.y) // this is okay bcos vert_pos is not adjusted by the node's pos_ray (bcos pos_ray can't cross neg_ray), so this condition is due to obstacle at vert_pos
                                new_node->pos_ray = V2(new_node->dx, vert_pos.y - new_root.y);

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
                            Node *new_node = new_crn->emplaceNode(node, new_g, NodeType::Cone, sgn(dx));
                            new_node->pos_ray = V2(new_node->dx, 0);

                            // adjust neg_ray
                            new_node->neg_ray = diff_pos;
                            int_t new_y = new_root.y + new_node->dx * diff_pos.y / diff_pos.x;
                            assert(vert_neg.x == x_next);
                            if (new_y < vert_neg.y) // this is okay bcos vert_pos is not adjusted by the node's pos_ray (bcos pos_ray can't cross neg_ray), so this condition is due to obstacle at vert_pos
                                new_node->pos_ray = V2(new_node->dx, vert_neg.y - new_root.y);

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
                    break;
                } // no intervals
                else if (has_flat_successors == false && has_cone_successors == false && intervals.size() == 1)
                {
                    _dbgtitle("[Cone] Intermediate Pruning: Continue to next row as there is only one successor");
                    _dbginc;
                    Interval &interval = intervals.back();
                    node->dx = dx_next;
                    node->neg_ray = interval.diff_neg;
                    node->pos_ray = interval.diff_pos;
                    _dbgdec;
                } // no flat nodes created and no other successors
                else
                {
                    _dbgtitle("[Cone] More than one successor:");
                    _dbginc;
                    // one interval and created flat/successor nodes, or multiple intervals (regardless of flat nodes creation)
                    assert((intervals.size() <= 1 && (has_flat_successors || has_cone_successors)) ||
                           intervals.size() > 1);

                    bool used_current_node = false;
                    for (const Interval &interval : intervals)
                    {
                        _dbg11("[Cone] ---- Interval from VertexNeg( " << interval.vert_neg << " ) DiffNeg(" << interval.diff_neg << ")");
                        _dbg11("[Cone]                 to VertexPos( " << interval.vert_pos << " ) DiffPos(" << interval.diff_pos << ")");

                        if (used_current_node == true)
                        {
                            Node *new_node = node->crn->emplaceNode(node->parent, node->g, NodeType::Cone, dx_next);
                            new_node->dx = dx_next;
                            new_node->neg_ray = interval.diff_neg;
                            new_node->pos_ray = interval.diff_pos;
                            updateHCost(new_node, p_goal);
                            open_list.queue(new_node);
                            _dbg11("[Cone] <<<<<< [QUEUE] new node at root {" << new_node << "}");
                        }
                        else
                        {
                            node->dx = dx_next;
                            node->neg_ray = interval.diff_neg;
                            node->pos_ray = interval.diff_pos;
                            updateHCost(node, p_goal);
                            open_list.queue(node);
                            _dbg11("[Cone] <<<<<< [QUEUE] node at root {" << node << "}");
                            used_current_node = true;
                        }
                    }

                    _dbgdec;
                    _dbgdec;
                    break;
                } // more than one successor (cone node or flat node)
                _dbgdec;
            }
            _dbgdec;
            return false;
        }

    public:
        ANYA2(Grid *const &grid) : grid(grid), los(grid) {}
        ANYA2 &operator=(const ANYA2 &) = delete; // Disallow copying
        ANYA2(const ANYA2 &) = delete;
        ~ANYA2() {}

        std::vector<V2> run(const V2 &p_start, const V2 &p_goal)
        {
            mapkey_t k_start = grid->coordToKey<false>(p_start);
            Corner *crn_start = crns.try_emplace(k_start, p_start);
            crn_start->min_g = 0;
            Node *node_start_pos = crn_start->emplaceNode(nullptr, 0, NodeType::Flat, 0);
            node_start_pos->neg_ray = V2(1, 0); // not used, just to indicate direction of flat expansion
            Node *node_start_neg = crn_start->emplaceNode(nullptr, 0, NodeType::Flat, 0);
            node_start_neg->pos_ray = V2(1, 0); // not used, just to indicate direction of flat expansion

            // Expand flat nodes and find cone nodes from start
            std::vector<V2> path = {};
            if (expandFlat<true>(node_start_pos, p_goal, path) == false)     // -y expansion did not find goal
                if (expandFlat<true>(node_start_neg, p_goal, path) == false) // +y expansion did not find goal
                    while (true)
                    {
                        // ====== Poll Node ======
                        Node *node = open_list.poll();

                        // ====== Openlist empty ======
                        if (node == nullptr)
                            break; // no path found;

                        _dbg11(">>>>>> [POLL] F$(" << node->f << ") Node {" << node << "} ");

                        // ======= Flat or Cone node ======
                        if (node->type == NodeType::Cone)
                        {
                            if (expandCone(node, p_goal, path) == true)
                                break; // found path
                        }
                        else
                        {
                            if (expandFlat<false>(node, p_goal, path) == true)
                                break;
                        }
                    }

            // ==== Remove nodes  ====
            open_list.clear();
            crns.clear();

            return path;
        }
    };
}
