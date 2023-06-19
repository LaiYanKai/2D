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

        inline bool expandStart(Node *const &node, const int_t &sgn_y, const V2 &p_goal, std::vector<V2> &path)
        {
            V2 vert_coord = node->crn->coord;
            dir_idx_t di = dirToDirIdx(0, sgn_y);
            const mapkey_t rel_cell_key = grid->getRelKey<true>(di);
            const int_t &y_bound = grid->getBoundary<false>(di);

            // ====== Get parameters for cells above ===========
            di = dirToDirIdx(1, sgn_y);
            mapkey_t cella_key = grid->coordToKey<true>(vert_coord + grid->getCellRelCoord(di));
            const bool has_a = vert_coord.x != grid->getBoundary<false>(0);
            Corner *crn_a = nullptr;
            bool scanninga_oc = true;

            // ====== Get parameters for cells below ===========
            di = dirToDirIdx(-1, sgn_y);
            mapkey_t cellb_key = grid->coordToKey<true>(vert_coord + grid->getCellRelCoord(di));
            const bool has_b = vert_coord.x != grid->getBoundary<false>(4);
            Corner *crn_b = nullptr;
            bool scanningb_oc = true;

            while (true)
            {
                if (vert_coord == p_goal)
                {
                    foundGoal(node, path, p_goal);
                    return true;
                }

                // ========= Get Occupancy information cells, map boundary, and if diag_block, if the current position is on a checkerboard corner ==============
                bool a_oc = true, b_oc = true;
                if (vert_coord.y != y_bound)
                { // have not reached end
                    // ------ Get Occupancy of cell above ----------
                    if (has_a == true)
                    {
                        cella_key = grid->addKeyToRelKey(cella_key, rel_cell_key);
                        a_oc = grid->isOc(cella_key);
                    }
                    // ------ Get Occupancy of cell below ----------
                    if (has_b == true)
                    {
                        cellb_key = grid->addKeyToRelKey(cellb_key, rel_cell_key);
                        b_oc = grid->isOc(cellb_key);
                    }
                    // ------ Check if diagonally blocked ----------
                    if (diag_block == true && scanninga_oc != scanningb_oc && a_oc != b_oc && a_oc != scanninga_oc)
                    { // checkerboard corner
                        a_oc = true;
                        b_oc = true;
                    }
                }

                // ============= Create cone nodes =================
                for (const int_t &sgn_x : {-1, 1})
                {
                    bool &scanning_oc = sgn_x < 0 ? scanningb_oc : scanninga_oc;
                    bool &oc = sgn_x < 0 ? b_oc : a_oc;
                    Corner *&crn = sgn_x < 0 ? crn_b : crn_a;

                    if (scanning_oc == true && oc == false)
                    { // [x] --(sgn_y)--> [ ]
                        assert(crn == nullptr);
                        crn = crns.try_emplace(grid->coordToKey<false>(vert_coord), vert_coord);
                        scanning_oc = false;
                    }
                    else if (scanning_oc == false && oc == true)
                    { // [ ] --(sgn_y)--> [x]
                        // generate new cone node
                        assert(crn != nullptr);
                        crn->min_g = crn->coord.y - node->crn->coord.y;
                        Node *new_node = crn->emplaceNode(crn == node->crn ? nullptr : node, crn->min_g, NodeType::Cone, sgn_x);
                        new_node->neg_ray = sgn_y < 0 ? V2(sgn_x, vert_coord.y - crn->coord.y) : V2(sgn_x, 0);
                        new_node->pos_ray = sgn_y < 0 ? V2(sgn_x, 0) : V2(sgn_x, vert_coord.y - crn->coord.y);
                        updateHCost(new_node, p_goal);
                        open_list.queue(new_node);
                        _dbg11("[Start] <<<<<< [QUEUE] Cone Node {" << new_node << "}");
                        crn = nullptr;
                        scanning_oc = true;
                    }
                }

                if (a_oc == true && b_oc == true)
                    break;
                vert_coord.y += sgn_y;
            }
            return false;
        }
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

                            assert(sgn(node->dx) == sgn_x);
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
                            Node *new_cone = crn->emplaceNode(node, crn->min_g, NodeType::Cone, sgn_x);
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
                    found_goal = p_goal.y >= cone.negRayCurY();                  // the result of quotient (dx * ray.y / ray.x) is in +y direction of or on true value
                else                                                             // ray is going to +y from root
                    found_goal = p_goal.y > cone.negRayCurY();                   // the result of quotient is in -y direction of true value

                if (cone.posRayHasTail() == true || cone.posRayIsPerp() == true) // ray is going to +y or no change in y from root
                    found_goal &= p_goal.y <= cone.posRayCurY();                 // result of quotient is in -y direction of or on true value
                else                                                             // ray is going to -y from root
                    found_goal &= p_goal.y < cone.posRayCurY();                  // the result of quotient is in +y direction of true value

                if (found_goal == true)
                { // terminate and find path if goal intersects interval
                    foundGoal(cone.node, path, p_goal);
                    return true;
                }
            }
            return false;
        }

        inline bool _expandConeFindFlatSuccessors(Cone &cone, const V2 &p_goal)
        {
            if (cone.root() == V2(165,305) && cone.dx() == 27)
                _dbghelp;
            bool has_flat_successor = false;
            for (const int_t &sgn_y : {-1, 1})
            {
                if (cone.rayIsPerp(sgn_y) == true || cone.rayHasTail(sgn_y) == true)
                {                                                  // (ray is perpendicular or has tail interval)
                    const V2 vert_coord = cone.rayCurCoord(sgn_y); // r2z vertex at cur row

                    // check +dx, +sgn_y cell
                    dir_idx_t di = dirToDirIdx(cone.dx(), sgn_y);
                    V2 cell_coord = vert_coord + grid->getCellRelCoord(di);
                    if (grid->isAccessible(cell_coord) == true)
                    { // +dx, +sgn_y cell is free

                        // check -dx, +sgn_y cell
                        di = dirToDirIdx(-cone.dx(), sgn_y);
                        cell_coord = vert_coord + grid->getCellRelCoord(di);
                        if (grid->isAccessible(cell_coord) == false)
                        { // -dx, +sgn_y cell is occupied

                            // Determine if flat successor can be created (check diagonal block):
                            bool create_flat = true;
                            if constexpr (diag_block == true)
                            { // check +dx, -sgn_y cell
                                di = dirToDirIdx(cone.dx(), -sgn_y);
                                cell_coord = vert_coord + grid->getCellRelCoord(di);
                                // create flat if +dx, +y cell is free
                                create_flat = grid->isAccessible(cell_coord) == true;
                            }

                            // Create flat successor if can be created:
                            if (create_flat == true)
                            {
                                mapkey_t vert_key = grid->coordToKey<false>(vert_coord);
                                Corner *new_crn = crns.try_emplace(vert_key, vert_coord);
                                float_t new_g = cone.node->g + norm(vert_coord, cone.root());
                                if (new_crn->min_g > new_g)
                                {
                                    new_crn->min_g = new_g;
                                    Node *new_node = new_crn->emplaceNode(cone.node, new_g, NodeType::Flat, cone.sgnX());
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
                                    has_flat_successor = true;
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
            return has_flat_successor;
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
                if (_expandConeFoundGoal(cone, p_goal, path))
                {
                    _dbgdec;
                    _dbgdec;
                    return true;
                }

                // ======== Check if the next interval is out of map =============
                if (cone.x() == grid->getBoundary<false>(4) || cone.x() == grid->getBoundary<false>(0))
                {
                    _dbg11("[Cone] Next Interval is out of map ");
                    _dbgdec;
                    break;
                }

                // ====================== Add Turning Points on both (+y and -y) sides ======================
                bool has_flat_successors = _expandConeFindFlatSuccessors(cone, p_goal);

                // ====================== Determine first and last cell to check for cell row after interval =======================
                int_t y_begin;
                bool neg_out_of_map = false;
                if (cone.negRayHasTail() == true)
                {
                    y_begin = cone.negRayNextY() - 1;           // so that the cell is always in +dx +y direction of vertex
                    int_t y_bound = grid->getBoundary<true>(6); // should be zero
                    neg_out_of_map = y_begin < y_bound;
                    if (neg_out_of_map == true)
                        y_begin = y_bound;
                }
                else // neg ray has no tail
                    y_begin = cone.negRayCurY();
                int_t y_end;
                bool pos_out_of_map = false;
                if (cone.posRayHasTail() == true)
                {
                    y_end = cone.posRayNextY() + 1; // to exclude this last vertex
                    int_t y_bound = grid->getBoundary<true>(2);
                    pos_out_of_map = y_end > y_bound;
                    if (pos_out_of_map == true)
                        y_end = y_bound;
                }
                else // pos ray has no tail
                    y_end = cone.posRayCurY();
                _dbg11("[Cone] Scan from YBegin(incl., vert)(" << y_begin << ") to YEnd(not incl., vert)(" << y_end << ")");
                assert(y_end > y_begin); // cannot be opposite under current assumptions

                dir_idx_t di = dirToDirIdx(cone.dx(), 1);
                V2 vert_coord(cone.x(), y_begin);
                mapkey_t cell_key = grid->addKeyToRelKey(grid->coordToKey<false>(vert_coord), grid->getCellRelKey(di, vert_coord.x));
                mapkey_t rel_cell_key = grid->getRelKey<true>(2);

                // =============== Scan for intervals from first to last cell, deleting intervals with no successors, and modify rows to next row  ===================
                std::vector<Interval> intervals;
                bool scanning_oc = grid->isOc(cell_key);
                // insert interval starting from cone's -ray
                if (scanning_oc == false)
                {
                    if (neg_out_of_map == true)
                    {
                        const int_t &y_bound = grid->getBoundary<true>(6);
                        intervals.emplace_back(V2(cone.xNext(), y_bound), V2(0, 0), 0b00);
                    }
                    else
                        intervals.emplace_back(cone.negRay(), V2(0, 0), 0b10);
                }

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
                        V2 new_neg_ray = vert_coord - cone.root();
                        scanning_oc = false;

                        if (new_neg_ray.y <= 0) // ray can point to next row
                            new_neg_ray.x += cone.sgnX();

                        // if beyond pos_ray, stop scan, don't insert interval
                        if (cone.sgnX() * det(new_neg_ray, cone.posRay()) <= 0)
                            break; // later cells will not result in any nodes.
                        else
                            intervals.emplace_back(new_neg_ray, V2(0, 0), 0b00); // if not beyond pos_ray, insert interval
                    }
                    else if (scanning_oc == false && is_oc == true)
                    { // found first oc cell after scanning free cells. fill last interval
                        // [ ] ---(+y)---> [x]
                        assert(intervals.empty() == false && intervals.back().pos_ray == 0);
                        V2 new_pos_ray = vert_coord - cone.root();
                        scanning_oc = true;

                        if (new_pos_ray.y >= 0) // can point to next row
                            new_pos_ray.x += cone.sgnX();

                        // if the new pos ray is beyond or parallel to ray neg,  delete the interval
                        if (cone.sgnX() * det(cone.negRay(), new_pos_ray) <= 0)
                            intervals.pop_back(); // no cone successor can be genereated
                        else
                            intervals.back().pos_ray = new_pos_ray;
                    }
                }
                // fill the ray if there is remaining pos_ray
                if (intervals.empty() == false && intervals.back().pos_ray == 0)
                {
                    if (pos_out_of_map == true)
                    {
                        const int_t &y_bound = grid->getBoundary<true>(2);
                        intervals.back().pos_ray = V2(cone.xNext(), y_bound);
                    }
                    else
                    {
                        intervals.back().pos_ray = cone.posRay();
                        intervals.back().position |= 0b01;
                    }
                }

                // =================== Generate Cone Successors and Remove intervals where the current node cannot continue  ===========================
                bool has_cone_successors = false;
                // All rays of intervals are now pointing to the next row (except if they are -ray and +ray of nodes and not out-of-map)
                for (auto interval_ = intervals.begin(); interval_ != intervals.end();)
                {
                    Interval &interval = *interval_;
                    bool remove_interval = cone.sgnX() * det(interval.neg_ray, interval.pos_ray) <= 0;

                    // ------- Generate cone at interval -ray (if not at node -ray) --------------
                    if ((interval.position == 0b01 || interval.position == 0b00) && interval.neg_ray.y > 0)
                    { // pos ray is modified to next row
                        assert(interval.pos_ray.y > 0);

                        V2 vert_coord = cone.root() + interval.neg_ray;
                        Corner *new_crn = crns.try_emplace(grid->coordToKey<false>(vert_coord), vert_coord);
                        float_t new_g = node->g + interval.neg_ray.norm();
                        if (new_crn->min_g > new_g)
                        {
                            new_crn->min_g = new_g;
                            Node *new_node = new_crn->emplaceNode(node, new_g, NodeType::Cone, cone.sgnX());
                            new_node->neg_ray = V2(cone.sgnX(), 0);
                            new_node->pos_ray = interval.neg_ray;

                            if (remove_interval == true)
                            { // delete this interval bcos cannot access the next row from root
                                assert(interval.pos_ray.x == cone.dxNext());
                                // for this to occur, the +ray of interval has to be adjusted beyond the -ray of interval, to the next x;
                                // the pos ray has to point to the next x from the root
                                new_node->pos_ray = interval.pos_ray - interval.neg_ray;
                            }

                            // queue
                            updateHCost(new_node, p_goal);
                            open_list.queue(new_node);
                            _dbg11("[Cone] <<<<<< [QUEUE] new cone node at neg {" << new_node << "}");
                            has_cone_successors = true;
                        }
                        else
                            _dbg11("[Cone] Cone node cannot be created at (" << vert_coord << ") bcos new G$(" << new_g << ") >= cur G$(" << new_crn->min_g << ")");
                    }
                    // ------- Generate cone at interval +ray (if not at node +ray)--------------
                    else if ((interval.position == 0b10 || interval.position == 0b00) && interval.pos_ray.y < 0)
                    { // neg ray is modified to next row
                        assert(interval.neg_ray.y < 0);

                        V2 vert_coord = cone.root() + interval.pos_ray;
                        Corner *new_crn = crns.try_emplace(grid->coordToKey<false>(vert_coord), vert_coord);
                        float_t new_g = node->g + interval.pos_ray.norm();
                        if (new_crn->min_g > new_g)
                        {
                            new_crn->min_g = new_g;
                            Node *new_node = new_crn->emplaceNode(node, new_g, NodeType::Cone, cone.sgnX());
                            new_node->neg_ray = interval.pos_ray;
                            new_node->pos_ray = V2(cone.sgnX(), 0);

                            if (remove_interval)
                            { // delete this interval bcos cannot access the next row from root
                                assert(interval.neg_ray.x == cone.dxNext());
                                // for this to occur, the -ray of interval has to be adjusted beyond the +ray of interval, to the next x;
                                // the +ray has to point to the next x from the root.
                                new_node->neg_ray = interval.neg_ray - interval.pos_ray;
                            }

                            // queue
                            updateHCost(new_node, p_goal);
                            open_list.queue(new_node);
                            _dbg11("[Cone] <<<<<< [QUEUE] new cone node at pos {" << new_node << "}");
                            has_cone_successors = true;
                        }
                        else
                            _dbg11("[Cone] Cone node cannot be created at (" << vert_coord << ") bcos new G$(" << new_g << ") >= cur G$(" << new_crn->min_g << ")");

                    }

                    // ------- delete interval if the rays do not result in an angular sector
                    if (remove_interval == true)
                        interval_ = intervals.erase(interval_);
                    else
                        ++interval_;
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
                    cone.dx() = cone.dxNext();
                    cone.negRay() = interval.neg_ray;
                    cone.posRay() = interval.pos_ray;
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
                        if (used_current_node == true)
                        {
                            Node *new_node = node->crn->emplaceNode(node->parent, node->g, NodeType::Cone, cone.dxNext());
                            new_node->dx = cone.dx(); // cannot be dxNext bcos it was udpated when seeting used_current_node to true
                            new_node->neg_ray = interval.neg_ray;
                            new_node->pos_ray = interval.pos_ray;
                            updateHCost(new_node, p_goal);
                            open_list.queue(new_node);
                            _dbg11("[Cone] <<<<<< [QUEUE] new node at root {" << new_node << "}");
                        }
                        else
                        {
                            node->dx = cone.dxNext();
                            node->neg_ray = interval.neg_ray;
                            node->pos_ray = interval.pos_ray;
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
            Corner *crn_start = crns.try_emplace(grid->coordToKey<false>(p_start), p_start);
            crn_start->min_g = 0;
            Node *node_start = crn_start->emplaceNode(nullptr, 0, NodeType::Flat, 0);

            // Expand flat nodes and find cone nodes from start
            std::vector<V2> path = {};
            if (expandStart(node_start, -1, p_goal, path) == false)   // -y expansion did not find goal
                if (expandStart(node_start, 1, p_goal, path) == false) // +y expansion did not find goal
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
                            if (expandFlat(node, p_goal, path) == true)
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
