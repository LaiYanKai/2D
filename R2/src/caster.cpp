#include "R2/R2.hpp"

namespace P2D::R2
{
    bool R2::caster(Query *const &query)
    {
        assert(units_enc.empty() == true);
        _dbgtitleheavy("Caster [C]: " << query);
        _dbginc;
        assert(query->prev_open == nullptr && query->next_open == nullptr); // unqueued
        Unit *const old_u_src = query->u_src;
        Unit *const old_u_tgt = query->u_tgt;
        unlinkQuery(query);
        eraseQuery(query);

        bool res = caster(old_u_src, old_u_tgt);
        convTracerEncTrees();

        _dbgdec;
        return res;
    }

    bool R2::caster(Unit *const &old_u_src, Unit *const &old_u_tgt)
    {
        _dbg11("[C] USrc{ " << old_u_src << " }");
        _dbg11("[C] UTgt{ " << old_u_tgt << " }");

        // =========================== Try Cast if VU =====================================
        Ray *ray = cast<false>(old_u_src->crn(), old_u_tgt->crn());
        bool res = false;
        if (ray->vis == Vis::Yes)
        {
            if (old_u_src->isSY() || old_u_src->isEY())
                res = _casterReachedFromSYorEY(ray, old_u_src, old_u_tgt);
            else
            {
                assert(old_u_src->isEU() || old_u_src->isSU());
                _casterReachedFromSUorEU(ray, old_u_src, old_u_tgt);
            }
        }
        else // ray is VN
            _casterCollided(ray, old_u_src, old_u_tgt);

        return res;
    }

    // ========================================================= CASTER Reached From SU =================================================================

    void R2::_casterReachedFromSUorEU(Ray *const &ray, Unit *const &old_u_src, Unit *const &old_u_tgt)
    {
        assert(old_u_src->isSU()); // not possible to cast from Eu
        assert(old_u_tgt->isPS() || old_u_tgt->isT() || old_u_tgt->isTD());

        if (old_u_tgt->isTD())
        {
            _dbgtitle("[C:U-D] Su Reached Td: Tree looped, erase tree.");
            _dbginc;
            old_u_src->unlinkTgt(old_u_tgt);
            eraseUnitTree(TreeDir::Src, old_u_src, false);
            eraseUnitTree(TreeDir::Tgt, old_u_tgt, false);
            _dbgdec;
            return;
        }
        else if (old_u_tgt->isPS() && old_u_tgt->node->units.size() == 1)
        {
            _dbgtitle("[C:U-P] Su Reached Ps (only unit): Continue Trace");
            _dbginc;

            Corner *crn_cur = old_u_tgt->crn();
            const Side side_traced = old_u_tgt->side_tgt;
            std::vector<Unit *> u_ttgts = old_u_tgt->tgts;
            if (old_u_tgt->numSrcs() > 1)
                old_u_tgt->unlinkSrc(old_u_src);
            else
            {
                assert(old_u_tgt->src() == old_u_src);
                old_u_src->removeTgt(old_u_tgt);
                for (Unit *const &old_u_ttgt : old_u_tgt->tgts)
                    old_u_ttgt->removeSrc(old_u_tgt);
                old_u_tgt->erase(EraseState::Del);
            }

            TraceStatus status(side_traced, crn_cur, old_u_tgt->ray_left, old_u_tgt->ray_right);
            status.initSrcWithProg(old_u_src, ray->vec);
            status.initTgtsWithCoord(u_ttgts, status.crn_cur->coord);

            _dbg11("[C:U-P] Begin Trace from Reached Pseudo");
            tracer(status);

            _dbgdec;
        }
        // =============================== 1. SU Reached TVY ===================================
        else if (old_u_tgt->isTY())
        {
            _dbgtitle("[C:U-Y] Su Reached Ty: Cast Src Segment");
            _dbginc; // 1

            // ------------------------ 1a. Unzip / Convert USrc to TVY --------------------------
            _dbgtitle("[C:U-Y] Setup USrc");
            _dbginc; // 2
            const float cost_src_src = old_u_src->cost_src;
            const float cost_tgt_src = old_u_tgt->cost_tgt + ray->length;
            Unit *u_src = toTgt(UnitType::TY, TreeDir::Tgt, old_u_src, old_u_tgt, old_u_src->ray_left, old_u_src->ray_right, cost_tgt_src);
            _dbg11("[C:U-Y] USrc{ " << u_src << " }");
            _dbgdec; // 2

            // ------------------------ 1b. Queue Src Segment --------------------------
            _dbgtitle("[C:U-Y] Queue Src Segment.");
            _dbginc; // 2
            assert(u_src->numSrcs() == 1);
            float_t f = cost_src_src + cost_tgt_src;
            queueQuery(QueryType::Cast, u_src->src(), u_src, f);
            _dbgdec; // 2

            _dbgdec; // 1
        }
        // =============================== 4. SU Reached Tu ===================================
        else if (old_u_tgt->isTU() && old_u_tgt->node->units.size() == 1)
        {
            _dbgtitle("[C:U-U] Su Reached Tu (only unit): Cast Tgt Segment");
            _dbginc; // 1

            assert(old_u_tgt->isTU());

            // ------------------------ 2a. Unzip / Convert UTgt to SU/EX --------------------------
            _dbgtitle("[C:U-U] Setup UTgt");
            _dbginc; // 2
            float_t cost_src_tgt = old_u_src->cost_src + ray->length;
            Unit *u_tgt = toSrc(old_u_src->type, TreeDir::Src, old_u_tgt, old_u_src, old_u_tgt->ray_left, old_u_tgt->ray_right, cost_src_tgt);
            _dbg11("[C:U-U] UTgt{ " << u_tgt << " }");
            _dbgdec; // 2

            // ------------------------ 2b. Queue Tgt Segments --------------------------
            for (Unit *const &u_ttgt : u_tgt->tgts)
            {
                _dbgtitle("[C:U-U] Queue Tgt Segment");
                _dbginc; // 2
                float_t f = cost_src_tgt + u_ttgt->cost_tgt + norm(u_tgt->coord(), u_ttgt->coord());
                QueryType qtype = u_ttgt->isTR() ? QueryType::Trace : QueryType::Cast;
                queueQuery(qtype, u_tgt, u_ttgt, f);
                _dbgdec; // 2
            }

            _dbgdec; // 1
        }
        else
        {
            assert((old_u_tgt->isTU() || old_u_tgt->isPS()) && old_u_tgt->node->units.size() > 1);
            _dbg11("[C:NumUnits>1] Go to earlier due to >1 numUnits");
            units_enc.push_back(old_u_tgt);
            old_u_tgt->addTgt(nullptr);
        }
    }

    // ========================================================= CASTER Reached From SY =================================================================

    bool R2::_casterReachedFromSYorEY(Ray *ray, Unit *old_u_src, Unit *old_u_tgt)
    {
        assert(old_u_src->isSY() || old_u_src->isEY());
        assert(old_u_tgt->isPS() || old_u_tgt->isT() || old_u_tgt->isTD());

        if (old_u_src->isEY())
        {
            if (old_u_tgt->isTY() || old_u_src->side_tgt != old_u_tgt->side_tgt) // note goal is always left
            {
                _dbgtitle("[C:Ey] Ey Reached Ty or RevSide Tgt: Erase Tree");
                _dbginc;
                old_u_src->unlinkTgt(old_u_tgt);
                eraseUnitTree(TreeDir::Src, old_u_src, false);
                eraseUnitTree(TreeDir::Tgt, old_u_tgt, false);
                _dbgdec;
                return false;
            }
        }

        // // ============== Trace immediately if the src->en enters en edge =======================
        // if (old_u_tgt->isPS()) // TODO merge with PS
        // {
        //     const Side &side_traced = old_u_tgt->side_tgt;

        //     assert(old_u_tgt->crn()->is_convex);

        //     if (dirToDirIdx(ray->vec) == old_u_tgt->crn()->di_occ) // enter edge, like proceeding with pseudo
        //     {
        //         _dbg11("[C:Y] Ray is pointing into corner of EN. convert to Ps"); // tgt is not valid turn pt
        //         old_u_tgt->type = UnitType::PS;
        //     }
        // }

        bool was_ps = false;
        // ============== Reached TD=======================
        if (old_u_tgt->isTD())
        {
            _dbgtitle("[C:Y-D] Sy/Ey Reached Td: Tree looped, erase tree.");
            _dbginc;
            old_u_src->unlinkTgt(old_u_tgt);
            eraseUnitTree(TreeDir::Src, old_u_src, false);
            eraseUnitTree(TreeDir::Tgt, old_u_tgt, false);
            _dbgdec;
            return false;
        }
        // ============== Reached PS=======================
        else if (old_u_tgt->isPS()) //
        {
            _dbgtitle("[C:Y-P] Sy/Ey Reached Ps");
            _dbginc;

            // ============== Decide if there is ang change ===================
            // const V2 v_cur = old_u_tgt->edgeVec(TreeDir::Tgt);
            // const V2 v_prev = old_u_tgt->edgeVec(TreeDir::Src);
            if (old_u_tgt->crn()->is_convex == false || dirToDirIdx(ray->vec) == old_u_tgt->crn()->di_occ)
            // if (angDirChanged(old_u_tgt->side_tgt, ray->vec,  v_cur, v_prev ) == false)
            { // trace immediately,
                _dbg11("[C:Y-P] Ps Point cannot be converted to Sy/Ey. Trace immediately");
                Ray *ray_left = old_u_tgt->ray_left;
                Ray *ray_right = old_u_tgt->ray_right;
                mergeRay(!old_u_tgt->side_tgt, ray, ray_left, ray_right);
                TraceStatus status(old_u_tgt->side_tgt, old_u_tgt->crn(), ray_left, ray_right);
                std::vector<Unit *> u_ttgts = old_u_tgt->tgts;
                if (old_u_tgt->numSrcs() > 1)
                    old_u_tgt->unlinkSrc(old_u_src);
                else
                {
                    assert(old_u_tgt->src() == old_u_src);
                    old_u_src->removeTgt(old_u_tgt);
                    for (Unit *const &old_u_ttgt : u_ttgts)
                        old_u_ttgt->removeSrc(old_u_tgt);
                    old_u_tgt->erase(EraseState::Del);
                }

                // make sure u_ttgt is not pointing to any src
                for (Unit *&u_ttgt : u_ttgts)
                    u_ttgt = toTgt(u_ttgt->type, TreeDir::Src, u_ttgt, nullptr, nullptr, nullptr, u_ttgt->cost_tgt);

                status.initSrcWithProg(old_u_src, ray->vec);
                status.initTgtsWithCoord(u_ttgts, status.crn_cur->coord);
                _dbg11("[C:Y-P] Begin Trace");
                tracer(status);

                _dbgdec;
                return false;
            }
            _dbg11("[C:Y-P] Convert Ps to Sy/Ey.");
            was_ps = true;
            _dbgdec;
        }
        // ============== Reached TY =======================
        else if (old_u_tgt->isTY())
        { // path found
            _casterGetPath(old_u_src, old_u_tgt);
            return true;
        }
        // ============== Reached Tu =======================
        else
        {
            _dbg11("[C:Y-U] Sy reached Tu");
            assert(old_u_tgt->isTU());
        }

        assert(old_u_tgt->isPS() || old_u_tgt->isTU());
        Node *const &node_tgt = old_u_tgt->node;
        assert(node_tgt != nullptr);
        float_t cost_src_tgt = old_u_src->cost_src + ray->length;

        // ============== Prune Parallel src and ssrc =======================
        _casterPruneSrc(old_u_src, old_u_tgt, ray);

        // =================== 2a. Is Cheapest so far to reach NodeTgt, continue =======================
        UnitType utype_tgt = old_u_src->type;
        if (approxGt(node_tgt->cost_src_min, cost_src_tgt))
        {
            _dbgtitle("[C:lt] CHEAPEST: <$g to reach NodeTgt(" << node_tgt << ") from uSrc{ " << old_u_src << " }");
            _dbginc; // 2
            _dbg11("[C:lt] $g  from USrc = " << P2D::to_string(cost_src_tgt, 18, 13) << ")");
            _dbg11("[C:lt] $g at NodeTgt = " << P2D::to_string(node_tgt->cost_src_min, 18, 13) << ")");
            _dbg11("[C:lt] NodeTgt(" << node_tgt << ")");

            // update cost
            node_tgt->cost_src_min = cost_src_tgt;

            // convert and queue all other more expensive trees
            _casterConvTrees(old_u_tgt);
            _dbgdec; // 2
        }
        // =================== 2b. More expensive to reach NodeTgt than previous. Return if cannot proj =======================
        else if (approxGt(cost_src_tgt, node_tgt->cost_src_min))
        {
            _dbgtitle("[C:gt] COSTLIER: >=$g to reach NodeTgt(" << node_tgt << ") from uSrc{ " << old_u_src << " }");
            _dbginc; // 2
            _dbg11("[C:gt] $g  from USrc = " << P2D::to_string(cost_src_tgt, 18, 13) << ")");
            _dbg11("[C:gt] $g at NodeTgt = " << P2D::to_string(node_tgt->cost_src_min, 18, 13) << ")");

            utype_tgt = UnitType::EY;
            _dbgdec; // 2
        }
        // =================== 2c. Equal cost, continue =======================
        else
        {
            _dbgtitle("[C:eq] EQUAL: =$g to reach NodeTgt(" << node_tgt << ") from uSrc{ " << old_u_src << " }");
            _dbginc; // 2
            _dbg11("[C:eq] $g  from USrc = " << P2D::to_string(cost_src_tgt, 18, 13) << ")");
            _dbg11("[C:eq] $g at NodeTgt = " << P2D::to_string(node_tgt->cost_src_min, 18, 13) << ")");
            _dbgdec; // 2
        }

        _dbgtitle("[C:Y-U/P] Sy/Ey Reached Tu/Ps. Setup Units");
        _dbginc; // 1
        _dbg11("[C:Y-U/P] USrc{ " << old_u_src << " }");

        // ==================== 3. Create New Tgt (with only one src, old_u_src) and rays ======================
        // Unit *u_tgt = toSrc(utype_tgt, TreeDir::Src, old_u_tgt, old_u_src, old_u_tgt->ray_left, old_u_tgt->ray_right, cost_src_tgt); // create a new u_tgt
        Unit *u_tgt = old_u_tgt;
        assert(old_u_tgt->hasSrc(old_u_src));
        if (old_u_tgt->numSrcs() > 1)
        { // create new Tgt pointing only to old_u_src;
            u_tgt = old_u_tgt->unzipToSrc(utype_tgt, {}, {}, old_u_tgt->ray_left, old_u_tgt->ray_right, cost_src_tgt);
            old_u_src->replaceTgtDeep(old_u_tgt, u_tgt);

            // create new ttgts for u_tgt
            for (Unit *const &old_u_ttgt : old_u_tgt->tgts)
            {
                old_u_ttgt->unzipToTgt(old_u_ttgt->type, {u_tgt}, old_u_ttgt->tgts, old_u_ttgt->ray_left, old_u_ttgt->ray_right, old_u_ttgt->cost_tgt);
                _dbg11("[C:Y-U/P] Unzipped UTTgt to {" << u_tgt->tgts.back() << "}");
                assert(u_tgt->tgts.back()->src() == u_tgt); // only one src
            }
            _dbg11("[C:Y-U/P] Unzipped UTgt to {" << u_tgt << "}");
        }
        else
        {
            u_tgt->convToSrcType(utype_tgt, cost_src_tgt, u_tgt->ray_left, u_tgt->ray_right);
            _dbg11("[C:Y-U/P] Converted UTgt to {" << u_tgt << "}");
            for (Unit *&u_ttgt : u_tgt->tgts)
            {
                if (u_ttgt->numSrcs() > 1)
                {
                    Unit *old_u_ttgt = u_ttgt;
                    u_ttgt = u_ttgt->unzipToTgt(old_u_ttgt->type, {}, old_u_ttgt->tgts, old_u_ttgt->ray_left, old_u_ttgt->ray_right, old_u_ttgt->cost_tgt);
                    u_ttgt->addSrc(u_tgt);
                    old_u_ttgt->removeSrc(u_tgt);
                    assert(u_tgt->hasTgt(old_u_ttgt) == false);
                    _dbg11("[C:Y-U/P] Unzipped UTTgt to { " << u_ttgt << " }");
                }
                assert(u_tgt->hasTgt(u_ttgt));
                assert(u_ttgt->src() == u_tgt);
            }
        }

        // --------- Adjust Rays ---------------------
        for (Unit *u_ttgt : u_tgt->tgts)
        {
            mergeRay(u_tgt->side_tgt, u_ttgt, ray);
            _dbg11("[C:Y-U/P] Adjust Rays of UTTgt to { " << u_ttgt << " }");
        }
        mergeRay(!u_tgt->side_tgt, u_tgt, ray);
        _dbg11("[C:Y-U/P] Adjust Rays of UTgt to { " << u_tgt << " }");

        assert(u_tgt->src() == old_u_src); // there is only one src.

        // ==================== 5. Merge Tgt if any ================================
        std::vector<Unit *> u_ttgts = u_tgt->tgts;
        for (Unit *const &u_merge : old_u_src->tgts)
        {
            if (u_merge->type != utype_tgt || u_merge == u_tgt || u_merge->crn() != u_tgt->crn())
                continue;

            assert(u_merge->side_tgt == u_tgt->side_tgt);
            assert(u_merge->ray(!u_tgt->side_tgt) == ray);
            if (u_merge->ray(u_tgt->side_tgt) == u_tgt->ray(u_tgt->side_tgt)) // mergeable only if the rays are identical
            {
                _dbg11("[C:Y:Merge] Mergable to another Tgt{" << u_merge << "}");
                assert(u_tgt->src() == old_u_src); // there is only one src after conversion
                // erase and add ttgts to u_merge
                for (Unit *const &u_ttgt : u_ttgts)
                {
                    u_ttgt->replaceSrc(u_tgt, u_merge);
                    u_merge->addTgt(u_ttgt);
                }
                Unit *u_ = u_merge;          // copy over so removeTgt bcos u_ cannot be invalidated with removeTgt
                old_u_src->removeTgt(u_tgt); // can cause u_merge to be invalidated since u_merge is *const &
                u_tgt->erase(EraseState::Del);
                u_tgt = u_;
                break;
            }
        }

        // ======================= 6. Isolate Traceable TTgts =========================
        // note old_u_tgt is conv to Ey/Sy from Ps if was_ps == true
        std::vector<Unit *> u_traces;
        const V2 v_cur = u_tgt->edgeVec(TreeDir::Tgt);

        if (was_ps == true)
        {
            for (auto it_ttgt = u_ttgts.begin(); it_ttgt != u_ttgts.end();)
            {
                V2 v_tgtFromTTgt = u_tgt->coord() - (*it_ttgt)->coord();
                if (isLeftOrRight<false>(u_tgt->side_tgt, v_cur, v_tgtFromTTgt))
                { // castable
                    // assert(isCastable(u_tgt->side_tgt, ray->vec, v_tgtFromTTgt, v_cur) == true); // bcos of opt tautness
                    _dbg11("[C:Y-P] TTgt on correct side of Tgt's tgt edge. Possibly Castable to UTTgt{ " << (*it_ttgt) << " }");
                    ++it_ttgt;
                }
                else
                { // not castable
                    // assert(isCastable(u_tgt->side_tgt, ray->vec, v_tgtFromTTgt, v_cur) == false);
                    _dbg11("[C:Y-P] TTgt not on correct side of Tgt's Tgt edge. NOT Castable to UTTgt{ " << (*it_ttgt) << " }");
                    Unit *u_ttgt = *it_ttgt;
                    // remove all srcs
                    u_tgt->unlinkTgt(*it_ttgt);
                    u_ttgt = toTgt(u_ttgt->type, TreeDir::Src, u_ttgt, nullptr, u_ttgt->ray_left, u_ttgt->ray_right, u_ttgt->cost_tgt); // rays are correctly set above
                    u_traces.push_back(u_ttgt);
                    it_ttgt = u_ttgts.erase(it_ttgt);
                }
            }
        }

        // =========================== 7. Queue Castable Tgts ==========================
        for (Unit *u_ttgt : u_ttgts)
        {
            _dbgtitle("[C:Y-U/P] Queue Castable Tgt-->TTgt");
            _dbginc; // 2
            if (isLeftOrRight<true>(u_tgt->side_tgt, ray->vec, v_cur) == true)
            {
                float_t f = cost_src_tgt + u_ttgt->cost_tgt + norm(u_tgt->coord(), u_ttgt->coord());
                if (u_ttgt->isTR() == true && u_tgt->isEY() == true)
                    u_ttgt->type = UnitType::PS; // cannot perform mnr trace, and always cast aggressively

                QueryType qtype = u_ttgt->isTR() == true ? QueryType::Trace : QueryType::Cast;
                queueQuery(qtype, u_tgt, u_ttgt, f);
            }
            else
            {
                u_tgt->unlinkTgt(u_ttgt);
                _dbg11("[C:Y-U/P] TTgt not on correct side of Tgt's tgt edge. Discard");
                _dbg11("[C:Y-U/P] UTTgt{ " << u_ttgt << " }");
                eraseUnitTree(TreeDir::Tgt, u_ttgt, false);
                eraseUnitTree(TreeDir::Src, u_tgt, false); // if not more tgts
            }
            _dbgdec; // 2
        }

        // ============================ 8. Spawn Traces =============================
        if (u_traces.empty() == false)
        {
            _dbgtitle("[C:Y-P] Spawn Trace from UTgt due to NonCastable Tgts");
            _dbginc; // 2
            assert(was_ps == true);

#if P2D_DEBUG // verify that all rays for ttgts are the same
            Ray *rl = u_traces.front()->ray_left;
            Ray *rr = u_traces.front()->ray_right;
            for (auto &u_ttgt_nc : u_traces)
            {
                assert(u_ttgt_nc->ray_left == rl);
                assert(u_ttgt_nc->ray_right == rr);
            }
#endif

            TraceStatus status(u_tgt->side_tgt, u_tgt->crn(), u_traces.front()->ray_left, u_traces.front()->ray_right); // TODO: if rays are not the same, this has to be changed
            _tracerTrace(status);
            status.initSrcWithProg(u_tgt, v_cur); // does not work if src is pointing into u_tgt's occ sec
            status.initTgtsWithCoord(u_traces, status.crn_cur->coord);

            _dbg11("[C:Y-P] Start Trace from UTgt:");
            tracer(status);

            _dbgdec; // 2
        }

        _dbgdec; // 1
        return false;
    }

    // ========================================================= CASTER Collided =================================================================

    void R2::_casterCollided(Ray *const &ray, Unit *const &old_u_src, Unit *const &old_u_tgt)
    {
        assert(old_u_src->isS() || old_u_src->isEU() || old_u_src->isEY());
        assert(old_u_tgt->isPS() || old_u_tgt->isT() || old_u_tgt->isTD());
        assert(ray->vis == Vis::No);
        assert(ray->collision != nullptr);

        const bool has_edge_trace = isStart(old_u_src) == false && isGoal(old_u_tgt) == true;
        const Side side_mjr = old_u_src->side_tgt;

        // ============================= 1. Init Units and Unlink ==========================
        old_u_src->unlinkTgt(old_u_tgt);

        if (old_u_src->isS())
        {
            _dbgtitle("[C:Col] Nrm Caster Collided: LCrn(" << ray->collision->crn(Side::L) << "), RCrn(" << ray->collision->crn(Side::R) << ")");
            _dbginc; // 1
            // ---------------- Mnr Trace ---------------------
            _dbgtitle("[C:Col] Setup Units for Mnr (" << !side_mjr << ") Trace");
            _dbginc; // 2
            std::array<Ray *, 2> rays_left = {old_u_tgt->ray_left, ray};
            std::array<Ray *, 2> rays_right = {ray, old_u_tgt->ray_right};

            Side side_mnr = !side_mjr;
            Unit *u_tgt_mnr = old_u_tgt->unzipToTgt(old_u_tgt->type, {}, old_u_tgt->tgts, nullptr, nullptr, old_u_tgt->cost_tgt);
            // add nullptr to book for mjr
            old_u_src->addTgt(nullptr);
            // add nullptr to use for edge
            if (has_edge_trace == true)
                old_u_src->addTgt(nullptr);

            Corner *crn_cur = ray->collision->crn(side_mnr);
            TraceStatus status_mnr(side_mnr, crn_cur, rays_left[side_mnr], rays_right[side_mnr]);
            status_mnr.initSrcWithProg(old_u_src, ray->vec);
            status_mnr.initTgtWithProg(u_tgt_mnr, -ray->vec);

            _dbg11("[C:Col] Mnr USrc{ " << old_u_src << " }");
            _dbg11("[C:Col] Mnr UTgt{ " << u_tgt_mnr << " }");
            _dbgdec; // 2

            // ---------------- Mjr Trace ---------------------
            _dbgtitle("[C:Col] Setup Units for Mjr (" << side_mjr << ") Trace");
            _dbginc; // 2
            Unit *u_tgt_mjr = toTgt(old_u_tgt->type, TreeDir::Src, old_u_tgt, nullptr, nullptr, nullptr, old_u_tgt->cost_tgt);

            crn_cur = ray->collision->crn(side_mjr);
            TraceStatus status_mjr(side_mjr, crn_cur, rays_left[side_mjr], rays_right[side_mjr]);
            status_mjr.initSrcWithProg(old_u_src, ray->vec);
            status_mjr.initTgtWithProg(u_tgt_mjr, -ray->vec);

            _dbg11("[C:Col] Mjr USrc{ " << old_u_src << " }");
            _dbg11("[C:Col] Mjr UTgt{ " << u_tgt_mjr << " }");
            _dbgdec; // 2

            // ============================= 2. Start Mnr Trace ==========================
            _dbg11("[C:Col] Begin Mnr (" << side_mnr << ") Trace");
            bool refound_mnr_src = tracer(status_mnr);

            // ============================= 4. Start Edge Trace if possible ==========================
            if (has_edge_trace)
            {
                old_u_src->removeTgt(nullptr); // remove nullptr for minor side src
                if (refound_mnr_src == true)
                {
                    _dbg11("[C:Col] Skip Edge Trace Bcos Mnr Src Refound");
                    // delete u_srcs
                    eraseUnitTree(TreeDir::Src, old_u_src, false);
                }
                else
                {
                    _dbgtitle("[C:Col] Setup Units for Edge (" << side_mjr << ") Trace");
                    _dbginc; // 2
                    // unzip old_u_tgt to make sure mjr does not use it.
                    Unit *u_tgt_edge = old_u_tgt->unzipToTgt(old_u_tgt->type, {}, old_u_tgt->tgts, nullptr, nullptr, old_u_tgt->cost_tgt);
                    // create a td type at src to ensure admissibility.
                    u_tgt_edge = old_u_src->unzipToTgt(UnitType::TD, {}, {u_tgt_edge}, nullptr, nullptr, old_u_tgt->cost_tgt + ray->vec.norm());
                    _dbg11("[C:Col]       Edge USrc{ " << old_u_src << " }");
                    _dbg11("[C:Col] Spawned TD UTgt{ " << u_tgt_edge << " }");
                    _dbgdec; // 2

                    _dbg11("[C:Col] Begin Edge (" << side_mjr << ") Trace");
                    TraceStatus status_edge(side_mjr, old_u_src->crn(), rays_left[side_mnr], rays_right[side_mnr]);
                    status_edge.initSrcWithProg(old_u_src, status_edge.v_cur);
                    status_edge.initTgtWithProg(u_tgt_edge, -status_edge.v_cur);
                    _tracerTrace(status_edge);

                    tracer(status_edge);
                }
            }

            // ============================= 5. Start Mjr Trace (placed after edge bcos edge needs to make a copy of the old src and tgts) ==========================
            _dbg11("[C:Col] Begin Mjr (" << side_mjr << ") Trace");
            old_u_src->removeTgt(nullptr);
            tracer(status_mjr);
            _dbgdec; // 1
        }
        else
        {
            assert(old_u_src->isE());
            // Spawn only Major Trace
            _dbgtitle("[C:Col] Eu/Ey Caster Collided: LCrn(" << ray->collision->crn(Side::L) << "), RCrn(" << ray->collision->crn(Side::R) << ")");
            _dbginc; // 1

            // ---------------- Mjr Trace ---------------------
            _dbgtitle("[C:Col] Eu/Ey Setup Units for Mjr (" << side_mjr << ") Trace");
            _dbginc; // 2
            Ray *ray_left = side_mjr == Side::L ? old_u_tgt->ray_left : ray;
            Ray *ray_right = side_mjr == Side::L ? ray : old_u_tgt->ray_right;
            Corner *crn_cur = ray->collision->crn(side_mjr);

            Unit *u_tgt = toTgt(old_u_tgt->type, TreeDir::Src, old_u_tgt, nullptr, nullptr, nullptr, old_u_tgt->cost_tgt); // rays removed in initfor ti tgts

            _dbg11("[C:Col] Eu/Ey Mjr USrc{ " << old_u_src << " }");
            _dbg11("[C:Col] Eu/Ey Mjr UTgt{ " << u_tgt << " }");
            _dbgdec; // 2

            _dbg11("[C:Col] Eu/Ey Begin Mjr (" << side_mjr << ") Trace");
            TraceStatus status_mjr(side_mjr, crn_cur, ray_left, ray_right);
            status_mjr.initSrcWithProg(old_u_src, ray->vec);
            status_mjr.initTgtWithProg(u_tgt, -ray->vec);
            tracer(status_mjr);

            _dbgdec; // 1
        }
    }

    void R2::_casterGetPath(Unit *const &u_src, Unit *const &u_tgt)
    {
        _dbg11("[C] Path found: $ " << P2D::to_string(u_src->cost_src + u_tgt->cost_tgt + norm(u_src->coord(), u_tgt->coord())));

        Unit *u = u_tgt;
        Path path_rev = {u->coord()};
        while (isGoal(u) == false)
        {
            u = u->tgt();
            path_rev.push_back(u->coord());
            assert(u->isTY());
            u->isValid();
        }

        // copy reverse to path
        assert(path.empty());
        _dbg10("\t" << path_rev.back());
        path.push_back(path_rev.back());
        for (auto it_coord = std::next(path_rev.rbegin()); it_coord != path_rev.rend(); ++it_coord)
        {
            path.push_back(*it_coord);
            _dbg00(";" << path.back());
        }

        // copy to src
        u = u_src;
        path.push_back(u->coord());
        while (isStart(u) == false)
        {
            u = u->src();
            path.push_back(u->coord());
            _dbg00(";" << path.back());
            assert(u->isSY());
            u->isValid();
        }
        _dbg01("");
    }

    bool R2::_casterConvTree(Unit *const u_from)
    {
        _dbginc;                // 1
        assert(u_from->isSY()); // convert u_from to EY
        for (size_t i_tgt = 0; i_tgt < u_from->tgts.size();)
        {
            Unit *const u_tgt = u_from->tgts[i_tgt];
            assert(u_tgt->isEU() == false); // Eu can never follow SY

            if (u_tgt->isSrcY())
            { // reached tgts
                if (u_tgt->side_tgt != u_from->side_tgt)
                {
                    _dbg11("[CastCTT] Erase Tree for USyTgt/UEyTgt with Opp SideTgt{ " << u_tgt << " }");
                    u_tgt->removeSrc(u_from);
                    eraseUnitTree(TreeDir::Tgt, u_tgt, true);
                    u_from->tgts.erase(u_from->tgts.begin() + i_tgt);
                    continue; // no change to i_tgt
                }

                assert(u_tgt->side_tgt == u_from->side_tgt);
                if (u_tgt->isSY() && _casterConvTree(u_tgt))
                { // is deleted
                    u_tgt->erase(EraseState::Del);
                    u_from->tgts.erase(u_from->tgts.begin() + i_tgt);
                    continue; // no change to i_tgt
                }

                assert(u_tgt->isEY()); // converted to EY in casterConvTree
            }
            else if (u_tgt->isSU())
            {
                _convToTgtTree(u_tgt); // convert u_tgt onwards to Tu, until non-Su / non-Eu
                assert(u_tgt->isTU());

                // queue
                float_t f = u_tgt->cost_tgt + u_from->cost_src + norm(u_tgt->coord(), u_from->coord());
                assert(f < INF);
                _dbg11("[CastCTT] Queue UTgt{" << u_tgt->repr(0) << "}. UFrom{" << u_from->repr(0) << "} conv to Eu later.");
                queueQuery(QueryType::Cast, u_from, u_tgt, f);

                // u_from is converted to ex later.
                assert(u_from->isSY());
            }
            else
            {
                // do nothing // there should be queued
                assert(u_tgt->isT() || u_tgt->isPS() || u_tgt->isTR());
                if (u_tgt->isTR())
                {
                    u_tgt->type = UnitType::PS; // prevent eu/ey reverse traces from occuring, and to aggressively cast for more ex
                    _dbg11("[CastCTT] Converted TgtTr to Ps{" << u_tgt << "}");
                    for (Query *const &q : u_from->queries)
                    {
                        if (q->u_tgt == u_tgt)
                        {
                            q->type = QueryType::Cast;
                            _dbg11("[CastCTT] Convert Query to Cast {" << q << "}");
                            break;
                        }
                    }
                }
#if P2D_DEBUG
                bool found = false;
                for (Query *const &q : u_from->queries)
                {
                    assert(q->u_src == u_from);
                    if (q->u_tgt == u_tgt)
                    {
                        found = true;
                        break;
                    }
                }
                assert(found == true);
#endif
                // u_from is converted to ex later.
                assert(u_from->isSY());
            }

            ++i_tgt;
        }

        bool no_tgts = u_from->tgts.empty();
        if (no_tgts == true)
            _dbg11("[CastCTT] Erase (0Tgt) USy{ " << u_from << " }");
        else
        {
            u_from->convToSrcType(UnitType::EY, u_from->cost_src, u_from->ray_left, u_from->ray_right);
            _dbg11("[CastCTT] Conv To UEy{ " << u_from << " }");
        }

        _dbgdec; // 1
        return no_tgts;
    }

    void R2::_casterConvTrees(Unit *u_tgt_cheapest)
    {
        Node *const node_tgt = u_tgt_cheapest->node;

        // ------------------ Setup Units -----------------------
        for (Unit *old_u_tgt = node_tgt->units.front(); old_u_tgt != nullptr;)
        {
            Unit *u_next = old_u_tgt->next;
            if (old_u_tgt == u_tgt_cheapest || old_u_tgt->isSY() == false)
            { // ignore non-SVY units in the current tgt
                old_u_tgt = u_next;
                continue;
            }

            assert(old_u_tgt->isSY());
            assert(old_u_tgt->cost_src < u_tgt_cheapest->cost_src);
            if (_casterConvTree(old_u_tgt))
            { // deleted
                assert(old_u_tgt->hasTgt() == false);
                _dbg11("[CastCTT] Erase >$g (0Tgts) USy{" << old_u_tgt << "}");
                eraseUnitTree(TreeDir::Src, old_u_tgt, false);
            }
            old_u_tgt = u_next;
        }
    }

    bool R2::_casterPruneSrc(Unit *&old_u_src, Unit *const &old_u_tgt, Ray *&ray)
    {
        assert(ray->vis == Vis::Yes);
        assert(old_u_src->isSrcY());

        if (isStart(old_u_src) == false)
        {
            Unit *const old_u_ssrc = old_u_src->src();
            V2 v_srcFromSSrc = old_u_src->coord() - old_u_ssrc->coord();
            assert(ray->vec != 0 && v_srcFromSSrc != 0);
            if (det(ray->vec, v_srcFromSSrc) == 0 && dot(ray->vec, v_srcFromSSrc) > 0) // parallel and in same direction
            {
                _dbgtitle("[C:PruneSrc] SSrc-Src-Tgt is parallel and same direction. Prune Src and replace Ray");
                _dbginc;
                _dbg11("[C:PruneSrc] Replaced OldUSrc{ " << old_u_src << " }");

                old_u_tgt->ray_left = old_u_src->ray_left;
                old_u_tgt->ray_right = old_u_src->ray_right;

                old_u_tgt->replaceSrcDeep(old_u_src, old_u_ssrc);
                assert(old_u_ssrc->hasTgt(old_u_tgt) == true);
                assert(old_u_tgt->hasSrc(old_u_ssrc));
                if (old_u_src->hasTgt() == false)
                {
                    old_u_ssrc->removeTgt(old_u_src);
                    old_u_src->erase(EraseState::Del);
                }
                old_u_src = old_u_ssrc;
                _dbg11("[C:PruneSrc]    with OldUSSrc{ " << old_u_ssrc << " }");

                ray = tryEmplaceRay(TreeDir::Src, old_u_ssrc->crn(), old_u_tgt->crn()).first;
                ray->vis = Vis::Yes;
                _dbg11("[C:PruneSrc]          New Ray{ " << ray << " }");

                _dbgdec;
                return true;
            }
        }
        return false;
    }

    bool R2::casterStart()
    {
        assert(units_enc.empty() == true);

        _dbgtitleheavy("[CS] Caster Start");
        _dbginc; // 1

        // =========================== 1. Try Casting =========================
        Ray *ray = cast<true>(crn_start, crn_goal);
        if (ray == nullptr)
        {
            _dbgdec;     // 1
            return true; // found a direct path
        }

        // =========================== 2. Project to get opposite ray (maybe unnecessary with optimisations) =========================
        Ray *ray_rev = tryEmplaceRay(TreeDir::Src, crn_goal, crn_start).first;

        // =========================== 3. Create Units and Trace =========================
        crn_reflect = new Corner(false, crn_start->mkey, 0, crn_start->coord); // make sure at check ray only the corner pointer is checked instead of the coord // quick hack
        ray_reflect = new Ray(crn_goal, crn_reflect);
        Unit *u_start_l = node_start->units.emplaceSrcType(Side::L, UnitType::SY, {}, {}, ray, ray_reflect, 0);
        node_start->units.emplaceSrcType(Side::R, UnitType::SY, {u_start_l}, {u_start_l}, nullptr, nullptr, 0);
        Unit *u_start_r = node_start->units.emplaceSrcType(Side::R, UnitType::SY, {}, {}, ray_reflect, ray, 0);
        node_start->units.emplaceSrcType(Side::L, UnitType::SY, {u_start_r}, {u_start_r}, nullptr, nullptr, 0);

        Unit *u_goal_l = node_goal->units.emplaceTgtType(Side::L, UnitType::TY, {}, {}, nullptr, nullptr, 0); // side isn't impt
        Unit *u_goal_r = node_goal->units.emplaceTgtType(Side::R, UnitType::TY, {}, {}, nullptr, nullptr, 0); // side isn't impt

        _dbg11("[CS] Begin (L) Start Collision Trace :");
        TraceStatus status_l(Side::L, ray->collision->crn(Side::L), ray_rev, ray);
        status_l.initSrcWithProg(u_start_l, ray->vec);
        status_l.initTgtWithProg(u_goal_l, ray_rev->vec);
        tracer(status_l);

        _dbg11("[CS] Begin (R) Start Collision Trace :");
        TraceStatus status_r(Side::R, ray->collision->crn(Side::R), ray, ray_rev);
        status_r.initSrcWithProg(u_start_r, ray->vec);
        status_r.initTgtWithProg(u_goal_r, ray_rev->vec);
        tracer(status_r);

        // =========================== 4. Convert to Tgt Trees if any =========================
        convTracerEncTrees();

        _dbgdec; // 1
        return false;
    }

    void R2::findCollisionCrns(Ray *const &ray, const LosResult &res)
    {
        assert(ray->collision == nullptr); // must not have a collision before
        assert(res.collided == true);

        Corner *crn_left, *crn_right;
        Corner c(false, res.key_left, 0, res.coord_left);
        if (trace(c, Side::L, res.di_left) == false)
            crn_left = &null_crn;
        else
            crn_left = tryEmplaceCrn(c).first;
        c = Corner (false, res.key_right, 0, res.coord_right);
        if (trace(c, Side::R, res.di_right) == false)
            crn_right = &null_crn;
        else
            crn_right = tryEmplaceCrn(c).first;

        ray->setCollision(crn_left, crn_right);
    }

    Ray *R2::project(Corner *const &crn_src, Corner *const &crn_tgt)
    { // a cast must have already occured between crn_src and crn_tgt
        Ray *ray = getRay(TreeDir::Src, crn_src, crn_tgt);
        project(ray);
        return ray;
    }
    void R2::project(Ray *const &ray)
    {
        assert(ray != nullptr);
        if (ray->collision == nullptr)
        { // project
            LosResult res = los.project<false, true, true>(ray->crn_src->coord, ray->crn_src->mkey, ray->crn_tgt->coord, ray->crn_tgt->mkey);
        
            if (res.collided == false)
            {
                _dbg11("[Proj] New Proj from Tgt(" << ray->crn_tgt << ") from Src(" << ray->crn_src << ") OOM");
                ray->setCollision(&null_crn, &null_crn);
            }
            else
            {
                _dbg11("[Proj] New Proj from Tgt(" << ray->crn_tgt << ") from Src(" << ray->crn_src << ") collided at " << res);
                findCollisionCrns(ray, res);
            }
        }
        else
            _dbg11("[Proj] Found Old Proj from Tgt(" << ray->crn_tgt << ") from Src(" << ray->crn_src << ")");

        assert(ray != nullptr);
        assert(ray->collision != nullptr);
        assert(ray->collision->crn_left != nullptr && ray->collision->crn_right != nullptr);

        return;
    }

    template <bool is_start = false>
    Ray *R2::cast(Corner *const &crn_src, Corner *const &crn_tgt)
    {
        Ray *ray = tryEmplaceRay(Src, crn_src, crn_tgt).first;
        if constexpr (is_start == true)
        {
            LosResult res = los.cast<false, true, true>(crn_src->coord, crn_src->mkey, crn_tgt->coord, crn_tgt->mkey);

            if (res.collided == false)
            {
                assert(path.empty());
                assert(crn_src == crn_start && crn_tgt == crn_goal);
                _dbg11("[CastStart] Path Found (a straight line)");

                path = {crn_tgt->coord, crn_src->coord};
                return nullptr;
            }
            else
            {
                _dbg11("[CastStart] VU Collided: {" << crn_src << ";" << crn_tgt << "} at " << res);
                assert(res.state == LosState::collided);
                ray->vis = Vis::No;
                findCollisionCrns(ray, res);
                return ray;
            }
        }
        else
        {
            if (ray->vis == Vis::Unknown)
            {
                LosResult res = los.cast<false, true, true>(crn_src->coord, crn_src->mkey, crn_tgt->coord, crn_tgt->mkey);

                if (res.collided == false)
                {
                    _dbg11("[Cast] VU Reached: {" << crn_src << ";" << crn_tgt << "}");
                    ray->vis = Vis::Yes;
                }
                else
                {
                    _dbg11("[Cast] VU Collided: {" << crn_src << ";" << crn_tgt << "} at " << res);
                    assert(res.state == LosState::collided);
                    ray->vis = Vis::No;
                    findCollisionCrns(ray, res);
                }
            }
            else if (ray->vis == Vis::Yes)
                _dbg11("[Cast] Found VY : {" << crn_src << ";" << crn_tgt << "}");
            else
            {
                assert(ray->vis == Vis::No);
                assert(ray->collision != nullptr);
                _dbg11("[Cast] Found VN : {" << crn_src << ";" << crn_tgt << "} collided at {" << ray->collision->crn_left << ";" << ray->collision->crn_right << "}");
            }
            return ray;
        }
    }
}