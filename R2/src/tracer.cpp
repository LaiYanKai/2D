#include "R2/R2.hpp"

namespace P2D::R2
{
    void R2::tracerPolled(Query *const &query)
    {
        Unit *u_src = query->u_src;
        Unit *const u_cur = query->u_tgt;
        assert(query->type == QueryType::Trace);

        _dbgtitleheavy("[TP] Tracer Polled");
        _dbginc;

        unlinkQuery(query);
        eraseQuery(query);

        _dbg11("[TPS] USrc{ " << u_src << " }");
        _dbg11("[TPS] UCur{ " << u_cur << " }");
        assert(u_src->isS() || u_src->isE());
        assert(u_cur->isTR());
        assert(u_src->hasTgt(u_cur));
        assert(u_cur->hasSrc(u_src));

        std::vector<Unit *> u_tgts = u_cur->tgts;

        TraceStatus status(u_cur->side_tgt, u_cur->crn(), u_cur->ray_left, u_cur->ray_right);

        // Unlink Psd and Src
        u_src->unlinkTgt(u_cur);
        if (u_cur->hasSrc() == false)
        {
            _dbgtitle("[TPS] Delete UTr bcos no more Srcs, and unlink UTgts from UTr");
            _dbginc;
            for (Unit *const &u_tgt : u_tgts)
                u_tgt->removeSrc(u_cur);

            u_cur->erase(EraseState::Del);
            _dbgdec;
        }

        // Unstage
        _dbgtitle("[TPS] Unstage Tgts");
        _dbginc;
        for (Unit *&u_tgt : u_tgts)
            u_tgt = toTgt(u_tgt->type, TreeDir::Src, u_tgt, nullptr, nullptr, nullptr, u_tgt->cost_tgt);
        _dbgdec;

        // ----------- 2. Continue Trace if not OOM ------------------
        status.initSrcWithProg(u_src, status.crn_cur->coord - u_src->coord());
        status.initTgtsWithCoord(u_tgts, status.crn_cur->coord);

        tracer(status);

        convTracerEncTrees();

        _dbgdec;
    }

    bool R2::tracer(TraceStatus &status)
    {
        _dbgtitleheavy(status.repr() << " (" << status.side << ") Tracer from Cur(" << status.crn_cur << ")");
        _dbginc; // 1
#if P2D_DEBUG
        for (TraceInfo *ti_src = status.srcs.front(); ti_src != nullptr; ti_src = ti_src->next)
            _dbg11(status.repr() << " From TiSrc{ " << ti_src << " }");
        for (TraceInfo *ti_tgt = status.tgts.front(); ti_tgt != nullptr; ti_tgt = ti_tgt->next)
            _dbg11(status.repr() << " From TiTgt{ " << ti_tgt << " }");
#endif

        assert(status.num_points == 0);
        assert(status.refound_mnr_src == false);
        while (1)
        {
            _dbgtitle(status.repr() << " " << status);
            _dbginc; // 2

            // ===================  OOM Check ========================
            if (status.crn_cur == &null_crn)
            {
                _dbg11(status.repr() << " OOM! Erase UnitTree");
                _dbgdec; // 2
                break;   // while
            }

            // ===================  Updates Information for Iteration ========================
            _tracerUpdate(status);

            // ===================  Tgt Check (refound src / prune / edge) ========================
            // may spawn recursive src traces from tgts.
            if (_tracerProcessTgt(status) == true)
            {            // no more tgts
                _dbgdec; // 2
                break;
            }

            // ===================  Src Check (refound src / prune / edge) ========================
            // may queue reverse traces from collided projections (requiring tgts are pruned / edge checked)
            if (_tracerProcessSrc(status) == true)
            {            // no more srcs
                _dbgdec; // 2
                break;
            }

            // ===================  Process Abnormal states (edge / reverse / encountered) ========================
            if (_tracerProcessAbnormal(status) == true)
            {            // at this point, if state is enc, then previously when discovered enc point, tgts were not opt taut.
                _dbgdec; // 2
                break;
            }

            assert(status.revs.empty() == true);

            // =================== Create Src Point ========================
            if (status.crn_cur->is_convex == true)
            {
                assert(status.srcs.size() == 1);
                TraceInfo *const &ti_src = status.srcs.front();
                if (_tracerPointSrc(ti_src, status) == true)
                {
                    // =================== Check Encounter  =======================
                    if (_tracerEncounteredTree(status))
                    {            // discovered enc point and all tgts are opt taut.
                        _dbgdec; // 2
                        break;
                    }

                    // ==================== Try Queueing (if normal, and points are opt taut) ========================
                    if (_tracerQueueSrc(ti_src, status))
                    {            // queueable
                        _dbgdec; // 2
                        break;
                    }
                }
            }

            // =================== Create Pseudo Tgt Point ========================
            if (status.crn_cur->is_convex == false)
            {
                assert(status.crn_cur->is_convex == false);
                if (_tracerPointPs(status) == true)
                {
                    if (_tracerQueueTgt(status))
                    {
                        _dbgdec; // 2
                        break;
                    }
                }
            }

            // =================== TraceCache ========================
            _tracerTrace(status);

            _dbgdec; // 2
        }

        // =================== Try Erasing Tree if not used ========================
        _tracerErase(status);

        _dbgdec; // 1
        return status.refound_mnr_src;
    }

    void R2::_tracerErase(TraceInfoContainer &ti_srcs, TraceInfoContainer &ti_tgts)
    {
        _dbgtitle("[T:ErsUTree] Erase UnitTree");
        _dbginc;

        for (TraceInfo *ti_src = ti_srcs.front(); ti_src != nullptr; ti_src = ti_src->next)
            eraseUnitTree(TreeDir::Src, ti_src->u, false);

        for (TraceInfo *ti_tgt = ti_tgts.front(); ti_tgt != nullptr; ti_tgt = ti_tgt->next)
            eraseUnitTree(TreeDir::Tgt, ti_tgt->u, false); // there shouldn't be any more queries

        ti_srcs.clear();
        ti_tgts.clear();
        _dbgdec;
    }

    void R2::_tracerErase(TraceStatus &status)
    {
        _dbgtitle("[T:ErsUTree] Erase UnitTree");
        _dbginc;

        for (TraceInfo *ti_src = status.srcs.front(); ti_src != nullptr; ti_src = ti_src->next)
            eraseUnitTree(TreeDir::Src, ti_src->u, false);

        for (TraceInfo *ti_tgt = status.tgts.front(); ti_tgt != nullptr; ti_tgt = ti_tgt->next)
            eraseUnitTree(TreeDir::Tgt, ti_tgt->u, false); // there shouldn't be any more queries

        status.srcs.clear();
        status.tgts.clear();
        _dbgdec;
    }

    void R2::_tracerErase(const TreeDir &tdir, TraceInfo *&ti_rel, TraceInfoContainer &ti_rels)
    {
        _dbgtitle("[T:Ers] Erase Unit and Ti{ " << ti_rel << " }");
        _dbginc;
        eraseUnitTree(tdir, ti_rel->u, false);
        TraceInfo *ti_next = ti_rel->next;
        ti_rels.erase(ti_rel);
        ti_rel = ti_next;
        _dbgdec;

        return;
    }

    void R2::_tracerUnstageTgts(Unit *const &u_src, std::vector<Unit *> &u_tgts)
    {
        _dbgtitle("[T:UnStgTgts] Unstage UTgts for Tracer");
        _dbginc;

        if (u_src != nullptr)
        {
            assert(&u_tgts != &(u_src->tgts)); // avoid invalidation problems.
            assert(u_src->numSrcs() == 1);
        }
        for (Unit *&u_tgt : u_tgts)
        {
            Unit *const old_u_tgt = u_tgt;
            if (u_src != nullptr)
            {                                                                                                     // unlink u_src if not nullptr
                _dbg11("[T:UnStgTgts] Unlink UTgt(" << u_tgt->repr(0) << ") and USrc(" << u_src->repr(0) << ")"); // unlink only if num srcs is one
                assert(u_tgt->hasSrc(u_src) == true);
                u_tgt->unlinkSrc(u_src);
            }
            u_tgt = toTgt(old_u_tgt->type, TreeDir::Src, old_u_tgt, nullptr, old_u_tgt->ray_left, old_u_tgt->ray_right, old_u_tgt->cost_tgt);

            assert(u_tgt->numSrcs() == 0);
        }

        _dbgdec;
    }

    void R2::_tracerStageUnitTree(Unit *const &u_src, const std::vector<Unit *> &u_tgts)
    {
        assert(u_src != nullptr);
        assert(u_tgts.empty() == false);
        for (Unit *const &u_tgt : u_tgts)
        {
            assert(u_src->hasTgt(u_tgt) == false);
            assert(u_tgt->numSrcs() == 0);
            u_src->linkTgt(u_tgt);
            u_src->isValid();
            u_tgt->isValid();
        }
    }

    bool R2::_tracerNotProgressed(const bool &traceTdir_is_relTdir, TraceInfo *const &ti, TraceStatus &status)
    {
        const Corner *const &crn_cur = status.crn_cur;
        Unit *const &u = ti->u;
        assert(ti->v_curFromRel == crn_cur->coord - ti->u->coord());
        V2 v_test = ti->v_curFromRel;
        Side side = traceTdir_is_relTdir == true ? !status.side : status.side;

        // ===== Filter special cases =====
        if (u->crn()->mkey == crn_cur->mkey)
        {
            assert(ti->v_curFromRel == 0);
            if (addDirIdx<true>(crn_cur->di_occ, 4) == u->crn()->di_occ)
            {
                _dbg11("[T:NotProg] Checkerboard Corner Found");
                v_test = dirIdxToDir(u->crn()->di_occ);
            }
            else
            {
                _dbg11("[T:NotProg] At Start / Rel");
                assert(isStart(u) == true || u->crn() == crn_cur);
                v_test = dirIdxToDir(crn_cur->di_occ);
            }
        }

        bool was_prog = ti->cnt_prog == 0;
        bool is_prog = isLeftOrRight<false>(side, v_test, ti->v_prog);

        if (was_prog == false && is_prog == true)
        {
            // this condition will activate if exactly zero to v_prog or 180.
            //    (180 case if progressed is possible for ray.
            //     180 case if not progressed is not possible for polygonal maps, so no need extra condition handling 180 for prog
            //     also satisfies v_cfs == 0, no need nextra condition handling for v_cfs == 0)
            assert(-crn_cur->edgeVec(!status.side) == status.v_prev);
            auto p = dtrLeftOrRight<false>(side, status.v_prev, v_test);
            // if (p.first == 0)
            // {   // parallel
            //     assert(v_test != 0);
            //     assert(dot(status.v_prev, v_test) > 0);
            //     // not informative.
            //     _dbg11("[T:NotProg:" << traceTdir_is_relTdir << "] Parallel cur and v_prev[" << ti->cnt_prog << "](" << ti->v_prog << "). Ti{ " << ti << " }");
            // }
            // else
            if (p.first == 0)
            {
                Corner *crn_prev = status.crn_cur->trace(!status.side);
                assert(crn_prev != nullptr);
                // assert(crn_prev->mkey == u->crn()->mkey);
                dir_idx_t unit_di = crn_prev->di_occ;                                                             // works for both rel is at corner, or on edge
                if (crn_prev->mkey == u->crn()->mkey && addDirIdx<true>(crn_prev->di_occ, 4) == u->crn()->di_occ) // checkerboard crn
                    unit_di = u->crn()->di_occ;                                                                   // need extra care for checkerboard crn
                V2 v_prev = status.crn_cur->coord - crn_prev->coord - dirIdxToDir(unit_di);
                _dbg11("[T:NotProg:" << traceTdir_is_relTdir << "] Replace Vprev bcos was not prog and is prog, and Vprev//vcfr. NewVprev(" << v_prev << ")");
                p = dtrLeftOrRight<false>(side, v_prev, v_test); // replace v_prev
            }
            if (p.second == false)
            { // is 180 degree. swap the prog
                ++ti->cnt_prog;
                ti->v_prog = -ti->v_prog;
                assert(ti->cnt_prog >= 2);
                _dbg11("[T:NotProg:" << traceTdir_is_relTdir << "] (half circle wind) to prog[" << ti->cnt_prog << "](" << ti->v_prog << "). Ti{ " << ti << " }");
            }
            else
            { // progressed
                --ti->cnt_prog;
                if (ti->cnt_prog > 0)
                    ti->v_prog = -ti->v_prog; // reverse if cnt prog is still larger than cnt_prog
                else
                    ti->v_prog = ti->v_curFromRel; // cannot be v_test to avoid problems in other functions
                assert(ti->cnt_prog >= 0);
            }
        }
        else if (was_prog == true && is_prog == false)
        {
            ++ti->cnt_prog;
            assert(ti->cnt_prog >= 1);
            _dbg11("[T:NotProg:" << traceTdir_is_relTdir << "] Not Progressed past prog[" << ti->cnt_prog << "](" << ti->v_prog << "). Ti{ " << ti << " }");
        }
        else if (was_prog == true && is_prog == true)
        {
            assert(ti->cnt_prog == 0);
            ti->v_prog = ti->v_curFromRel;
        }
        else
        {
            assert(was_prog == false && is_prog == false);
            assert(ti->cnt_prog >= 1);
            _dbg11("[T:NotProg:" << traceTdir_is_relTdir << "] Not Progressed past prog[" << ti->cnt_prog << "](" << ti->v_prog << "). Ti{ " << ti << " }");
        }

        bool has_prog = ti->cnt_prog == 0;
        status.all_progressed &= has_prog;
        return !has_prog;
    }

    bool R2::_tracerRefoundRel(const TreeDir &tdir, const Corner *const &crn_cur, TraceInfo *&ti_rel, TraceInfoContainer &ti_rels)
    {
        if (crn_cur == ti_rel->u->crn())
        {
            _dbg11("[T:Refound:" << tdir << "] URel{ " << ti_rel->u << " }");
            _tracerErase(tdir, ti_rel, ti_rels);
            return true;
        }
        else
            return false;
    }

    R2::BeyondRayType R2::_tracerCheckRay(const Side &side_traced, const Corner *const &crn_cur, Ray *const &ray, const V2 &v_curFromSrc)
    {
        if (ray == nullptr)
            return BeyondRayType::NotBeyondRay;

        assert(ray->vec != 0);
        auto p_dtr = dtrLeftOrRight<true>(side_traced, v_curFromSrc, ray->vec);

        bool beyond_ray;
        if (p_dtr.first == 0) // applies to parallel v_curFromSrc and ray->vec, and v_curFromSrc == 0.
        {
            V2 v_occ = dirIdxToDir(crn_cur->di_occ);
            long_t dtr_ray = det(ray->vec, v_occ);

            // if (crn_cur->mkey == crn_start->mkey)
            // {
            //     assert(v_curFromSrc == 0);
            //     _dbg11("[T:BR] BEYOND RAY:");
            // }

            assert(v_curFromSrc != 0 || (v_curFromSrc == 0 && crn_cur->mkey == crn_start->mkey));
            if (dtr_ray == 0)
            {
                beyond_ray = dot(ray->vec, v_occ) > 0; // can be < 0 for start case; when cast is perfectly 45 deg and collided into corner. rev ray is exactly reverse.
                if (beyond_ray)
                    _dbg11("[T:BR] BEYOND RAY: Ray coincide with diocc of Cur(" << crn_cur << "). Ray{ " << ray << " }");
            }
            else
            {
                V2 v_nTracePrev = crn_cur->edgeVec(!side_traced);
                long_t dtr_nTracePrev = det(v_nTracePrev, v_occ);
                assert(dtr_nTracePrev != 0);
                beyond_ray = sgn(dtr_nTracePrev) == sgn(dtr_ray);
                if (beyond_ray)
                { // ray and nTracePrev on same side from di_occ: beyond ray
                    _dbg11("[T:BR] BEYOND RAY: Ray on Same side of Cur's di_occ as nTracePrev. Cur(" << crn_cur << "). Ray{ " << ray << " }");
                }
            }
        }
        else
        {
            beyond_ray = p_dtr.second;
            if (beyond_ray)
                _dbg11("[T:BR] Cur(" << crn_cur << ") is Beyond (" << side_traced << ") Ray{ " << ray << " }");
        }

        if (ray != nullptr && beyond_ray == true)
        {
            project(ray);
            if (ray->collision->crn(side_traced) == crn_cur)
                return BeyondRayType::BeyondRayRefoundTrace;
            else
                return BeyondRayType::BeyondRayHiddenTrace;
        }

        return BeyondRayType::NotBeyondRay;
    }

    bool R2::_tracerProcessSrc(TraceStatus &status)
    {
        for (TraceInfo *ti_src = status.srcs.front(); ti_src != nullptr;)
        {
            Unit *const &u_src = ti_src->u;
            // ti_src->v_curFromRel = status.crn_cur->coord - u_src->coord();

            // =========================== Check if Refound Tgt ========================
            if (_tracerRefoundRel(TreeDir::Src, status.crn_cur, ti_src, status.srcs))
            {
                status.refound_mnr_src = status.side != u_src->side_tgt;
                for (TraceRev &rev : status.revs)
                    rev.u->removeTgt(nullptr);
                status.revs.clear(); // pointless to continue, since it will be more expensive after rounding a reverse trace
                continue;
            }

            // ============ Skip if not Progressed Tgt =====================
            if (_tracerNotProgressed(false, ti_src, status) == true)
            {
                ti_src = ti_src->next;
                assert(status.all_progressed == false);
                continue; // nothing to check
            }

            // ============ Check Beyond Ray =====================
            if (_tracerRaySrc(ti_src, status) == true)
            { // pruned sy / beyond ray
                continue;
            }

            // ============ Prune or Do Edge Check first depending on side traced =====================
            // ti_src is next if pruned or in edge, otherwise, is unchanged
            if (u_src->side_tgt == status.side)
            { // side tgt same: prune first, edge second
                if (_tracerPruneSrc(ti_src, status) == true)
                {             // su/ex(w/o prune ray) prunable. // if a prune ray exists, all srcs must have prune rays: be ex or sy
                    continue; // skip prune bcos ti_tgt is deleted. pruned tgts at back of ti_tgts
                }
            } // side tgt same
            else
            { // side tgt different: edge first, prune second
                if (_tracerEdgeSrc(ti_src) == true)
                {             // any edge cur will cause ti_srcs to be empty
                    continue; // skip prune. always taut and trace is recursed. ti_src should be nullptr (only one ti_srcs)
                }
            }

            // no edge or prune
            ti_src = ti_src->next;
            assert(ti_src == nullptr); // only one src
            break;
        }

        // ====================== Look for Rev Traces ==================================
        if (status.revs.empty() == false)
        {
            // reserve ti_src to avoid being pruned in rev traces
            if (status.srcs.empty() == false)
            {
                assert(status.srcs.size() == 1);
                status.srcs.front()->u->addTgt(nullptr);
            }

            // ====================== Setup Units for RevTraces ==================================
            // create a copy for the ti_tgts
            std::vector<TraceStatus *> statuses;
            for (const TraceRev &rev : status.revs)
            {
                _dbg11("[T:ProcSrc] Setup Units for Rev Trace from USrc{ " << rev.u << " }");
                TraceStatus *const &new_status = statuses.emplace_back(new TraceStatus(status.side, status.crn_cur, rev.ray_left, rev.ray_right));
                V2 v_curFromSrc = status.crn_cur->coord - rev.u->coord();
                TraceInfo *ti_src = new_status->srcs.emplace(ListDir::Back, rev.u, v_curFromSrc, v_curFromSrc);
                assert(ti_src->cnt_prog == 0);
                ti_src->state = TraceState::Reverse;
                ti_src->u_spec = ti_src->u;

                for (TraceInfo *ti_tgt = status.tgts.front(); ti_tgt != nullptr; ti_tgt = ti_tgt->next)
                {
                    assert(ti_tgt->v_curFromRel == status.crn_cur->coord - ti_tgt->u->coord());
                    Unit *u_tgt = ti_tgt->u;
                    assert(u_tgt->hasSrc() == false);
                    u_tgt = u_tgt->unzipToTgt(u_tgt->type, {}, u_tgt->tgts, u_tgt->ray_left, u_tgt->ray_right, u_tgt->cost_tgt);
                    _dbg11("[T:ProcSrc] Unzipped UTgt for Rev Trace{ " << u_tgt << " }");
                    new_status->tgts.emplace(ListDir::Back, u_tgt, ti_tgt->v_curFromRel, ti_tgt->v_prog); // may not have been progressed
                }
                _dbg11("[T:ProcSrc] ---");
            }

            // ====================== Spawn RevTraces ==================================
            for (TraceStatus *const &new_status : statuses)
            {
                _dbg11("[T:ProcSrc] Spawn Trace to progress Tgts and then Rev Trace");
                Unit *u_src = new_status->srcs.front()->u;
                tracer(*new_status);
                u_src->removeTgt(nullptr);
                eraseUnitTree(TreeDir::Src, u_src, false);
                delete new_status;
            }

            // remove nullptr
            if (status.srcs.empty() == false)
            {
                assert(status.srcs.size() == 1);
                status.srcs.front()->u->removeTgt(nullptr);
            }

            status.revs.clear();
        }

        // ============ Stop Trace if No more Tgts =====================
        if (status.srcs.empty())
        {
            _dbg11("[T:ProcSrc] Erase UnitTree (Src) bcos no more Tgts");
            return true;
        }
        return false;
    }

    bool R2::_tracerRaySrc(TraceInfo *&ti_src, TraceStatus &status)
    {
        if ((ti_src->state == TraceState::Edge || ti_src->state == TraceState::Reverse))
        {
            assert(ti_src->u_spec != nullptr);
            return false; // no rays to check for edge or reverse srcs.
        }

        assert(ti_src->v_curFromRel == status.crn_cur->coord - ti_src->u->coord());
        Unit *const &u_src = ti_src->u;
        Ray *const &ray = status.ray(status.side);
        const BeyondRayType brtype = _tracerCheckRay(status.side, status.crn_cur, ray, ti_src->v_curFromRel);
        bool has_prune = false;

        // =========================== Check Prunability ========================
        if (brtype == BeyondRayType::BeyondRayHiddenTrace)
        {
            if (u_src->isE() == true)
                assert(status.side == u_src->side_tgt); // cur trace cannot be reversed
            else
            {
                _dbg11("[T:RaySrc] BR,HT: Spawn RevTrace. USrc{ " << u_src << " }"); // to spawn a reverse side_traced, u_src cannot be EY or EU
                status.revs.emplace_back(u_src, status.ray_left, status.ray_right);
                u_src->addTgt(nullptr); // prevent deletion in pruning
            }

            if (ray->crn_tgt == u_src->crn()) // make sure the mkey is not checked (for start crnreflect, bcos proj is always from crn_tgt coord and key)
            {
                assert(u_src->isSY() || u_src->isEY());
                _dbg11("[T:RaySrc] BR,HT: Prune Sy/Ey.");
                has_prune = true;
            }
            else
            {
                // assert(side_traced != u_src->side_tgt); // nope
                assert(u_src->isSrcU() || (u_src->isSrcY() && ray->crn_tgt != u_src->crn()));
                // ti_cast = nullptr; // set to not castable.
                _tracerErase(TreeDir::Src, ti_src, status.srcs);
                return true; // cannot prune / check edge anymore. // rays are collided or prior rays on reverse direction.
            }
        }
        else if (brtype == BeyondRayType::BeyondRayRefoundTrace)
        {
            if (ray->crn_tgt == u_src->crn()) // sy is prunable // make sure the mkey is not checked (for start crnreflect, bcos proj is always from crn_tgt coord and key)
            {
                assert(u_src->isSY() || u_src->isEY());
                _dbg11("[T:RaySrc] BR,FT: Prune Sy/Ey. USrc{ " << u_src << " }");
                has_prune = true;
            }
            else
            {
                // assert(side_traced != u_src->side_tgt);
                _dbg11("[T:RaySrc] BR,FT: Stop bcos RevRay. USrc{ " << u_src << " }");
                for (TraceRev &rev : status.revs)
                    rev.u->removeTgt(nullptr);
                status.revs.clear(); // pointless to continue, since reverse ray
                // ti_cast = nullptr; // set to not castable.

                _tracerErase(TreeDir::Src, ti_src, status.srcs);
                return true;
            }
        }

        if (has_prune == true)
        {
            Unit *const u_ssrc = u_src->src();
            _dbgtitle("[T:RaySrc] At Cur(" << status.crn_cur << "), Prune USrc");
            _dbginc; // 1
            _dbg11("[T:RaySrc] Prune USrc{ " << u_src << " }");
            _dbg11("[T:RaySrc]  wrt USSrc{ " << u_ssrc << " }");

            assert(u_src->isSrcY());
            assert(u_ssrc->isSrcY());

            if (u_src->numTgts() == 0)
            {
                _dbg11("[T:RaySrc] Erase Src because no more Tgts");
                u_ssrc->removeTgt(u_src);
                u_src->erase(EraseState::Del);
            }

            status.ray_left = u_src->ray_left;
            status.ray_right = u_src->ray_right;
            ti_src->u = u_ssrc;
            ti_src->v_curFromRel = status.crn_cur->coord - u_ssrc->coord();
            ti_src->v_prog = ti_src->v_curFromRel;
            assert(ti_src->cnt_prog == 0);
            assert(ti_src->state == TraceState::Normal);
            _dbg11("[T:RaySrc] Status adjusted to { " << status << " }");

            _dbgdec;     // 1
            return true; // always taut after pruning, no change to ti_src
        }

        // not beyond ray
        return false;
    }

    bool R2::_tracerPruneSrc(TraceInfo *&ti_src, TraceStatus &status)
    { // returns success if pruned, fail if refound cur, normal if taut.
        Unit *const u_src = ti_src->u;

        if (ti_src->u->isSrcY())
            return false; // should have been pruned in ray src checks.

        // =================== Ignore Src ========================
        if ((ti_src->state == TraceState::Edge || ti_src->state == TraceState::Reverse))
        {
            assert(ti_src->u_spec != nullptr);
            if (ti_src->u == ti_src->u_spec) // ignore special srcs
                return false;
        }

        assert(u_src->isSrcY() == false);
        Unit *const u_ssrc = u_src->src();
        assert(u_src->isSU() || u_src->isEU());
        V2 v_srcFromSSrc = u_src->coord() - u_ssrc->coord();
        assert(ti_src->v_curFromRel == status.crn_cur->coord - u_src->coord());
        if (isPrunable(u_src->side_tgt, ti_src->v_curFromRel, v_srcFromSSrc) == false)
            return false;

        // =================== Prune Src ========================
        _dbgtitle("[T:PrnSrc] At Cur(" << status.crn_cur << "), Prune USrc");
        _dbginc; // 1
        _dbg11("[T:PrnSrc] Prune USrc{ " << u_src << " }");
        _dbg11("[T:PrnSrc]  wrt USSrc{ " << u_ssrc << " }");

        if (ti_src->state == TraceState::Encountered)
        {
            assert(ti_src->u_spec != nullptr);
            if (u_src == ti_src->u_spec)
            {
                ti_src->u_spec = nullptr;
                ti_src->state = TraceState::Normal;
                _dbg11("[T:PrnSrc] Pruned Encountered Unit (USrc)");
            }
        } // this part is required, search later tries to find uspec, which when pruned, will not exist in the src tree.

        assert(u_src->src() == u_ssrc);
        assert(u_ssrc->hasTgt());

        status.ray_left = u_src->ray_left;
        status.ray_right = u_src->ray_right;
        ti_src->u = u_ssrc;
        ti_src->v_curFromRel = status.crn_cur->coord - u_ssrc->coord();
        ti_src->v_prog = ti_src->v_curFromRel;
        assert(ti_src->state == TraceState::Normal || ti_src->state == TraceState::Encountered ||
               ((ti_src->state == TraceState::Edge || ti_src->state == TraceState::Reverse) && u_src != ti_src->u_spec));
        assert(ti_src->cnt_prog == 0);
        _dbg11("[T:SrcPt] Status adjusted to { " << status << " }");

        if (u_src->numTgts() == 0)
        {
            _dbg11("[T:PrnSrc] Erase Src because no more Tgts");
            u_ssrc->removeTgt(u_src);
            u_src->erase(EraseState::Del);
        }

        _dbgdec;     // 1
        return true; // no change to ti_src
    }

    bool R2::_tracerEdgeSrc(TraceInfo *&ti_src)
    { // returns success if in edge, normal if not
        // ============ Ignore Start ===================
        if (isStart(ti_src->u) == true)
            return false;
        else if (ti_src->state != TraceState::Normal)
            return false; // there are no edges to check even for those other units added in current trace

        assert(ti_src->state == TraceState::Normal);

        Unit *&u_src = ti_src->u;
        V2 &v_curFromSrc = ti_src->v_curFromRel;
        assert(u_src->crn()->is_convex == true);

        const Side side_traced_src = u_src->side_tgt; // avoid using ref bcos tracer uses ref, and to guard against u_src deletion (in prunes)
        Corner *crn_src = u_src->crn();
        dir_idx_t di_src = crn_src->edgeDi(side_traced_src);
        const V2 v_src = dirIdxToDir(di_src);

        // ============ Check Edges ===================
        if (isLeftOrRight<true>(side_traced_src, v_src, v_curFromSrc))
        { // in edge
            _dbgtitle("[T:EdgSrc] Entered Tgt edge of USrc{ " << u_src << " }");
            _dbginc; // 1
            assert(ti_src->state == TraceState::Normal);
            assert(ti_src->u_spec == nullptr);

            // elevate to tracestate::edge
            ti_src->state = TraceState::Edge;
            ti_src->u_spec = ti_src->u;
            _dbg11("[T:EdgSrc] Trace will spawn rcr Trace after making sure cur is all progressed. TiSrc{ " << ti_src << " }");

            _dbgdec; // 1
            return true;
        }
        return false;
    }

    bool R2::_tracerProcessAbnormal(TraceStatus &status)
    {
        TraceInfo *const &ti_src = status.srcs.front();

        // =================== Spawn recursive traces / or queue interim trace if is enc / edge / reverse ========================
        assert(status.srcs.size() == 1);
        if (ti_src->state != TraceState::Normal && status.all_progressed == true)
        {
            _dbgtitle("[T:ProcNN] Non-Normal State and all Srcs & Tgts Progressed");
            _dbginc; // 2
            assert(ti_src->cnt_prog == 0);
            assert(ti_src->u_spec != nullptr);

            // ========= Create Ps =============
            Unit *u_src = ti_src->u;
            Unit *u_tgt = _tracerMakeTRorPS(UnitType::PS, false, status.side, status.crn_cur, status.tgts);
            u_tgt->ray_left = status.ray_left;
            u_tgt->ray_right = status.ray_right;
            u_src->linkTgt(u_tgt);
            _dbg11("[T:ProcNN] Created Ps at Cur. UPs{" << u_tgt << "}");

            // =========== Convert all prior points (before cur, if any) to Tu ==================
            while (u_src != ti_src->u_spec)
            { // link all tgts
                u_src->convToTgtType(UnitType::TU, u_src->findCostTgt());
                _dbg11("[T:ProcNN] Conv. non spec to UTu{" << u_src << "}");
                u_tgt = u_src;
                u_src = u_src->src();
            }
            assert(u_src == ti_src->u_spec);

            if (ti_src->state == TraceState::Encountered)
            {
                assert(u_src == ti_src->u_spec);
                assert(ti_src->u_spec->crn() != status.crn_cur);
                assert(u_src->numTgts() >= 1);
                u_src->convToTgtType(UnitType::TU, u_src->findCostTgt());
                _dbg11("[T:ProcEnc] Converted UEnc to{ " << u_src << " }");
                _tracerAddEncUnits(u_src);
            }
            else if (ti_src->state == TraceState::Edge)
            {
                _dbg11("[T:ProcEdg] Setup Status for Recursive in-Edge Trace");
                TraceStatus new_status(u_src->side_tgt, u_src->crn(), u_tgt->ray_left, u_tgt->ray_right); // must take the rays from the tgt in case of tu
                u_tgt->unlinkSrc(u_src);
                new_status.initSrcWithProg(u_src, new_status.v_cur);
                new_status.initTgtWithProg(u_tgt, u_src->coord() - u_tgt->coord());
                status.srcs.clear(); // clear bcos all srcs and tgts are transferred into tracer
                status.tgts.clear();
                _tracerTrace(new_status);

                _dbg11("[T:ProcEdg] Begin Recursive in-Edge Trace");
                tracer(new_status);
            }
            else
            {
                assert(ti_src->state == TraceState::Reverse);
                _dbg11("[T:ProcRev] Setup Status for Reverse Trace");
                // u_src->removeTgt(nullptr); cannot remove nullptr here, if trace is rejected before reaching here, nullptr is dangling.
                Ray *const &ray = u_tgt->ray(status.side); // must take the rays from the tgt in case of tu(where status rays != tu rays)
                assert(ray != nullptr);
                Corner *const &crn_rev = ray->collision->crn(!status.side);
                TraceStatus new_status(!status.side, crn_rev, u_tgt->ray_left, u_tgt->ray_right);
                u_tgt->unlinkSrc(u_src);
                new_status.initSrcWithProg(u_src, ray->vec);
                // assumes status.v_prev is a unit vector.
                new_status.initTgtWithProg(u_tgt, crn_rev->coord - new_status.v_prev - u_tgt->coord()); // cannot use other side due to oom. can move one cell in direction of previous trace to get correct unit vector
                status.srcs.clear();                                                                    // clear bcos all srcs and tgts are transferred into tracer
                status.tgts.clear();

                _dbg11("[T:ProcRev] Begin Reverse Trace");
                tracer(new_status);
            }
            _dbgdec; // 2
            return true;
        }
        return false;
    }

    bool R2::_tracerPointSrc(TraceInfo *const &ti_src, TraceStatus &status)
    { // returns success if all queued. // normal otherwise
        assert(status.crn_cur->is_convex == true);

        // ignore cur if tracer is not progressed
        if (ti_src->cnt_prog > 0)
            return false; // must be progressed

        assert(ti_src->v_curFromRel == status.crn_cur->coord - ti_src->u->coord());
        assert(status.v_cur != status.v_prev);

        // Ignore cur if no ang dir changed
        if (angDirChanged(status.side, ti_src->v_curFromRel, status.v_cur) == false)
            return false;

        Node *node_cur = tryEmplaceNode(status.crn_cur).first;
        // =================== (merge?) ==============
        if (ti_src->u->isSrcY() && node_cur->cost_src_min < INF)
        {
            Ray *ray = tryEmplaceRay(TreeDir::Src, ti_src->u->crn(), status.crn_cur).first;
            if (ray->vis == Vis::Yes)
            { // visible

                // ray must always replace the existing !side ray in status, bcos otherwise, it will be BR
                for (Unit *const &u_src_tgt : ti_src->u->tgts)
                {
                    if (u_src_tgt != nullptr && u_src_tgt->isSrcY() && u_src_tgt->crn() == status.crn_cur)
                    { // if share the same src (i.e. all paths before are the same), then any visible cur point will inherit the utype of the merged unit bcos same cost_src
                        assert(status.ray(!status.side) == nullptr || isLeftOrRight<false>(status.side, ray->vec, status.ray(!status.side)->vec));
                        assert(u_src_tgt->ray(!status.side) == ray);
                        // status.ray(!status.side) = ray;
                        // _dbg11("[T:SrcPt] Replace !side Ray of Status to Ray. Status{" << status << "}");
                        if (u_src_tgt->ray(status.side) != status.ray(status.side))
                        {
                            _dbgtitle("[T:SrcPt] Cannot Merge to SyTgt bcos different rays. SyTgt{ " << u_src_tgt << " }");
                            _dbginc;
                            _dbg11("[T:SrcPt]     Expected L Ray{" << status.ray_left << "}");
                            _dbg11("[T:SrcPt]     Expected R Ray{" << status.ray_right << "}");
                            _dbgdec;
                        }
                        else
                        {
                            _dbg11("[T:SrcPt] Merge UCur to SyTgt of USrc. SyTgt{ " << u_src_tgt << " }");
                            ti_src->u = u_src_tgt;
                            ti_src->v_prog = status.v_cur;
                            status.ray_left = status.side == Side::L ? ray : nullptr;
                            status.ray_right = status.side == Side::L ? nullptr : ray;
                            // don't count this as a point, to avoid conversion to ps/tr at cur
                            return true;
                        }
                    } // if a sibling that is Sy/Ey and same corner found
                }     // for each sibling in src
                // create sy if not mergeable and cost is equal
                float_t cost_src = ti_src->u->cost_src + ray->vec.norm();
                if (approxEq(cost_src, node_cur->cost_src_min))
                {
                    _dbg11("[T:SrcPt] Create Sy bcos u_src is Sy and Node has eq cost");
                    ti_src->u = _tracerMakeSrc(UnitType::SY, status.side, status.crn_cur, ti_src->u);
                    ti_src->u->ray(status.side) = status.ray(status.side);
                    ti_src->u->ray(!status.side) = ray;
                    _dbg11("[T:SrcPt] Create New UCur{ " << ti_src->u << " }");
                    ti_src->v_prog = status.v_cur; // update prog, ignore v_curFromRel, is updated later
                    status.ray(status.side) = ray;
                    status.ray(!status.side) = nullptr;
                    _dbg11("[T:SrcPt] Status adjusted to { " << status << " }");
                    return true;
                }
                // if not mergeable, create su and see if encountered
            } // ray is VY
        }

        ++status.num_points;
        // =================== Src is Su/Eu --Or-- Src is Sy/Ey and Ray is VU/VN to Cur  =======================
        UnitType utype_cur = ti_src->u->isS() ? UnitType::SU : UnitType::EU;
        ti_src->u = _tracerMakeSrc(utype_cur, status.side, status.crn_cur, ti_src->u);
        ti_src->u->ray_left = status.ray_left;
        ti_src->u->ray_right = status.ray_right;
        _dbg11("[T:SrcPt] Create New UCur{ " << ti_src->u << " }");
        ti_src->v_prog = status.v_cur; // update prog, ignore v_curFromRel, is updated later
        status.ray_left = nullptr;
        status.ray_right = nullptr;
        _dbg11("[T:SrcPt] Status adjusted to { " << status << " }");
        return true;
    }

    void R2::_tracerAddEncUnits(Unit *const &u_cur)
    {
        // ================== 2. Add all other Su units in node ================
        assert(u_cur->node->units.front() == u_cur);           // u_cur is always at the front.
        for (Unit *u = u_cur->next; u != nullptr; u = u->next) // note must check from u_cur->next
        {
            if (u->isSrcU() == false)
            {
#if P2D_DEBUG
                for (Unit *u_ = u_cur->next; u_ != nullptr; u_ = u_->next)
                    assert(u->isSrcU() == false); // guarantee due to insertion from front, beginning u_cur is always inf ront
#endif
                break;
            }

            if (u->isSU())
            {
                units_enc.push_back(u);
                u->addTgt(nullptr); // add nullptr tgt to prevent deletion
                _dbg11("[T:ProcEnc] Pushed USu in Node to UnitsEnc. NodeUSu{ " << u << " }");
            }
        }

        // ========= 3. Add UEnc into units_enc ===============
        units_enc.push_back(u_cur);
        u_cur->addTgt(nullptr);
    }

    bool R2::_tracerEncounteredTree(TraceStatus &status)
    {
        assert(status.srcs.size() == 1);

        TraceInfo *const &ti_src = status.srcs.front();
        Unit *u_cur = ti_src->u;
        assert(u_cur->crn() == status.crn_cur);

        // ignore enc tree if is ex, or if there is a merged point
        if (u_cur->isSrcY() || u_cur->isEU() || ti_src->state != TraceState::Normal)
            return false;

        assert(ti_src->state == TraceState::Normal);
        assert(ti_src->u_spec == nullptr);

        assert(u_cur->isS()); // cannot be EU or EY
        assert(u_cur->src()->isS());
        assert(u_cur->src()->hasTgt(u_cur) == true);

        if (u_cur->node->units.size() == 1)
            return false; // only itself

        // ================== 1. More Than one Unit ================
        _dbgtitleheavy("[T:EncTree] ENC: NodeCur{ " << u_cur->node << " }");
        _dbginc; // 1
        ti_src->state = TraceState::Encountered;
        ti_src->u_spec = u_cur;

#if P2D_DEBUG
        for (Unit *u = u_cur->node->units.front(); u != nullptr; u = u->next)
            _dbg11("[T:EncTree] Unit(" << u << ")");
#endif

        if (status.all_progressed == true)
        {
            _dbg11("[T:EncTree] ENC: All Tgts admissible. Exit Tracer");
            _tracerAddEncUnits(u_cur);
            // convert u_cur to ps
            assert(u_cur->cost_tgt == INF);
            for (TraceInfo *ti_tgt = status.tgts.front(); ti_tgt != nullptr; ti_tgt = ti_tgt->next)
            {
                assert(ti_tgt->u->hasSrc() == false);
                u_cur->linkTgt(ti_tgt->u);
                assert(ti_tgt->v_curFromRel == u_cur->coord() - ti_tgt->u->coord());
                float_t cost_tgt = ti_tgt->u->cost_tgt + ti_tgt->v_curFromRel.norm();
                if (cost_tgt < u_cur->cost_tgt)
                    u_cur->cost_tgt = cost_tgt;

                // overwrite the rays
                ti_tgt->u->ray_left = status.ray_left;
                ti_tgt->u->ray_right = status.ray_right;
            }
            u_cur->convToTgtType(UnitType::PS, u_cur->cost_tgt);
        }

        _dbgdec; // 1
        return status.all_progressed;
    }

    bool R2::_tracerQueueSrc(TraceInfo *const &ti_cur, TraceStatus &status)
    {
        if (ti_cur->state != TraceState::Normal)
        {
            assert(ti_cur->u_spec != nullptr);
            return false; // can only queue as castable / interim trace if normal state
        }

        Unit *u_cur = ti_cur->u;
        assert(isStart(u_cur) == false);
        assert(ti_cur->v_curFromRel == u_cur->coord() - u_cur->src()->coord()); // vcfs not updated after adding point at cur
        assert(ti_cur->state == TraceState::Normal);
        assert(ti_cur->cnt_prog == 0);

        // ================= 6. Separate Castable Tgts from Non-castable tgts =================
        std::vector<Unit *> u_castable_tgts;
        for (TraceInfo *ti_tgt = status.tgts.front(); ti_tgt != nullptr;)
        {
            TraceInfo *ti_next = ti_tgt->next;
            Unit *const &u_tgt = ti_tgt->u;
            assert(ti_tgt->v_curFromRel == u_cur->coord() - u_tgt->coord()); // requires tgt process is run first, and tgts are admissible (i.e. pruned)
            assert(u_tgt->numSrcs() == 0);

            if (ti_tgt->cnt_prog > 0)
            {
                _dbg11("[T:SrcQ] Cannot queue for Non-progressed Tgt { " << ti_tgt << " }");
                ti_tgt = ti_next;
                continue;
            }

            if (isCastable(status.side, ti_cur->v_curFromRel, ti_tgt->v_curFromRel, status.v_cur))
            {
                _dbgtitle("[T:SrcQ] CASTABLE to UTgt{ " << u_tgt << " }");
                _dbginc; // 1
                _dbg11("[T:SrcQ]      from UTurnpt{ " << u_cur << " }");
                _dbg11("[T:SrcQ]       wrt    USrc{ " << u_cur->src() << " }");
                _dbgdec; // 1

                u_castable_tgts.push_back(u_tgt);
                status.tgts.erase(ti_tgt, EraseState::Del);
            }
            ti_tgt = ti_next;
        }

        // ================= 7. Queue Castable Queries =================
        if (u_castable_tgts.empty() == false)
        {
            _dbgtitle("[T:SrcQ] Stage and Queue CASTABLES");
            _dbginc; // 1

            for (Unit *const &u_castable_tgt : u_castable_tgts)
            {
                _dbgtitle("[T:SrcQ] Queue Query to CASTABLE UTgt{ " << u_castable_tgt << " }");
                _dbginc; // 2
                u_castable_tgt->linkSrc(u_cur);
                assert(u_castable_tgt->src() == u_cur);
                assert(u_cur->hasTgt(u_castable_tgt));

                float_t f = u_cur->cost_src + u_castable_tgt->cost_tgt + norm(u_cur->coord(), u_castable_tgt->coord());
                u_castable_tgt->ray_left = status.ray_left;
                u_castable_tgt->ray_right = status.ray_right;
                if (u_castable_tgt->isTR())
                    queueQuery(QueryType::Trace, u_cur, u_castable_tgt, f);
                else
                    queueQuery(QueryType::Cast, u_cur, u_castable_tgt, f);
                _dbgdec; // 2
            }

            if (u_cur->isEU())
            {
                _dbg11("[T:SrcQ] Push Castable Eu UTurnpt to UEnc"); // always go to src to prune aggressively
                units_enc.push_back(u_cur);
                u_cur->addTgt(nullptr);
            }
            _dbgdec; // 1
        }

        // =================-- 5. Stop Trace if no more Tgts in trace =================-
        if (status.tgts.empty() == true)
        {
            _dbg11("[T:SrcQ] No more UTgts after Casting. Stop Trace");
            return true;
        }

        // =================-- 6. Created Point but there are still some uncastables =================-
        if (status.num_points > NUM_POINTS_TO_QUEUE)
        {
            if (status.all_progressed == false)
            {
                _dbg11("[T:SrcQ] Cannot queue interim trace due to all points not progresesed.");
                return false;
            }

            _dbgtitle("[T:SrcQ] Queue SVU as UTr bcos " << status.num_points << " points have been created in this tracer");
            _dbginc; // 1
            float_t cost_src = u_cur->cost_src;
            u_cur = toTgt(UnitType::TR, TreeDir::Tgt, u_cur, nullptr, u_cur->ray_left, u_cur->ray_right, INF); // u_cur may have been used in cast.
            assert(u_cur->hasSrc() == true);
            for (TraceInfo *ti_tgt = status.tgts.front(); ti_tgt != nullptr; ti_tgt = ti_tgt->next)
            {
                Unit *const &u_tgt = ti_tgt->u;
                assert(u_tgt->hasSrc() == false);
                u_tgt->linkSrc(u_cur);
                u_tgt->ray_left = status.ray_left;
                u_tgt->ray_right = status.ray_right;
                float_t cost_tgt = u_tgt->cost_tgt + norm(u_tgt->coord(), u_cur->coord());
                if (cost_tgt < u_cur->cost_tgt)
                    u_cur->cost_tgt = cost_tgt;
            }

            float_t f = cost_src + u_cur->cost_tgt;
            queueQuery(QueryType::Trace, u_cur->src(), u_cur, f);
            _dbgdec; // 1
            return true;
        }

        return false; // can continue trace
    }

    bool R2::_tracerProcessTgt(TraceStatus &status)
    {
        [[maybe_unused]] bool pruned_tgt;
        for (TraceInfo *ti_tgt = status.tgts.front(); ti_tgt != nullptr;)
        {
            assert(ti_tgt->u->ray_left == nullptr);
            assert(ti_tgt->u->ray_right == nullptr);
            // ti_tgt->v_curFromRel = status.crn_cur->coord - u_tgt->coord();

            // =========================== Check if Refound Tgt ========================
            if (_tracerRefoundRel(TreeDir::Src, status.crn_cur, ti_tgt, status.tgts))
                continue;

            // ============ Skip if not Progressed Tgt =====================
            if (_tracerNotProgressed(true, ti_tgt, status) == true)
            {
                ti_tgt = ti_tgt->next;
                assert(status.all_progressed == false);
                continue;
            }

            // ============ Prune or Do Edge Check first depending on side traced =====================
            if (_tracerPruneTgt(ti_tgt, status) == true)
                continue; // skip prune bcos ti_tgt is deleted. pruned tgts at back of ti_tgts

            // // ti_tgt is next if pruned or in edge, otherwise, is unchanged
            // if (status.from_src == (u_tgt->side_tgt == status.side))
            // { // side tgt same: prune first, edge second

            //     // if (_tracerEdgeTgt(crn_cur, ti_tgt, ti_tgts) == true)
            //     //     continue; // may have deleted ti_tgt
            // } // side tgt same
            // else
            // { // side tgt different: edge first, prune second
            //     if (_tracerEdgeTgt(status.crn_cur, ti_tgt, status.tgts) == true)
            //         continue; // skip prune. always taut. May have deleted ti_tgt

            //     // if (_tracerPruneTgt(crn_cur, ti_tgt, ti_tgts) == true) // ti_tgt is next if pruned
            //     //     continue;
            // }
            // no edge or prune
            ti_tgt = ti_tgt->next;
        }

        // ============ Stop Trace if No more Tgts =====================
        if (status.tgts.empty())
        {
            _dbg11("[T:ProcTgt] Erase UnitTree (Src) bcos no more Tgts");
            return true;
        }

        return false;
    }

    bool R2::_tracerPruneTgt(TraceInfo *&ti_tgt, TraceStatus &status)
    {
        Unit *&u_tgt = ti_tgt->u;
        // ==================== Skip goal ===========================
        if (isGoal(u_tgt))
            return false;

        V2 &v_curFromTgt = ti_tgt->v_curFromRel;
        assert(v_curFromTgt == status.crn_cur->coord - u_tgt->coord());
        const Side side_traced_tgt = u_tgt->side(Src);

        // // =================== Reject searches if enter edge of tgt (not workable) ==================
        // if ((u_tgt->isTU() || u_tgt->isTY()) && u_tgt->crn()->is_convex == true && u_tgt->node->cost_src_min < INF)
        // {
        //     V2 v_srcOfTgt = u_tgt->crn()->edgeVec(side_traced_tgt);
        //     if (isLeftOrRight<true>(side_traced_tgt, v_srcOfTgt, v_curFromTgt))
        //     {   //in source edge of tgt
        //         assert(status.srcs.size() == 1);
        //         TraceInfo *const &ti_src = status.srcs.front();
        //         assert(ti_src->v_curFromRel == status.crn_cur->coord - ti_src->u->coord());
        //         float_t cost_src_to_reach = ti_src->u->cost_src + ti_src->v_curFromRel.norm() + v_curFromTgt.norm();
        //         if (cost_src_to_reach > u_tgt->node->cost_src_min)
        //         {
        //             _dbg11("Reject More Expensive search bcos u_tgt->node is already reached");
        //             _tracerErase(TreeDir::Tgt, ti_tgt, status.tgts);
        //             return true;
        //         }
        //     }
        // }

        for (auto it_u_ttgt = u_tgt->tgts.begin(); it_u_ttgt != u_tgt->tgts.end();)
        {
            Unit *const u_ttgt = *it_u_ttgt;
            const V2 v_tgtFromTTgt = u_tgt->coord() - u_ttgt->coord();

            // ==================== Go To Next TTgt if Taut ===========================
            auto p = dtrLeftOrRight<false>(side_traced_tgt, v_curFromTgt, v_tgtFromTTgt); // not opt taut
            // if (u_ttgt->crn() == status.crn_cur)
            // {
            //     _dbg11("TTgt at same position as Cur");
            //     it_u_ttgt = u_tgt->tgts.erase(it_u_ttgt); // erase from u_tgt
            //     continue;
            // }

            bool prunable = p.second;
            if (p.first == 0)
                prunable = dot(v_curFromTgt, v_tgtFromTTgt) >= 0; // B prunable if either is zero (A on B, or B on C), or if B is coliear and lies between A-C.

            if (prunable == false)
            { // works for ncv pseudos and standard cv
                ++it_u_ttgt;
                continue; // taut, continue
            }

            _dbgtitle("[T:PrnTgt] Cur(" << status.crn_cur << ") PRUNE Tgt( " << u_tgt->crn() << " ) wrt TTgt( " << u_ttgt->crn() << " )");
            _dbginc; // 1
            _dbg11("[T:PrnTgt]  UTgt { " << u_tgt << " }");
            _dbg11("[T:PrnTgt] UTTgt { " << u_ttgt << " }");

            // ==================== Check if prunable the correct way =========================
            V2 v_prev = status.v_prev;
            long_t dtr_cft_prev = det(v_curFromTgt, v_prev);
            // assert(dtr_cft_prev != 0); // shouldn't be prunable now
            // assert(dtr_tftt_prev != 0); // shouldn't be prunable now

            if (dtr_cft_prev == 0)
            {
                Corner *crn_prev = traceCache(status.crn_cur, !status.side);
                assert(crn_prev != nullptr);
                // assert(crn_prev->mkey == u->crn()->mkey);
                dir_idx_t unit_di = crn_prev->di_occ;                                                                     // works for both rel is at corner, or on edge
                if (crn_prev->mkey == u_tgt->crn()->mkey && addDirIdx<true>(crn_prev->di_occ, 4) == u_tgt->crn()->di_occ) // checkerboard crn
                    unit_di = u_tgt->crn()->di_occ;                                                                       // need extra care for checkerboard crn
                v_prev = status.crn_cur->coord - crn_prev->coord - dirIdxToDir(unit_di);
                dtr_cft_prev = det(v_curFromTgt, v_prev);
                _dbg11("[T:PrnTgt] Cft//v_prev. Change vPrev to (" << v_prev << ")");
            }

            long_t dtr_tftt_prev = det(v_tgtFromTTgt, v_prev);

            if (sgn(dtr_cft_prev) * sgn(dtr_tftt_prev) < 0)
            { // trace intersected ray from tgt in ttgt direction
                _dbgtitle("[T:PrnTgt] Cannot Prune Tgt (in wrong direction)");
                _dbginc; // 2
                V2 v_curFromTTgt = status.crn_cur->coord - u_ttgt->coord();

                // if (dtr_cft_prev == -dtr_tftt_prev)
                // {
                //     // implies intersection at ttgt (ttgt on trace prev. delete)
                //     _dbg11("[T:PrnTgt] TTgt lies on Trace. Delete TTgt if possible"); // case b in notes
                //     u_ttgt->removeSrc(u_tgt);
                //     eraseUnitTree(TreeDir::Tgt, u_ttgt, false);
                //     it_u_ttgt = u_tgt->tgts.erase(it_u_ttgt); // erase from u_tgt
                // }
                if (std::abs(dtr_cft_prev) > std::abs(dtr_tftt_prev))
                {
                    _dbg11("[T:PrnTgt] Trace (intx pt) lies further than TTgt from Tgt. Create Td at TTgt as new Tgt"); // case b in notes
                    assert(u_tgt->isTR() == false);
                    Unit *new_u_tgt = u_tgt->unzipToTgt(u_tgt->type, {}, {}, nullptr, nullptr, u_tgt->cost_tgt);
                    assert(u_tgt->hasSrc() == false);
                    u_ttgt->replaceSrc(u_tgt, new_u_tgt);
                    new_u_tgt->addTgt(u_ttgt);

                    // add to front of ttgts
                    float_t cost_tgt = u_tgt->cost_tgt + v_tgtFromTTgt.norm();
                    Unit *new_u_ps = u_ttgt->node->units.emplaceTgtType(!status.side, UnitType::TD, {}, {new_u_tgt}, nullptr, nullptr, cost_tgt); // must be reverse of side for it to be taut later.
                    status.tgts.emplace(ListDir::Front, new_u_ps, v_curFromTTgt, v_curFromTTgt);
                    assert(status.tgts.front()->cnt_prog == 0);

                    _dbg11("[T:PrnTgt] Unzipped UTgt(tgt of TD){ " << new_u_tgt << " }");
                    _dbg11("[T:PrnTgt]   Created TD(as new tgt){ " << new_u_ps << " }");
                    _dbg11("[T:PrnTgt]      Placed TiTd @ front{ " << status.tgts.front() << " }");

                    it_u_ttgt = u_tgt->tgts.erase(it_u_ttgt); // erase from u_tgt
                }
                else
                {
                    // dtr_cft_prev == -dtr_tftt_prev // ttgt is on intersection / previous trace.
                    _dbg11("[T:PrnTgt] Trace (intx pt) lies on/ closer than TTgt from Tgt. Create Ps at Cur, betw Tgt (Src of Ps) and TTgt (TTgt of Ps) "); // case a in notes

                    Node *node_cur = tryEmplaceNode(status.crn_cur).first;
                    float_t cost_tgt = u_ttgt->cost_tgt + v_curFromTTgt.norm();
                    Unit *new_u_ps = node_cur->units.emplaceTgtType(!status.side, UnitType::TD, {}, {}, u_ttgt->ray_left, u_ttgt->ray_right, cost_tgt);
                    u_ttgt->ray_left = nullptr;
                    u_ttgt->ray_right = nullptr;
                    assert(u_tgt->hasSrc() == false);
                    u_ttgt->replaceSrc(u_tgt, new_u_ps);
                    new_u_ps->addTgt(u_ttgt);
                    new_u_ps->addSrc(u_tgt);
                    u_tgt->replaceTgt(u_ttgt, new_u_ps); // does not invalidate iterator
                    u_tgt->cost_tgt = u_tgt->findCostTgt();

                    _dbg11("[T:PrnTgt] Created Ps(replacing ttgt){ " << new_u_ps << " }");
                    _dbg11("[T:PrnTgt]                    For Tgt{ " << u_tgt << " }");
                    ++it_u_ttgt;
                }
                _dbgdec; // 2
            }
            else
            { // prune normally (intersected ray from tgt in opposite ttgt direction)
                // ==================== Unzip / Conv to UTgt (Final UTgt has no srcs) ====================
                _dbgtitle("[T:PrnTgt] Promote UTTgt(" << u_ttgt->repr(0) << ")");
                _dbginc; // 2
                Unit *promoted_u_ttgt;
                u_ttgt->removeSrc(u_tgt); // uttgt removed from utgt later
                if (u_ttgt->isTR())       // convert from trace type to ps type due to pruning (can no longer trigger trace when castable to ttgt)
                    u_ttgt->type = UnitType::PS;

                if (u_ttgt->numSrcs() > 0) // unzip if more than 1 srcs
                {
                    assert(u_ttgt->isT() || u_ttgt->isPS() || u_ttgt->isTR() || u_ttgt->isTD());
                    promoted_u_ttgt = u_ttgt->unzipToTgt(u_ttgt->type, {}, u_ttgt->tgts, nullptr, nullptr, u_ttgt->cost_tgt);
                    _dbg11("[T:PrnTgt] Unzipped promoted UTTgt because >1 Src. Unzipped UTTgt{ " << promoted_u_ttgt << " }");
                }
                else
                {
                    promoted_u_ttgt = u_ttgt;
                    promoted_u_ttgt->ray_left = nullptr;
                    promoted_u_ttgt->ray_right = nullptr;
                }

                assert(promoted_u_ttgt->numSrcs() == 0);
                _dbgdec; // 2

                V2 v_curFromTTgt = status.crn_cur->coord - promoted_u_ttgt->coord();
                status.tgts.emplace(ListDir::Back, promoted_u_ttgt, v_curFromTTgt, v_curFromTTgt); // push to back to check if prunable / edge recur later
                assert(status.tgts.back()->cnt_prog == 0);

                // Erase UTTgt from UTgt
                it_u_ttgt = u_tgt->tgts.erase(it_u_ttgt); // always erase when prunable (is safe) because tgt does not have src
            }
            _dbgdec; // 1
        }

        // ==================== Push promoted TTgt to Tgts if not refound crn ====================
        if (u_tgt->numTgts() == 0)
        {
            assert(u_tgt->numSrcs() == 0);

            _dbg11("[T:PrnTgt] Erase Tgt because no more Tgts");
            TraceInfo *ti_next = ti_tgt->next;
            u_tgt->erase(EraseState::Del);
            status.tgts.erase(ti_tgt);
            ti_tgt = ti_next;
            _dbg11("[T:PrnTgt] --------");

            return true;
        }
        return false;
    }

    // bool R2::_tracerEdgeTgt(Corner *const &crn_cur, TraceInfo *&ti_tgt, TraceInfoContainer &ti_tgts)
    // {
    //     // ============ Check Goal ===================
    //     Unit *&u_tgt = ti_tgt->u;

    //     if (isGoal(u_tgt))
    //         return false;

    //     assert(ti_tgt->v_curFromRel == crn_cur->coord - ti_tgt->u->coord());
    //     const Side &side_src_of_tgt = u_tgt->side(Src);
    //     Corner *const &crn_tgt = ti_tgt->u->crn();
    //     V2 v_edge = u_tgt->isPS() || u_tgt->isTR() ? crn_tgt->edgeVec(side_src_of_tgt) : crn_tgt->edgeVec(!side_src_of_tgt); // TODO this should be ncv? why is it taken out in the first place

    //     // ============ Check Edges ===================
    //     if (isLeftOrRight<true>(side_src_of_tgt, v_edge, ti_tgt->v_curFromRel))
    //     { // in edge
    //         _dbgtitle("[T:EdgTgt] Cur(" << crn_cur << ") Entered Edge of UTgt{ " << u_tgt << " }");
    //         _dbginc;

    //         TraceInfo *ti_next = ti_tgt->next;
    //         if (tracerTgt(ti_tgt, crn_cur) == true)
    //         { // no more tgts. Remove this ti_tgt
    //             ti_tgts.erase(ti_tgt);
    //             // don't erase unit because it may have been erased in tracerTgt.
    //         }
    //         _dbgdec;
    //         ti_tgt = ti_next;
    //         return true;
    //     }
    //     return false;
    // }

    bool R2::_tracerQueueTgt(TraceStatus &status)
    {
        assert(status.srcs.size() == 1);
        TraceInfo *ti_src = status.srcs.front();

        if (ti_src->state != TraceState::Normal)
            return false; // cannot queue as interim trace if not normal
        else if (status.all_progressed == false)
        {
            _dbg11("[T:TgtQ] Cannot queue as interim trace as some src/tgts are not progressed");
            return false;
        }
        else if (status.tgts.size() != 1)
        {
            _dbg11("[T:TgtQ] Cannot queue as interim trace bcos nto one Ps tgt");
            return false;
        }

        assert(status.tgts.size() == 1);
        TraceInfo *ti_tgt = status.tgts.front();
        Unit *const &u_tgt = ti_tgt->u;
        Unit *const &u_src = ti_src->u;

        if (status.num_points > NUM_POINTS_TO_QUEUE)
        {
            _dbgtitle("[T:TgtQ] Queue Ps as UTr bcos " << status.num_points << " points have been created in this tracer");
            _dbginc; // 1
            u_tgt->convToTgtType(UnitType::TR, u_tgt->cost_tgt);
            u_tgt->linkSrc(u_src);
            assert(u_tgt->hasTgt() == true);
            assert(u_tgt->hasSrc() == true);
            assert(ti_src->v_curFromRel == u_tgt->coord() - ti_src->u->coord());

            float_t f = u_src->cost_src + u_tgt->cost_tgt + ti_src->v_curFromRel.norm();
            u_tgt->ray_left = status.ray_left;
            u_tgt->ray_right = status.ray_right;
            queueQuery(QueryType::Trace, u_src, u_tgt, f);
            _dbgdec; // 1
            return true;
        }

        return false;
    }

    bool R2::_tracerPointPs(TraceStatus &status)
    {
        assert(status.crn_cur->is_convex == false);

        // ================ Find suitable targets to create Ps ==================
        std::vector<TraceInfo *> ps_tgts;
        for (TraceInfo *ti_tgt = status.tgts.front(); ti_tgt != nullptr; ti_tgt = ti_tgt->next)
        {
            if (ti_tgt->cnt_prog >= 1)
                continue; // not progressed

            assert(ti_tgt->v_curFromRel == status.crn_cur->coord - ti_tgt->u->coord());
            if (angDirChanged(!status.side, ti_tgt->v_curFromRel, status.v_cur)) // ncv psd point
                ps_tgts.push_back(ti_tgt);
        }

        // ================ Create Psd for all suitable targets ==================
        if (ps_tgts.empty() == false)
        {
            _dbgtitle("[T:PsdPt] Make Psd for Tgts");
            _dbginc; // 1

            Node *const &node_cur = tryEmplaceNode(status.crn_cur).first;
            Unit *u_ps = node_cur->units.emplaceTgtType(status.side, UnitType::TD, {}, {}, nullptr, nullptr, INF);
            ++status.num_points;

            TraceInfo *ti_ps = status.tgts.emplace(ListDir::Front, u_ps, V2(0, 0), status.v_cur);

            for (TraceInfo *ti_ttgt : ps_tgts)
            {
                Unit *const &u_ttgt = ti_ttgt->u;
                u_ps->linkTgt(u_ttgt);
                assert(ti_ttgt->v_curFromRel == status.crn_cur->coord - u_ttgt->coord());
                float_t cost_tgt = ti_ttgt->v_curFromRel.norm() + u_ttgt->cost_tgt;
                if (cost_tgt < u_ps->cost_tgt)
                    u_ps->cost_tgt = cost_tgt;
                status.tgts.erase(ti_ttgt);
            }
            assert(status.tgts.empty() == false);

            // ================ Update VProg ==================
            ti_ps->v_prog = status.v_cur;
            assert(ti_ps->cnt_prog == 0);

            _dbg11("[T:PsdPt] Created UPs and Ti{ " << ti_ps << " }");

            _dbgdec;
            return true;
        }

        return false;
    }

    bool R2::_tracerPointTu(TraceStatus &status)
    { // can only be used in tgt recur
        if (status.crn_cur->is_convex == false)
            return false;

        // ================ Find suitable targets to create Ps ==================
        assert(status.v_cur == status.crn_cur->edgeVec(status.side));
        assert(status.v_prev == -status.crn_cur->edgeVec(!status.side));
        std::vector<TraceInfo *> demoted_tgts;
        for (TraceInfo *ti_tgt = status.tgts.front(); ti_tgt != nullptr; ti_tgt = ti_tgt->next)
        {
            if (ti_tgt->cnt_prog >= 1)
                continue; // not progressed

            assert(ti_tgt->v_curFromRel == status.crn_cur->coord - ti_tgt->u->coord());
            if (angDirChanged(status.side, ti_tgt->v_curFromRel, status.v_cur)) // convex
                demoted_tgts.push_back(ti_tgt);                                 // ang change from tgt at cur
        }

        // ================ Create Tu for all suitable targets ==================
        if (demoted_tgts.empty() == false)
        {
            _dbgtitle("[T:TuPt] Make Tu for Tgts");
            _dbginc; // 1

            Node *const &node_cur = tryEmplaceNode(status.crn_cur).first;
            Unit *u_tu = node_cur->units.emplaceTgtType(!status.side, UnitType::TU, {}, {}, nullptr, nullptr, INF);
            _dbg11("[T:TuPt] Created UTu{ " << u_tu << " }");

            status.tgts.emplace(ListDir::Front, u_tu, V2(0, 0), status.v_cur);

            for (TraceInfo *ti_ttgt : demoted_tgts)
            {
                Unit *const &u_ttgt = ti_ttgt->u;
                u_tu->linkTgt(u_ttgt);
                assert(ti_ttgt->v_curFromRel == status.crn_cur->coord - u_ttgt->coord());
                float_t cost_tgt = ti_ttgt->v_curFromRel.norm() + u_ttgt->cost_tgt;
                if (cost_tgt < u_tu->cost_tgt)
                    u_tu->cost_tgt = cost_tgt;
                status.tgts.erase(ti_ttgt);
            }
            assert(status.tgts.empty() == false);

            _dbgdec;
            return true;
        }

        return false;
    }

    Unit *R2::_tracerMakeTRorPS(const UnitType &utype, const bool &unzip_tgt, const Side &side_traced, Corner *const &crn_cur, const TraceInfoContainer &ti_tgts)
    {
        assert(utype == UnitType::PS || utype == UnitType::TR || utype == UnitType::TD);
        _dbgtitle("[T:MakTr/Ps] Make " << utype << " at Cur(" << crn_cur << ")");
        _dbginc;
        Node *const &node_cur = tryEmplaceNode(crn_cur).first;
        Unit *u_tr = node_cur->units.emplaceTgtType(side_traced, utype, {}, {}, nullptr, nullptr, INF);

        for (TraceInfo *ti_tgt = ti_tgts.front(); ti_tgt != nullptr; ti_tgt = ti_tgt->next)
        {
            Unit *u_tgt = ti_tgt->u;
            if (unzip_tgt == true)
                u_tgt->unzipToTgt(u_tgt->type, {u_tr}, u_tgt->tgts, u_tgt->ray_left, u_tgt->ray_right, u_tgt->cost_tgt);
            else
                u_tr->linkTgt(u_tgt);
            const float_t cost_tgt = u_tgt->cost_tgt + norm(u_tgt->coord(), crn_cur->coord);
            if (cost_tgt < u_tr->cost_tgt) // no need to use approxGt
                u_tr->cost_tgt = cost_tgt;
        }
        _dbg11("[T:MakTr/Ps] Created U" << utype << "{ " << u_tr << " }");
        _dbgdec;
        return u_tr;
    }
    Unit *R2::_tracerMakeSrc(const UnitType &utype, const Side &side_tgt, Corner *const &crn_cur, Unit *const &u_src)
    {
        assert(utype == UnitType::SU || utype == UnitType::SY || utype == UnitType::EY || utype == UnitType::EU);
        _dbgtitle("[T:Mak" << utype << "] Make " << utype << " at Cur(" << crn_cur << ")");
        _dbginc;
        Node *const &node_cur = tryEmplaceNode(crn_cur).first;
        float_t cost_src = u_src->cost_src + norm(u_src->coord(), crn_cur->coord);
        Unit *u = node_cur->units.emplaceSrcType(side_tgt, utype, {u_src}, {}, nullptr, nullptr, cost_src);
        assert(u->hasSrc(u_src));
        assert(u_src->hasTgt(u));

        // set rayprog
        _dbg11("[T:Mak" << utype << "] Created U" << utype << "{ " << u << " }");
        _dbgdec;

        return u;
    }

    void R2::_tracerUpdate(TraceStatus &status)
    {
        status.all_progressed = true;
        // update v_curFromRel
        for (TraceInfoContainer *const &tis : {&status.srcs, &status.tgts})
            for (TraceInfo *ti = tis->front(); ti != nullptr; ti = ti->next)
                ti->v_curFromRel = status.crn_cur->coord - ti->u->coord();
    }
    void R2::_tracerTrace(TraceStatus &status)
    {
        status.crn_cur = traceCache(status.crn_cur, status.side, status.di);
        assert(status.crn_cur == &null_crn || status.crn_cur->edgeDi(status.side) == status.di);
        status.v_prev = status.v_cur; // should work even for OOM
        status.v_cur = dirIdxToDir(status.di);
    }
}
