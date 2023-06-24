#include "R2/R2.hpp"

namespace P2D::R2
{
    Corner R2::null_crn = Corner();

    std::pair<Corner *, bool> R2::tryEmplaceCrn(const Corner &corner) // does not copy adjacent corners
    {
        return corners.tryEmplace(corner);
    }
    std::pair<Node *, bool> R2::tryEmplaceNode(Corner *const &crn)
    {
        if (crn->node == nullptr)
        {
            Node *node = nodes.emplace(crn);
            crn->node = node;
            return std::make_pair(crn->node, true);
        }
        else
            return std::make_pair(crn->node, false);
    }
    std::pair<Ray *, bool> R2::tryEmplaceRay(const TreeDir &tdir, Corner *const &crn_first, Corner *const &crn_second)
    {
        if (tdir == TreeDir::Src)
            return rays.tryEmplace(crn_first, crn_second);
        else
            return rays.tryEmplace(crn_second, crn_first);
    }
    Ray *R2::getRay(const TreeDir &tdir, Corner *const &crn_first, Corner *const &crn_second)
    {
        if (tdir == TreeDir::Src)
            return rays.at(crn_first, crn_second);
        else
            return rays.at(crn_second, crn_first);
    }

    void R2::initialise()
    {
        assert(Node::_cnt == 0);
        assert(Unit::_cnt == 0);
        assert(Query::_cnt == 0);
        assert(TraceInfo::_cnt == 0);

        path.clear();
        units_enc.clear();
        node_start = new Node(crn_start);
        node_start->cost_src_min = 0;
        crn_start->node = node_start;
        node_goal = new Node(crn_goal);
        node_goal->cost_src_min = INF;
        crn_goal->node = node_goal;
    }
    void R2::initialiseMinimal(const V2 &start_coord, const V2 &goal_coord)
    {
        mapkey_t mkey = grid->coordToKey<false>(start_coord);

        // uint8_t window = 0;
        // for (dir_idx_t di = 1; di < 8; di += 2)
        // {
        //     mapkey_t mkey2 = grid->addKeyToRelKey(mkey, grid->getAdjRelKeyCFV(di));
        //     window <<= 1;
        //     window |= grid->isAccessible(mkey2);
        // }

        // dir_idx_t di_start = 0;
        // if (window == 0b0101)
        // { // can be 3 or 7
        //     V2 v_gfs = goal_coord - start_coord;
        //     V2 v_di = V2(1, 1);                            // dirIdxToDir(1);
        //     if (isLeft<false>(v_gfs, v_di).second == true) // don't care the parallel case (ambiguous)
        //         di_start = 7;
        //     else
        //         di_start = 3;
        // }
        // else if (window == 0b1010)
        // { // can be 1 or 5
        //     V2 v_gfs = goal_coord - start_coord;
        //     V2 v_di = V2(-1, 1);                            // dirIdxToDir(3);
        //     if (isLeft<false>(v_gfs, v_di).second == true) // don't care the parallel case (ambiguous)
        //         di_start = 1;
        //     else
        //         di_start = 5;
        // }

        crn_start = new Corner(true, mkey, 0, start_coord); // initialise to convex to avoid deletion for temporary ncv nodes
        mkey = grid->coordToKey<false>(goal_coord);
        crn_goal = new Corner(true, mkey, 0, goal_coord); // initialise to convex to avoid deletion for temporary ncv nodes
    }
    void R2::terminateMinimal()
    {
        delete crn_start;
        delete crn_goal;
    }
    void R2::terminate()
    {
        queries.clear();
        open_list.clear();
        rays.clear();
        nodes.clear();
        corners.clear();

        delete node_start;
        delete node_goal;

        assert(Node::_cnt == 0);
        assert(Unit::_cnt == 0);
        assert(Query::_cnt == 0);
        assert(TraceInfo::_cnt == 0);
    }

    Corner *R2::traceCache(Corner *const &crn_cur, const Side &side_traced)
    {
        dir_idx_t di_trace = crn_cur->edgeDi(side_traced);
        return traceCache(crn_cur, side_traced, di_trace);
    }
    Corner *R2::traceCache(Corner *const &crn_cur, const Side &side_traced, dir_idx_t &di_trace)
    {
        Corner *crn_next = crn_cur->trace(side_traced);
        if (crn_next == nullptr)
        { // unknown
            Corner crn = *crn_cur;
            if (trace(crn, side_traced, di_trace) == false)
                crn_next = &null_crn; // out of map
            else
            {
                crn_next = tryEmplaceCrn(crn).first;
                crn_next->trace(!side_traced) = crn_cur;
            }
            // _dbg01("TRACE found (" << crn << ") next di(" << int(di_trace) << ")");
            crn_cur->trace(side_traced) = crn_next;
        }
        else
        { // get next
            di_trace = crn_next->edgeDi(side_traced);
            // _dbg01("TRACECACHE found (" << *crn_next << ") next di(" << int(di_trace) << ")");
        }
        return crn_next;
    }

    // returns true if in map, otherwise false
    // assumes that trace does not occur parallel to and on a map boundary.
    // assumes that trace begins at a corner, or on an edge and di_trace is pointing parallel to the edge
    // stops when the trace goes out of map, or a 90 degree turn is required at a corner.
    bool R2::trace(Corner &crn, const Side &side_traced, dir_idx_t &di_trace)
    {
        // dir_idx needs to cardinal (0,2,4,6), and must be pointing parallel to the edge at vertex described by key and coord.
        // key and coord correspond to the same vertex.

        // _dbg10("Trace from (" << crn << ") in di(" << int(di_trace) << ")...");

        assert(isCardinal(di_trace) == true);
        dir_idx_t di = addDirIdx(di_trace, side_traced == Side::L ? 5 : 3);
        mapkey_t obs_cell_key = grid->addKeyToRelKey(crn.mkey, grid->getCellRelKey(di, crn.coord.x));
        di = addDirIdx(di_trace, side_traced == Side::L ? 1 : 7);
        mapkey_t free_cell_key = grid->addKeyToRelKey(crn.mkey, grid->getCellRelKey(di, crn.coord.x));
        const int_t &last_step = grid->getBoundary<false>(di_trace);
        const bool dim_l = di_trace == 2 || di_trace == 6;
        int_t &step = crn.coord[dim_l];
        const int_t step_inc = dirIdxToDir(di_trace)[dim_l];
        const mapkey_t rel_cell_key = grid->getRelKey<true>(di_trace);

        for (; step != last_step; step += step_inc)
        {
            bool free_side_oc = grid->isOc(free_cell_key);
            if (free_side_oc == true)
            { // non convex reached
                di_trace = addDirIdx(di_trace, side_traced == Side::L ? 2 : 6);
                crn.di_occ = addDirIdx(di_trace, side_traced == Side::L ? 5 : 3);
                crn.mkey = grid->coordToKey<false>(crn.coord);
                crn.is_convex = false;
                return true;
            }

            obs_cell_key = grid->addKeyToRelKey(obs_cell_key, rel_cell_key);
            bool obs_side_oc = grid->isOc(obs_cell_key);
            if (obs_side_oc == false)
            { // convex corner
                di_trace = addDirIdx(di_trace, side_traced == Side::L ? 6 : 2);
                crn.di_occ = addDirIdx(di_trace, side_traced == Side::L ? 7 : 1);
                crn.mkey = grid->coordToKey<false>(crn.coord);
                crn.is_convex = true;
                return true;
            }

            free_cell_key = grid->addKeyToRelKey(free_cell_key, rel_cell_key);
        }
        return false;
    }

    Query *R2::queueQuery(const QueryType &qtype, Unit *const &u_src, Unit *const &u_tgt, const float_t &f)
    {
        assert(u_src->isS() || u_src->isE());
        if (qtype == QueryType::Trace)
            assert(u_tgt->isPS() || u_tgt->isTR());
        else if (qtype == QueryType::Cast)
            assert(u_tgt->isT() || u_tgt->isPS() || u_tgt->isTD()); // cannot be TR

        // make sure u_src and u_tgt point to each other
        assert(u_src->hasTgt(u_tgt));
        assert(u_tgt->hasSrc(u_src));

        // make sure no duplicate queues exist
        for (Query *const &q : u_src->queries)
        {
            if (q->u_tgt == u_tgt)
            {
                _dbg11("[DUPLICATE QUEUE] A query already exists for the same pair of units { " << q << " }"); // occurs from td/ps from sy in enc.
                return q;
            }
        }

        // Queue it
        Query *q = queries.emplace(qtype, f, u_src, u_tgt);
        linkQuery(q); // link to u_src
        open_list.queue(q);
        _dbg11("<<<<<< [QUEUE] { " << q << " }");

        return q;
    }

    void R2::unqueueQuery(Query *const &q)
    {
        open_list.unqueue(q);
        assert(q->next_open == nullptr && q->prev_open == nullptr);
    }

    void R2::linkQuery(Query *const &q) { q->u_src->queries.push_back(q); }

    void R2::unlinkQuery(Query *const &q)
    {
        q->u_src->removeQuery(q);
        assert(q->u_tgt->hasQuery(q) == false);
        q->u_src = nullptr;
        q->u_tgt = nullptr;
    }

    void R2::eraseQuery(Query *const &q)
    {
        assert(open_list.empty() == true || (q->prev_open == nullptr && q->next_open == nullptr && open_list.empty() == false)); // must be removed from open_list
        assert(q->u_src == nullptr && q->u_tgt == nullptr);
        // must also be unlinked from its unit_src.
        queries.erase(q);
    }

    void R2::eraseUnitTree(const TreeDir &tdir, Unit *const &u_from, const bool &erase_query)
    {
        if (u_from->numRels(!tdir) == 0)
        {
            _dbginc;
            for (Unit *const &u_rel : u_from->rels(tdir))
            { // remove rel and recurse
                u_rel->removeRel(!tdir, u_from);
                eraseUnitTree(tdir, u_rel, erase_query);
            }
            _dbgdec;

            _dbgtitle("[ErsUTre:" << tdir << "]: Erase UFrom{ " << u_from->repr(0) << " }");
            _dbginc;

            if (erase_query == true)
            {
                if (tdir == TreeDir::Tgt)
                {
                    for (Query *const &q : u_from->queries)
                    {
                        _dbg11("[ErsUTre:" << tdir << "]: Erase Query (" << get_addr(q) << ")"); // note q->tgt may be deleted.
                        unqueueQuery(q);
                        // unlinkQuery(q);// cannot unlink here due to vector invalidation
                        q->u_src = nullptr;
                        q->u_tgt = nullptr;
                        eraseQuery(q);
                    }
                }
                else
                { // Src
                    if (u_from->srcs.empty() == false)
                    {
                        auto &_queries = u_from->src()->queries;
                        for (auto it_q = _queries.begin(); it_q != _queries.end();)
                        {
                            Query *const &q = *it_q;
                            if (q->u_tgt == u_from)
                            {
                                _dbg11("[ErsUTre:" << tdir << "]: Erase Query (" << get_addr(q) << ")"); // note q->tgt may be deleted.
                                unqueueQuery(q);
                                // unlinkQuery(q);// cannot unlink here due to vector invalidation
                                q->u_src = nullptr;
                                q->u_tgt = nullptr;
                                eraseQuery(q);
                                it_q = _queries.erase(it_q);
                            }
                            else
                                ++it_q;
                        }
                    }
                }
            }
            u_from->erase(EraseState::Del);
            _dbgdec;
        }
    }

    Unit *R2::toSrc(const UnitType &utype, const TreeDir &tdir, Unit *const &u_from, Unit *const &u_rel,
                    Ray *const &ray_left, Ray *const &ray_right, const float_t &cost_src)
    {
        _dbgtitle("[toSrc] From OldU{ " << u_from << " }");
        _dbginc;

        if (u_rel != nullptr)
        {
            assert(u_rel->hasRel(!tdir, u_from));
            assert(u_from->hasRel(tdir, u_rel));
        }

        const size_t num_rels = u_rel == nullptr ? 0 : 1;
        Unit *u_ret;

        if (u_from->numRels(tdir) > num_rels)
        {
            u_ret = u_from->unzipToSrc(utype, {}, {}, ray_left, ray_right, cost_src);
            if (u_rel != nullptr)
                u_rel->replaceRelDeep(!tdir, u_from, u_ret);
            for (Unit *const &u_rev : u_from->rels(!tdir))
                u_rev->linkRel(tdir, u_ret);

            _dbg11("[toSrc] Unzipped (>" << num_rels << tdir << ") to U{" << u_ret << "}");
            _dbg11("[toSrc]                OldU{" << u_ret << "}");
        }
        else
        {
            u_from->convToSrcType(utype, cost_src, ray_left, ray_right);
            u_ret = u_from;

            _dbg11("[toSrc] Converted OldU{" << u_ret << "}");
        }

        assert((u_rel == nullptr && u_ret->numRels(tdir) == 0) || (u_rel != nullptr && u_ret->rel(tdir) == u_rel));
        assert((u_rel == nullptr) || (u_rel != nullptr && u_rel->hasRel(!tdir, u_ret) == true));
        _dbgdec;
        return u_ret;
    }

    Unit *R2::toTgt(const UnitType &utype, const TreeDir &tdir, Unit *const &u_from, Unit *const &u_rel, Ray *const &ray_left, Ray *const &ray_right, const float_t &cost_tgt)
    {
        _dbgtitle("[toTgt] From OldU{ " << u_from << " }");
        _dbginc;

        if (u_rel != nullptr)
        {
            assert(u_rel->hasRel(!tdir, u_from));
            assert(u_from->hasRel(tdir, u_rel));
        }

        const size_t num_rels = u_rel == nullptr ? 0 : 1;
        Unit *u_ret;
        if (u_from->numRels(tdir) > num_rels)
        {
            u_ret = u_from->unzipToTgt(utype, {}, {}, ray_left, ray_right, cost_tgt);
            if (u_rel != nullptr)
                u_rel->replaceRelDeep(!tdir, u_from, u_ret);
            for (Unit *const &u_rev : u_from->rels(!tdir))
                u_rev->linkRel(tdir, u_ret);
            _dbg11("[toTgt] Unzipped (>" << num_rels << tdir << ") to U{" << u_ret << "}");
            _dbg11("[toTgt]                OldU{" << u_ret << "}");
        }
        else
        {
            u_from->convToTgtType(utype, cost_tgt);
            u_ret = u_from;

            _dbg11("[toTgt] Converted OldU{" << u_ret << "}");
        }

        assert((u_rel == nullptr && u_ret->numRels(tdir) == 0) || (u_rel != nullptr && u_ret->rel(tdir) == u_rel));
        assert((u_rel == nullptr) || (u_rel != nullptr && u_rel->hasRel(!tdir, u_ret) == true));
        _dbgdec;
        return u_ret;
    }

    void R2::_convToTgtTree(Unit *const &u_from)
    {
        _dbginc; // 1
        assert(u_from->hasTgt());
        assert(u_from->isSrcU());

        // =================== Recurse to Deepest =======================
        float_t cost_tgt_min = INF;
        for (Unit *const &u_tgt : u_from->tgts)
        {
            // ----------------- Ignore nullptr Tgts (added for enc tree) -----------------
            if (u_tgt == nullptr)
                continue;

            assert(u_tgt->isSrcY() == false);
            // ----------------- Recurse and convert to Tgt -----------------
            if (u_tgt->isSrcU())
                _convToTgtTree(u_tgt); // recurse

            assert(u_tgt->isT() || u_tgt->isPS() || u_tgt->isTD() || u_tgt->isTR());
            float_t cost_tgt = u_tgt->cost_tgt + norm(u_tgt->coord(), u_from->coord());
            if (cost_tgt < cost_tgt_min)
                cost_tgt_min = cost_tgt;
        }

        // ----------------- Remove Queries -----------------
        for (Query *const &q : u_from->queries)
        {
            _dbg11("[_CTT] Remove Query{ " << q << " }");
            unqueueQuery(q);
            q->u_src = nullptr; // to avoid invalidation in unlinkQuery (uses remove that may invalidate queries vector)
            q->u_tgt = nullptr;
            eraseQuery(q);
        }
        u_from->queries.clear();

        // =================== Convert to Tgt Type =======================
        u_from->convToTgtType(UnitType::TU, cost_tgt_min);
        _dbg11("[_CTT] UFrom { " << u_from << " }");

        _dbgdec; // 1
    }

    void R2::convTracerEncTrees()
    {
        // TODO: examine case for only one nullptr

        if (units_enc.empty() == false)
        {
            _dbgtitleheavy("[CTT] ConvTracerEncTrees");
            _dbginc; // 1

            for (Unit *const &u_to_conv : units_enc)
            {
                _dbgtitle("[CTT] U2Conv{ " << u_to_conv << " }");
                _dbginc; // 2

                // =========== Skip Units that have a common src and that has its tgt tree converted ==============
                Unit *u = u_to_conv;

                // remove nullptr Tgt
                u->removeTgt(nullptr);

                // delete u if was marked for deletion.
                if (u->numTgts() == 0)
                {
                    _dbg11("[CTT] U2Conv was prepared for Deletion. Delete");
                    eraseUnitTree(TreeDir::Src, u, true); // TODO: true if enctree before and queued?
                    _dbgdec;                              // 2
                    continue;
                }
                assert(u->isTU() || u->isPS() || u->isSrcU()); // EU added for aggressive casting of E units

                // skip converted units
                // Eu added for aggressive casting from earlier Ey, when Eu is castable to tgt
                // Ps/Tu is added by enc tracer, Src is always S(), and not E(), any T(), TD(), TR() or PS().
                // Su are other units in the node (when Ps/Tu)
                bool not_converted = u->isSrcU() ||
                                     ((u->isTU() || u->isPS()) && u->src()->isS());
                if (not_converted == false)
                {
                    _dbg11("[CTT] Skip conv. U2Conv");
                    _dbgdec; // 2
                    continue;
                }
                assert(not_converted == true);

                // =========== Get Earliest Sy ==============
                while ((u->src()->isSrcY()) == false) // cannot have E bcos trace does not care about E encounters
                    u = u->src();

                assert(u->src()->isSrcY());
                if (u->isSrcU()) // cannot have an Eu/Ey after Sy when following this branch.
                    _convToTgtTree(u);

                // =========== Queue Query ==============
                assert(u->isPS() || u->isTU()); // after conversion, or if _convToTgtTree is even run at all

                // queue only the nodes in this query
                float_t f = u->src()->cost_src + u->cost_tgt + norm(u->src()->coord(), u->coord());
                assert(f < INF);

                _dbg11("[CTT] Queue Sy USrc{ " << u->src() << " }");
                _dbg11("[CTT] Queue    UTgt{ " << u << " }");
                queueQuery(QueryType::Cast, u->src(), u, f);

                _dbgdec; // 2
            }

            units_enc.clear();
            _dbgdec;
        }
    }

    bool R2::mergeRay(const Side &side_to_merge, Unit *const &u_to_merge, Ray *const &ray_new)
    {
        return mergeRay(side_to_merge, ray_new, u_to_merge->ray_left, u_to_merge->ray_right);
    }

    bool R2::mergeRay(const Side &side_to_merge, Ray *const &ray_new, Ray *&ray_left, Ray *&ray_right)
    {
        Ray *&ray_to_merge = side_to_merge == Side::L ? ray_left : ray_right;
        if (ray_to_merge == nullptr || isLeftOrRight<false>(side_to_merge, ray_to_merge->vec, ray_new->vec))
        {
            ray_to_merge = ray_new;
            return true;
        }
        return false;
    }
}