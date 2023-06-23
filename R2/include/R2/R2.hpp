#include "P2D/P2D.hpp"
#include "types.hpp"
#include "corner.hpp"
#include "node.hpp"
#include "openlist.hpp"
#include "ray.hpp"
#include "query.hpp"
#include "traceinfo.hpp"
#include "tracestatus.hpp"

#include <memory>
#include <unordered_set>
#include <deque>
#include <vector>
#include <list>
#include <assert.h>
#include <iterator>

#pragma once
namespace P2D::R2
{
    class R2
    {

    private:
        enum BeyondRayType
        {
            NotBeyondRay,
            BeyondRayRefoundTrace,
            BeyondRayHiddenTrace
        };
        std::vector<Unit *> units_enc;
        Grid *const grid;

        LosAdvanced los;
        QueriesContainer queries;
        OpenList open_list;
        Rays rays;
        Nodes nodes;
        Corners corners;
        Path path;
        Corner *crn_start, *crn_goal, *crn_reflect;
        Node *node_start, *node_goal;
        Ray *ray_reflect;
        static Corner null_crn;
        static const int NUM_POINTS_TO_QUEUE = 5;
        const bool R2E = true;

        // ===================== R2.cpp =======================
        std::pair<Corner *, bool> tryEmplaceCrn(const Corner &corner);
        std::pair<Node *, bool> tryEmplaceNode(Corner *const &crn);
        std::pair<Ray *, bool> tryEmplaceRay(const TreeDir &tdir, Corner *const &crn_first, Corner *const &crn_second);
        // Ray must have already been created
        Ray *getRay(const TreeDir &tdir, Corner *const &crn_first, Corner *const &crn_second);
        void initialise();
        void initialiseMinimal(const V2 &start_coord, const V2 &goal_coord);
        void terminateMinimal();
        void terminate();

        Corner *traceCache(Corner *const &crn_cur, const Side &side_traced);
        Corner *traceCache(Corner *const &crn_cur, const Side &side_traced, dir_idx_t &di_trace);
        bool trace(Corner &crn, const Side &side_traced, dir_idx_t &di_trace);
        // qtype can be inferred from u_src (if only Trace or Cast types)
        // creates a query, queues it, and returns the queued query
        Query *queueQuery(const QueryType &qtype, Unit *const &u_src, Unit *const &u_tgt, const float_t &f);
        // pushes query into u_src of q
        void linkQuery(Query *const &q);
        // removes unit ptrs from query, and removes query from u_src
        void unlinkQuery(Query *const &q);
        void unqueueQuery(Query *const &q);
        void eraseQuery(Query *const &q);
        void eraseUnitTree(const TreeDir &tdir, Unit *const &u_from, const bool &erase_query);
        // when u_rel is nullptr, checks u_from if it has >0 rel(tdir), returning a new_u_from as a Src utype. If u_from has no rels, returns the converted u_from (as a Src utype)
        // When u_rel is not nullptr, checks u_from if it has >1 rel(tdir) and returns a new_u_from as a Src utype. If u_from has only one rel that is u_rel, returns the converted u_from (as a Src utype).
        // If unzipped, links new_u_from to u_rel (must be a rel(tdir) of u_from, if not nullptr), unlinks u_from from u_rel (if not nullptr), and only writes crn_left, ray_right and cost_src into new_u_from.
        // If not unzipped, converts u_from into a src type and writes crn_left, ray_right and cost_src into u_from.
        Unit *toSrc(const UnitType &utype, const TreeDir &tdir, Unit *const &u_from, Unit *const &u_rel, Ray *const &ray_left, Ray *const &ray_right, const float_t &cost_src);
        // when u_rel is nullptr, checks u_from if it has >0 rel(tdir), returning a new_u_from as a Tgt utype. If u_from has no rels, returns the converted u_from (as a Tgt utype)
        // When u_rel is not nullptr, checks u_from if it has >1 rel(tdir) and returns a new_u_from as a Tgt utype. If u_from has only one rel that is u_rel, returns the converted u_from (as a Tgt utype).
        // If unzipped, links new_u_from to u_rel (must be a rel(tdir) of u_from, if not nullptr), unlinks u_from from u_rel (if not nullptr), and only writes cost_tgt into new_u_from.
        // If not unzipped, converts u_from into a tgt type and writes cost_src into u_from.
        Unit *toTgt(const UnitType &utype, const TreeDir &tdir, Unit *const &u_from, Unit *const &u_rel, Ray *const &ray_left, Ray *const &ray_right, const float_t &cost_tgt);

        void _convToTgtTree(Unit *const &u_from);
        void convTracerEncTrees();

        static bool mergeRay(const Side &side_to_merge, Unit *const &u_to_merge, Ray *const &ray_new);
        static bool mergeRay(const Side &side_to_merge, Ray *const &ray_new, Ray *&ray_left, Ray *&ray_right);

        // ==================== Checks.cpp ==============================
        // for cv turning points, and assuming cur is progressed
        static bool angDirChanged(const Side &side_traced, const V2 &v_curFromSrc, const V2 &v_traceCur);
        // for cv turning points, and assuming there is no progression info
        static bool angDirChanged(const Side &side_traced, const V2 &v_curFromSrc, const V2 &v_traceCur, const V2 &v_tracePrev);

        // returns true if tgt is castable from cur wrt src
        static bool isOptTaut(const Side &side_rev_of_rel, const V2 &v_curFromRel, const V2 &v_relFromRRel);
        static bool isCorrectSide(const Side &side_rel_of_cur, const V2 &v_curFromRel, const V2 &v_rel_edge_of_cur);
        static bool isCastable(const Side &side_traced, const V2 &v_curFromSrc, const V2 &v_curFromTgt, const V2 &v_traceCur);
        static bool isPrunable(const Side &side_rev_of_rel, const V2 &v_curFromRel, const V2 &v_relFromRRel);
        // static bool isPrunable(const bool rel_is_psd, const Side &side_revOfRel, Corner *const &crn_rel, const V2 &v_curFromRel, const V2 &v_relFromRRel);
        inline bool isStart(const Unit *const &u) { return isStart(u->node); }
        inline bool isStart(const Node *const &node) { return node == this->node_start; }
        inline bool isGoal(const Unit *const &u) { return isGoal(u->node); }
        inline bool isGoal(const Node *const &node) { return node == this->node_goal; }
        bool isStartOrGoal(const TreeDir &tdir, const Unit *const &u) { return isStartOrGoal(tdir, u->node); }
        bool isStartOrGoal(const TreeDir &tdir, const Node *const &node) { return (tdir == Src && isStart(node)) || (tdir == Tgt && isGoal(node)); }
        bool inEdge(const Corner *const &crn, V2 v_testFromCrn);
        bool isCheckerboardNonconvex(const Corner *const &crn) const
        {
            uint8_t window = 0;
            for (dir_idx_t di = 1; di < 8; di += 2)
            {
                mapkey_t cell_key = grid->addKeyToRelKey(crn->mkey, grid->getCellRelKey(di, crn->coord.x));
                V2 cell_coord = crn->coord + grid->getCellRelCoord(di);
                window <<= 1;
                window |= grid->isAccessible(cell_key, cell_coord);
            }

            return (window == 0b1010 || window == 0b0101);

            // int num_access = 0;
            // for (dir_idx_t di = 1; di < 8; di += 2)
            // {
            //     mapkey_t mkey = grid->addKeyToRelKey(crn->mkey, grid->getAdjRelKeyCFV(di));
            //     window <<= 1;
            //     bool isAcc = grid->isAccessible(mkey);
            //     num_access += isAcc;
            //     window |= isAcc;
            // }

            // return num_access <= 1 || (window == 0b1010 || window == 0b0101);
        }

        // ================================== caster.cpp ================================
        bool caster(Query *const &query);
        bool caster(Unit *const &old_u_src, Unit *const &old_u_tgt);
        void _casterReachedFromSUorEU(Ray *const &ray, Unit *const &old_u_src, Unit *const &old_u_tgt);
        bool _casterReachedFromSYorEY(Ray *ray, Unit *old_u_src, Unit *old_u_tgt);
        void _casterCollided(Ray *const &brn, Unit *const &old_u_src, Unit *const &old_u_tgt);
        // returns true if trace from proj is beyond ray and refound trace
        // bool _casterProj(Ray *const &ray, Unit *const &old_u_src, Unit *const &old_u_tgt);
        void _casterGetPath(Unit *const &u_src, Unit *const &u_tgt);
        bool _casterConvTree(Unit *const u_from);
        void _casterConvTrees(Unit *u_tgt_cheapest);
        // if ssrc-src-tgt are colinear, prunes src(old_u_src) and sets it to ssrc(old_u_src->src), then replaces ray from (src->tgt) to (ssrc->tgt).
        bool _casterPruneSrc(Unit *&old_u_src, Unit *const &old_u_tgt, Ray *&ray);

        bool casterStart();
        void findCollisionCrns(Ray *const &ray, const LosResult &res);
        Ray *project(Corner *const &crn_src, Corner *const &crn_tgt);
        void project(Ray *const &ray);
        template <bool is_start = false>
        Ray *cast(Corner *const &crn_src, Corner *const &crn_tgt);

        //  ================================== tracer.cpp ================================
        void tracerPolled(Query *const &query);
        // returns true if terminated when beyond ray and refound trace.
        bool tracer(TraceStatus &status);

        void _tracerErase(TraceInfoContainer &ti_srcs, TraceInfoContainer &ti_tgts);
        // erases ti_rel from ti_rels and then replaces ti_rel with the next ti in the container.
        void _tracerErase(const TreeDir &tdir, TraceInfo *&ti_rel, TraceInfoContainer &ti_rels);
        void _tracerErase(TraceStatus &status);

        // u_tgts cannot contain u_src
        void _tracerUnstageTgts(Unit *const &u_src, std::vector<Unit *> &u_tgts);
        void _tracerStageUnitTree(Unit *const &u_src, const std::vector<Unit *> &u_tgts);

        // returns true if tracer did not progress past ray_prog. Updates the ray_prog.
        // relTdir (if ti_rel is src, relTdir is src). traceTdir(normal traces are tgt. Src traces are spawned when cur is in tgt edge.)
        bool _tracerNotProgressed(const bool &traceTdir_is_relTdir, TraceInfo *const &ti, TraceStatus &status);

        // erases ti_rel from ti_rels and replaces ti_rel with next ti_rel if refound crn_cur.
        bool _tracerRefoundRel(const TreeDir &tdir, const Corner *const &crn_cur, TraceInfo *&ti_rel, TraceInfoContainer &ti_rels);

        // returns true if beyond src ray of u_src. u_src must be SVY
        BeyondRayType _tracerCheckRay(const Side &side_traced, const Corner *const &crn_cur, Ray *const &ray, const V2 &v_curFromSrc);
        bool _tracerProcessSrc(TraceStatus &status);
        // returns Fail if trace is beyond ray and cannot continue. Success if src is SU and trace is hidden behind projection. Normal otherwise (beyond ray of prunable SY, or not beyond ray).
        bool _tracerRaySrc(TraceInfo *&ti_src, TraceStatus &status);
        // returns Fail if refound trace. Success if pruned. Normal if taut.
        bool _tracerPruneSrc(TraceInfo *&ti_src, TraceStatus &status);
        // returns Success if in edge (stop trace). Normal if not in edge
        bool _tracerEdgeSrc(TraceInfo *&ti_src);
        bool _tracerProcessAbnormal(TraceStatus &status);
        // returns Success if all tgts are castable and queued, or search tree is encountered. Normal if there are tgts remaining that cannnot be casted.
        bool _tracerPointSrc(TraceInfo *const &ti_src, TraceStatus &status);
        void _tracerAddEncUnits(Unit *const &u_cur);
        bool _tracerEncounteredTree(TraceStatus &status);
        bool _tracerQueueSrc(TraceInfo *const &ti_cur, TraceStatus &status);

        bool _tracerProcessTgt(TraceStatus &status);
        // returns Success if ti_tgt is erased as it has been pruned. Normal if not. Updates ti_tgt with next ti_tgt if pruned.
        bool _tracerPruneTgt(TraceInfo *&ti_tgt, TraceStatus &status);
        bool _tracerQueueTgt(TraceStatus &status);
        bool _tracerPointPs(TraceStatus &status);
        bool _tracerPointTu(TraceStatus &status);

        Unit *_tracerMakeTRorPS(const UnitType &utype, const bool &unzip_tgt, const Side &side_traced, Corner *const &crn_cur, const TraceInfoContainer &ti_tgts);
        Unit *_tracerMakeSrc(const UnitType &utype, const Side &side_tgt, Corner *const &crn_cur, Unit *const &u_src);

        // assumes crn_cur, side and di are updated
        void _tracerUpdate(TraceStatus &status);
        void _tracerTrace(TraceStatus &status);

    public:
        R2(Grid *const &grid, const bool &R2E) : grid(grid), los(grid), R2E(R2E) {}
        Path run(const V2 start_coord, const V2 goal_coord)
        {
            _dbgreset;
            _dbg11("R2 from (" << start_coord << ") to (" << goal_coord << ")");

            if (start_coord == goal_coord)
            {
                _dbg11("Start is Goal. Path found.");
                path = {goal_coord, start_coord};
                return path;
            }

            initialiseMinimal(start_coord, goal_coord);
            if (isCheckerboardNonconvex(crn_start))
            {
                std::cout << "Ignore start on checkerboard ncv" << std::endl;
                path = {V2(0, 0), V2(0, 0)};
                terminateMinimal();
                return path;
            }
            initialise();
            // =========== create new query with start and goal =======================
            if (casterStart() == false)
            {
                while (1)
                {
                    Query *q = open_list.poll();
                    if (q == nullptr)
                        break; // no more queries

                    _dbg11(">>>>>> [POLL] >>>>>> { " << q << " }");
                    if (q->type == QueryType::Trace)
                        tracerPolled(q);
                    else
                    {
                        assert(q->type == QueryType::Cast);
                        if (caster(q))
                            break;
                    }
                }
                delete crn_reflect;
                delete ray_reflect;
            }

            if (path.empty())
            {
                _dbg11("No path found.");
                // if (crn_start->di_occ != 0)
                // {
                //     std::cout << "Ignore start on checkerboard ncv" << std::endl;
                //     path = {V2(0, 0), V2(0, 0)};
                // }
            }

            // TODO: delete branch between start and goal
            terminate();
            terminateMinimal(); // place after terminate so start and goal crns don't become ncv, which can then induce a node deletion when batches in queries are deleted.

            return path;
        }
    };
}
