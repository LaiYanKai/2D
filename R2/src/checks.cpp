#include "R2/R2.hpp"
#include "R2/types.hpp"

namespace P2D::R2
{
    // for cv turning points
    bool R2::angDirChanged(const Side &side_traced, const V2 &v_curFromSrc, const V2 &v_traceCur)
    {
        if (side_traced == Side::L)
            return isLeft<true>(v_curFromSrc, v_traceCur).second;
        else
            return isLeft<true>(v_traceCur, v_curFromSrc).second;
    }

    bool R2::angDirChanged(const Side &side_traced, const V2 &v_curFromSrc, const V2 &v_traceCur, const V2 &v_tracePrev)
    {
        if (side_traced == Side::L)
            return isLeft(v_tracePrev, v_curFromSrc).second && isLeft<true>(v_curFromSrc, v_traceCur).second;
        else
            return isLeft<true>(v_traceCur, v_curFromSrc).second && isLeft(v_curFromSrc, v_tracePrev).second;
    }

    // if any negatives are used, just swap the parameters (2d cross product/ det)
    // side_traced, -v_curFromTgt, v_curFromSrc == side_traced, v_curFromSrc, v_curFromTgt
    bool R2::isOptTaut(const Side &side_rev_of_rel, const V2 &v_curFromRel, const V2 &v_relFromRRel)
    {
        return isLeftOrRight<false>(side_rev_of_rel, v_relFromRRel, v_curFromRel);
    }

    // if any negatives are used, just swap the parameters (2d cross product/ det)
    bool R2::isCorrectSide(const Side &side_rel_of_cur, const V2 &v_curFromRel, const V2 &v_rel_edge_of_cur)
    {
        return isLeftOrRight<false>(side_rel_of_cur, v_rel_edge_of_cur, v_curFromRel);
    }

    // to remove v_curFromSrc
    bool R2::isCastable(const Side &side_traced, [[maybe_unused]] const V2 &v_curFromSrc, const V2 &v_curFromTgt, const V2 &v_traceCur)
    {
        return // isOptTaut(side_traced, v_curFromSrc, v_curFromTgt) && // is actually (side_traced, v_curFromSrc, -v_curFromTgt)
            isCorrectSide(side_traced, v_curFromTgt, v_traceCur);
        // isLeftOrRight<false>(side_traced, v_curFromTgt, v_curFromSrc) &&
        // isLeftOrRight<false>(side_traced, v_traceCur, v_curFromTgt); // to guard against casting to pseudo target immediately before trace

        // isLeftOrRight<false>(side_traced, v_curFromSrc, v_traceCur) && // not neccessary bcos castable checks are done after ang dir change and ray prog checks
    }

    bool R2::isPrunable(const Side &side_revOfRel, const V2 &v_curFromRel, const V2 &v_relFromRRel)
    {
        // pseudo's tgt and src directions are flipped. (future points that take current trace side, which is reversed)

        // opt taut (same for both cvx (svu / tvu) and ncv (psd))
        // false to avoid beyond ray issues (edge trace casts from src, cast // to prev edge, src is directly behind edge)

        // also, avoid pruning point B if C is colinear and between A-B.
        // always prune if vcfr == 0
        auto p = dtrLeftOrRight<false>(side_revOfRel, v_curFromRel, v_relFromRRel); // not opt taut
        if (p.first == 0)
            return dot(v_curFromRel, v_relFromRRel) >= 0; // B prunable if either is zero (A on B, or B on C), or if B is coliear and lies between A-C.
        else
            return p.second;
    }

    bool R2::inEdge(const Corner *const &crn, V2 v_testFromCrn)
    {
        dir_idx_t di = dirToDirIdx(v_testFromCrn);
        if (crn->is_convex == true)
            return crn->di_occ == di; // ray is pointing into convex corner
        else
            return addDirIdx<true>(crn->di_occ, 3) != di && // ray is pointing into ncv corner
                   addDirIdx<true>(crn->di_occ, 4) != di &&
                   addDirIdx<true>(crn->di_occ, 5) != di;
    }

    /*
    bool R2::isPrunable(const bool rel_is_psd, const Side &side_revOfRel, Corner *const &crn_rel, const V2 &v_curFromRel, const V2 &v_relFromRRel)
    {
        const V2 v_relEdgeRel = crn_rel->edgeVec(rel_is_psd == true ? side_revOfRel : !side_revOfRel);
        // pseudo's tgt and src directions are flipped. (future points that take current trace side, which is reversed)

        // opt taut (same for both cvx (svu / tvu) and ncv (psd))
        return (isLeftOrRight<false>(side_revOfRel, v_relFromRRel, v_curFromRel) == false ||
                isLeftOrRight<true>(side_revOfRel, v_curFromRel, v_relEdgeRel) == false);
    }

    bool R2::isPrunable(const Side &side_rev_of_rel, const V2 &v_curFromRel, const V2 &v_relFromRRel)
    {
        assert(v_relFromRRel != 0);
        long_t dtr = det(v_curFromRel, v_relFromRRel);
        if (dtr == 0)
            return dot(v_curFromRel, v_relFromRRel) <= 0;

        assert(dtr != 0);
        if (side_rev_of_rel == Side::L)
            return dtr < 0;
        else
            return dtr > 0;

        // if (include_parallel == false) return isLeftOrRight<true>(side_rev_of_rel, v_curFromRel, v_relFromRRel);
        // else return isLeftOrRight<false>(side_rev_of_rel, v_curFromRel, v_relFromRRel);
    } */

}
