#include <ostream>
#include "types.hpp"
#include "math.hpp"
#include "dir_idx.hpp"
#include "Vec2.hpp"
#include "grid.hpp"

#pragma once
namespace P2D
{
    struct LosResult
    {
        V2 coord_left = {-1, -1}, coord_right = {-1, -1};
        mapkey_t key_left = 0, key_right = 0;
        dir_idx_t di_left = 0, di_right = 0;
        bool collided = false;

        friend std::ostream &operator<<(std::ostream &out, const LosResult &res)
        {
            if (res.collided)
            {
                out << "Collided: ";
                out << "L(" << scast(res.di_left) << "|" << res.coord_left << ")";
                out << ", R(" << scast(res.di_right) << "|" << res.coord_right << ")";
            }
            else
                out << "Not collided";
            return out;
        }
    };

    // Advanced LOS that returns collided vertices and direction of next trace from collision point.
    // diag_block is specified for non-inverted case. If diag_block == false, no diag_block occurs when passing over free cells (inverted == false), but diag_block occurs when passing over occupied cells (inverted == true)
    class LosAdvanced
    {
    private:
        Grid *const grid;

        inline LosResult getNotCollidedResult() const { return LosResult(); }
        inline LosResult getCollidedResult(const V2 &coord_left, const V2 &coord_right, const dir_idx_t &di_left, const dir_idx_t &di_right) const
        {
            LosResult res;
            res.coord_left = coord_left;
            res.coord_right = coord_right;
            res.di_left = di_left;
            res.di_right = di_right;
            res.key_left = grid->coordToKey<false>(res.coord_left);
            res.key_right = grid->coordToKey<false>(res.coord_right);
            res.collided = true;
            return res;
        }

        // assumes that mapkey_t is not out of map
        template <bool invert>
        inline bool isObstructed(const mapkey_t &cell_key) const
        {
            if constexpr (invert == true)
                return !grid->isOc(cell_key);
            else
                return grid->isOc(cell_key);
        }

        template <bool invert, bool cast, bool diag_block, bool diag_block_at_start>
        inline LosResult searchCardinal(const V2 &src_coord, const V2 &tgt_coord, const V2 &dir) const
        {
            dir_idx_t di_front = dirToDirIdx(dir);
            assert(isCardinal(di_front) == true);
            const bool dim_l = (di_front == 0 || di_front == 4);
            const int_t &last_l = cast == true ? tgt_coord[dim_l] : grid->getBoundary<false>(di_front);
            V2 coord = cast == true ? src_coord : tgt_coord;
            int_t &l = coord[dim_l];

            if (l == last_l) // for bozos who start casting at the edge of the map, or src_coord == tgt_coord;
                return getNotCollidedResult();

            const V2 sgn_dir = P2D::sgn(dir);
            const V2 abs_dir = P2D::abs(dir);
            const mapkey_t rel_cell_key = grid->getRelKey<true>(di_front);
            const int_t &sgn_l = sgn_dir[dim_l];
            const int_t &s = coord[!dim_l];
            const bool left_in_map = s != grid->getBoundary<false>(addDirIdx(di_front, 2));
            const bool right_in_map = s != grid->getBoundary<false>(addDirIdx(di_front, 6));
            assert(left_in_map == true || right_in_map == true);
            char window = 0;
            mapkey_t flcell_key, frcell_key;

            // --------------- Get the first cell keys, and check the first coord for diagonal blocking, if required ---------------
            if constexpr (diag_block == true && diag_block_at_start == true)
            {
                if (l != grid->getBoundary<false>(addDirIdx(di_front, 4)))
                { // make sure cells behind starting coord is in map
                    V2 cell_coord = coord + grid->getCellRelCoord(addDirIdx(di_front, 3));
                    flcell_key = grid->coordToKey<true>(cell_coord); // is blcell_key atm
                    window = (isObstructed<invert>(flcell_key) << 3);
                    cell_coord = coord + grid->getCellRelCoord(addDirIdx(di_front, 5));
                    frcell_key = grid->coordToKey<true>(cell_coord); // is brcell_key atm
                    window |= (isObstructed<invert>(frcell_key) << 2);
                }
                else
                    window = 0b1100;
                flcell_key = grid->addKeyToRelKey(flcell_key, rel_cell_key);
                frcell_key = grid->addKeyToRelKey(frcell_key, rel_cell_key);
            }
            else
            {
                V2 cell_coord = coord + grid->getCellRelCoord(addDirIdx(di_front, 1));
                flcell_key = grid->coordToKey<true>(cell_coord);
                cell_coord = coord + grid->getCellRelCoord(addDirIdx(di_front, 7));
                frcell_key = grid->coordToKey<true>(cell_coord);
            }

            // ------------------- Loop until front blocked or reached / out_of_map
            do
            {
                // ------------ get window -----------------
                window |= (left_in_map == false || isObstructed<invert>(flcell_key)) << 1;
                window |= (right_in_map == false || isObstructed<invert>(frcell_key));

                // ------ Check if front is blocked ----------
                bool front_blocked;
                if constexpr (diag_block)
                    front_blocked = window == 0b1001 || window == 0b1010 || (window & 0b11) == 0b11;
                else
                    front_blocked = window == 0b11;

                if (front_blocked == true)
                    return getCollidedResult(coord, coord, addDirIdx(di_front, 2), addDirIdx(di_front, 6));

                // not blocked

                // ---------- Increment -------------
                l += sgn_l;
                if (l == last_l)
                    break;
                ;
                if (left_in_map == true)
                    flcell_key = grid->addKeyToRelKey(flcell_key, rel_cell_key);
                if (right_in_map == true)
                    frcell_key = grid->addKeyToRelKey(frcell_key, rel_cell_key);

                window <<= 2;
                window &= (diag_block ? 0b1111 : 0b11);
            } while (1);

            // not collided (reached / out of map)
            return getNotCollidedResult();
        }

        // returns true if the front is blocked, or diagonally blocked.
        template <bool invert, bool diag_block_at_start, bool diag_block, bool first_check>
        inline bool _ordinalObstructedFront(const dir_idx_t &di_long, const dir_idx_t &di_short, const dir_idx_t &di_front, const bool &dim_l,
                                            const mapkey_t &cell_key, const mapkey_t &rel_nlcell_key, const mapkey_t &rel_nscell_key,
                                            const int_t &err_s, [[maybe_unused]] const V2 &src_coord) const
        {
            const bool front_ob = isObstructed<invert>(cell_key);
            if (front_ob == false)
            {
                bool check_diag_block;
                if constexpr (diag_block_at_start == true && first_check == true)
                {
                    assert(err_s == 0);
                    bool nl_in_map = grid->getBoundary<false>(addDirIdx(di_long, 4)) != src_coord[dim_l];
                    bool ns_in_map = grid->getBoundary<false>(addDirIdx(di_short, 4)) != src_coord[!dim_l];
                    mapkey_t bcell_key = grid->addKeyToRelKey(cell_key, grid->getRelKey<true>(addDirIdx(di_front, 4)));
                    check_diag_block = nl_in_map && ns_in_map && isObstructed<invert>(bcell_key) == false; // diagonal block requires checkerboard corners
                }
                else
                    check_diag_block = diag_block;

                // check diagonal blocking, if required
                if (check_diag_block == true && err_s == 0)
                {
                    mapkey_t nlcell_key = grid->addKeyToRelKey(cell_key, rel_nlcell_key); // cell in -long direction from current cell
                    mapkey_t nscell_key = grid->addKeyToRelKey(cell_key, rel_nscell_key); // cell in -short direction from current cell
                    return isObstructed<invert>(nlcell_key) == true && isObstructed<invert>(nscell_key) == true;
                }
            }
            return front_ob;
        }

        // returns the collided result
        template <bool invert>
        inline LosResult _ordinalObstructedFrontResult(const bool &is_rh_frame, const dir_idx_t &di_front, const V2 &abs_dir,
                                                       const mapkey_t &cell_key, const mapkey_t &rel_nlcell_key, const mapkey_t &rel_nscell_key, const int_t &err_s) const
        {
            V2 vert_coord = grid->keyToCoord<true>(cell_key);
            vert_coord += grid->getVertexRelCoord(addDirIdx(di_front, 4));

            dir_idx_t di_left, di_right;
            // (1) ray passes through vertex
            if (err_s == 0)
            {
                mapkey_t nlcell_key = grid->addKeyToRelKey(cell_key, rel_nlcell_key);
                mapkey_t nscell_key = grid->addKeyToRelKey(cell_key, rel_nscell_key);
                bool nl_ob = isObstructed<invert>(nlcell_key);
                bool ns_ob = isObstructed<invert>(nscell_key);

                // (1a) if nl (left) and ns (right) are blocked (nonconvex)
                if (nl_ob == true && ns_ob == true)
                {
                    if (abs_dir.x == abs_dir.y)
                    { // parallel to normal of corner
                        di_left = addDirIdx(di_front, 1);
                        di_right = addDirIdx(di_front, 7);
                    }
                    else
                    { // not parallel to normal of corner, hitting the wall on the -short side first
                        di_left = addDirIdx(di_front, is_rh_frame == true ? 1 : 3);
                        di_right = addDirIdx(di_front, is_rh_frame == true ? 5 : 7);
                    }
                }
                // (1b) if nl (left) and ns (right) are blocked,
                else if (nl_ob == true && ns_ob == false)
                {
                    di_left = addDirIdx(di_front, is_rh_frame == true ? 3 : 1);
                    di_right = addDirIdx(di_front, is_rh_frame == true ? 7 : 5);
                }
                else if (nl_ob == false && ns_ob == true)
                {
                    di_left = addDirIdx(di_front, is_rh_frame == true ? 1 : 3);
                    di_right = addDirIdx(di_front, is_rh_frame == true ? 5 : 7);
                }
                else
                { // nl_ob == false && ns_ob == false (ncv)
                    if (abs_dir.x == abs_dir.y)
                    { // parallel to normal of corner
                        di_left = addDirIdx(di_front, 3);
                        di_right = addDirIdx(di_front, 5);
                    }
                    else
                    { // not parallel to normal of corner, hitting the wall on the +long side first
                        di_left = addDirIdx(di_front, is_rh_frame == true ? 3 : 1);
                        di_right = addDirIdx(di_front, is_rh_frame == true ? 7 : 5);
                    }
                }
                return getCollidedResult(vert_coord, vert_coord, di_left, di_right);
            }
            else
            { // hit wall that points in +short direction
                V2 coord_left, coord_right;
                if (is_rh_frame == true)
                {
                    coord_right = vert_coord;
                    di_left = addDirIdx(di_front, 1);
                    di_right = addDirIdx(di_front, 5);
                    coord_left = grid->getRelCoord<false>(di_left) + vert_coord;
                }
                else
                {
                    coord_left = vert_coord;
                    di_left = addDirIdx(di_front, 3);
                    di_right = addDirIdx(di_front, 7);
                    coord_right = grid->getRelCoord<false>(di_right) + vert_coord;
                }
                return getCollidedResult(coord_left, coord_right, di_left, di_right);
            }
        }

        template <bool invert, bool cast, bool diag_block, bool diag_block_at_start>
        inline LosResult searchOrdinal(const V2 &src_coord, const mapkey_t &src_key, const V2 &tgt_coord, const mapkey_t &tgt_key, const V2 &dir) const
        {
            dir_idx_t di_front = dirToDirIdx(dir);
            assert(isOrdinal(di_front) == true);

            const V2 sgn_dir = P2D::sgn(dir);
            const V2 abs_dir = P2D::abs(dir);
            const bool dim_l = abs_dir.x < abs_dir.y;

            // note is_rh_frame has undefined behavior for 45 deg lines.
            const bool is_rh_frame = ((sgn_dir.x > 0) == (sgn_dir.y > 0)) != (dim_l); // right hand frame (long axis lies right of short axis when superimposed on cartesian space)
            const dir_idx_t di_long = addDirIdx(di_front, is_rh_frame == true ? 7 : 1);
            const dir_idx_t di_short = addDirIdx(di_front, is_rh_frame == true ? 1 : 7);

            // -------- Determine last cell key ----------
            mapkey_t cell_key, last_cell_key;
            if constexpr (cast == true)
            {
                cell_key = grid->addKeyToRelKey(src_key, grid->getCellRelKey(di_front, src_coord.x));
                last_cell_key = grid->addKeyToRelKey(tgt_key, grid->getCellRelKey(di_front, tgt_coord.x));
            }
            else
            { // find where the last vert_key is, find the difference between the last vert_key and src_hey, and get the relative cell key using the difference
                cell_key = grid->addKeyToRelKey(tgt_key, grid->getCellRelKey(di_front, tgt_coord.x));

                // last_vert_key is dependent on the point the projection exits the map
                // The difference between the long direction grid boundary and the src long coordinate, if the projection collides at the long direction's grid boundary
                const int_t diff_long_long = grid->getBoundary<false>(di_long) - src_coord[dim_l];
                // The difference between the short direction grid boundary and the src short coordinate, if the projection collides at the short direction's grid boundary
                const int_t diff_short_short = grid->getBoundary<false>(di_short) - src_coord[!dim_l];
                // The floored difference between the short direction grid boundary and the src long coordinate, if the projection collides at the short direction's grid boundary
                const int_t diff_long_short = diff_short_short * dir[dim_l] / dir[!dim_l];

                if (std::abs(diff_long_long) >= std::abs(diff_long_short))
                { // collide at the short direction's grid boundary. Last vertex is determined by diff_short_short and diff_long_short
                    last_cell_key = grid->getKey<true>(cell_key,
                                                       dim_l == 0 ? diff_long_short : diff_short_short,
                                                       dim_l == 0 ? diff_short_short : diff_long_short);
                }
                else
                {
                    const int_t diff_short_long = diff_long_long * dir[!dim_l] / dir[dim_l];
                    last_cell_key = grid->getKey<true>(cell_key,
                                                       dim_l == 0 ? diff_long_long : diff_short_long,
                                                       dim_l == 0 ? diff_short_long : diff_long_long);
                }
            }

            if (cell_key == last_cell_key) // for bozos who start casting at the edge of the map, or src_coord == tgt_coord;
                return getNotCollidedResult();

            const mapkey_t &rel_nlcell_key = grid->getRelKey<true>(addDirIdx(di_long, 4));
            const mapkey_t &rel_nscell_key = grid->getRelKey<true>(addDirIdx(di_short, 4));
            const mapkey_t &rel_lcell_key = grid->getRelKey<true>(di_long);
            const mapkey_t &rel_scell_key = grid->getRelKey<true>(di_short);
            int_t err_s = 0;
            const int_t &abs_dir_short = abs_dir[!dim_l];
            const int_t &abs_dir_long = abs_dir[dim_l];

            // ================== check the front cell and diagonal blocking before loop =========================
            if (_ordinalObstructedFront<invert, diag_block_at_start, diag_block, true>(
                    di_long, di_short, di_front, dim_l, cell_key, rel_nlcell_key, rel_nscell_key, err_s, src_coord))
                return _ordinalObstructedFrontResult<invert>(is_rh_frame, di_front, abs_dir, cell_key, rel_nlcell_key, rel_nscell_key, err_s);

            // do main while loop
            while (1)
            {
                err_s += abs_dir_short;
                // ================== Increment in short if required =====================
                if (err_s == abs_dir_long)
                {
                    cell_key = grid->addKeyToRelKey(cell_key, rel_scell_key);
                    err_s = 0;
                }
                else if (err_s > abs_dir_long)
                {
                    cell_key = grid->addKeyToRelKey(cell_key, rel_scell_key);
                    // check cell in short direction
                    err_s -= abs_dir_long;

                    // ------------ check if projection and went out of map ----------------------
                    if (cast == false && cell_key == last_cell_key) // projection and went out of map
                        return getNotCollidedResult();

                    // ------------ check if collide with wall parallel to long axis ----------------------
                    if (isObstructed<invert>(cell_key) == true)
                    {
                        dir_idx_t di_left, di_right;
                        V2 coord_left, coord_right;
                        if (is_rh_frame == true)
                        {
                            di_left = addDirIdx(di_front, 3);
                            di_right = addDirIdx(di_front, 7);
                            coord_left = grid->keyToCoord<true>(cell_key);
                            coord_left += grid->getVertexRelCoord(addDirIdx(di_front, 4));
                            coord_right = coord_left + grid->getRelCoord<false>(di_long);
                        }
                        else
                        {
                            di_left = addDirIdx(di_front, 1);
                            di_right = addDirIdx(di_front, 5);
                            coord_right = grid->keyToCoord<true>(cell_key);
                            coord_right += grid->getVertexRelCoord(addDirIdx(di_front, 4));
                            coord_left = coord_right + grid->getRelCoord<false>(di_long);
                        }
                        return getCollidedResult(coord_left, coord_right, di_left, di_right);
                    }
                }

                // ================== Increment in long =====================
                cell_key = grid->addKeyToRelKey(cell_key, rel_lcell_key);

                // ================== Return if OOM or reached target ===========================
                if (cell_key == last_cell_key)
                    return getNotCollidedResult(); // cast reached target or projection went out of map

                // ================== check the front cell and diagonal blocking before loop =========================
                if (_ordinalObstructedFront<invert, diag_block_at_start, diag_block, false>(
                        di_long, di_short, di_front, dim_l, cell_key, rel_nlcell_key, rel_nscell_key, err_s, src_coord))
                    return _ordinalObstructedFrontResult<invert>(is_rh_frame, di_front, abs_dir, cell_key, rel_nlcell_key, rel_nscell_key, err_s);
            }
        }

        template <bool invert, bool cast, bool diag_block, bool diag_block_at_start>
        inline LosResult search(const V2 &src_coord, const mapkey_t &src_key, const V2 &tgt_coord, const mapkey_t &tgt_key) const
        {
            assert(grid->coordToKey<false>(src_coord) == src_key);
            assert(grid->coordToKey<false>(tgt_coord) == tgt_key);
            assert(grid->inMap<false>(src_coord));
            assert(grid->inMap<false>(tgt_coord));

            const V2 dir = tgt_coord - src_coord;

            if (dir.x == 0 || dir.y == 0)
                return searchCardinal<invert, cast, diag_block, diag_block_at_start>(src_coord, tgt_coord, dir);
            else
                return searchOrdinal<invert, cast, diag_block, diag_block_at_start>(src_coord, src_key, tgt_coord, tgt_key, dir);
        }

    public:
        LosAdvanced(Grid *const &grid) : grid(grid) {}

        // cast from Src in the direction src->tgt to Tgt
        template <bool invert, bool diag_block, bool diag_block_at_start>
        inline LosResult cast(const V2 &src_coord, const mapkey_t &src_key, const V2 &tgt_coord, const mapkey_t &tgt_key) const
        {
            return search<invert, true, diag_block, diag_block_at_start>(src_coord, src_key, tgt_coord, tgt_key);
        }

        // project from Tgt in the direction src->tgt
        template <bool invert, bool diag_block, bool diag_block_at_start>
        inline LosResult project(const V2 &src_coord, const mapkey_t &src_key, const V2 &tgt_coord, const mapkey_t &tgt_key) const
        {
            return search<invert, false, diag_block, diag_block_at_start>(src_coord, src_key, tgt_coord, tgt_key);
        }
    };

    // template <bool is_proj, bool find_successors, bool is_blocking>
    // class LosAdvanced : public P2D::Los2::Base<false>
    // { // no is_blocking, return state; assumes path traversed is in map
    //   // using eod_t = typename std::conditional<is_proj, V2, Position>::type;

    // private:
    //     template <bool to_coord>
    //     inline void emplaceSuccessor(const dir_idx_t &rel_from_fwd_dir_idx, const dir_idx_t &fwd_dir_idx, Position &pos, std::vector<Pose> &successors)
    //     {
    //         dir_idx_t crn_dir_idx = addDirIdx(fwd_dir_idx, rel_from_fwd_dir_idx);
    //         if constexpr (to_coord)
    //             pos.coord = grid->keyToCoord<V2>(pos.key);
    //         else
    //             pos.key = grid->coordToKey<V2>(pos.coord);
    //         successors.emplace_back(crn_dir_idx, pos);
    //     }

    //     static inline void cardinalFrontBlocked(Position &pos, const dir_idx_t &l_dir_idx, const dir_idx_t &r_dir_idx, LosInfo &linfo, OcGrid<false> *const grid)
    //     {
    //         // front is blocked
    //         pos.key = grid->coordToKey<V2>(pos.coord); // toCrnKey<is_blocking>(pos.coord, 0);
    //         // fill result
    //         linfo.fill(LosState::collided,
    //                    l_dir_idx, pos,
    //                    r_dir_idx, pos);
    //     }

    //     inline void searchCardinal(const Corner &crn_src, [[maybe_unused]] const Corner &crn_tgt,
    //                                const V2 &sgn_diff, const uint8_t &longer_dim, const uint8_t &shorter_dim, LosInfo &linfo)
    //     { // assumes points and straight path betw src and tgt do not go out of map

    //         const dir_idx_t fwd_dir_idx = dirToDirIdx(sgn_diff);
    //         // track longer dim
    //         int_t longer_end;
    //         if constexpr (is_proj)
    //         {
    //             longer_end = grid->getBoundaryVertexValue(fwd_dir_idx);
    //             if (crn_src.coord[longer_dim] == longer_end)
    //             {
    //                 linfo.fill(LosState::out_of_bounds);
    //                 return;
    //             }
    //         }
    //         else
    //             longer_end = crn_tgt.coord[longer_dim];

    //         // get directional information
    //         dir_idx_t l_dir_idx = addDirIdx(fwd_dir_idx, 2);
    //         dir_idx_t r_dir_idx = addDirIdx(fwd_dir_idx, 6);
    //         bool left_in_map = grid->inMap<false>(crn_src.coord[shorter_dim] + grid->getAdjRelCoord(l_dir_idx)[shorter_dim], l_dir_idx);
    //         bool right_in_map = grid->inMap<false>(crn_src.coord[shorter_dim] + grid->getAdjRelCoord(r_dir_idx)[shorter_dim], r_dir_idx);
    //         const mapkey_t &rel_f_key = grid->getAdjRelKey(fwd_dir_idx); // vertex or cell in long direction

    //         // init at src vertex
    //         Position pos(crn_src.coord, crn_src.mkey);
    //         int_t &longer_val = pos.coord[longer_dim];
    //         const int_t &longer_sgn = sgn_diff[longer_dim];

    //         // ------------- Case 1: BOTH SIDES OF TRACE IN MAP -------------
    //         if (left_in_map && right_in_map)
    //         {
    //             // inits
    //             dir_idx_t fl_dir_idx = addDirIdx(fwd_dir_idx, 1);
    //             const mapkey_t &rcfv_fl_key = grid->getAdjRelKeyCFV(fl_dir_idx);
    //             mapkey_t flcell_key = grid->addKeyToRelKey(crn_src.mkey, rcfv_fl_key); // cell at front Side::L of vertex
    //             dir_idx_t fr_dir_idx = addDirIdx(fwd_dir_idx, 7);
    //             const mapkey_t &rcfv_fr_key = grid->getAdjRelKeyCFV(fr_dir_idx);
    //             mapkey_t frcell_key = grid->addKeyToRelKey(crn_src.mkey, rcfv_fr_key); // cell at front Side::R of vertex

    //             // Pose identification
    //             uint8_t window = 0b0000; // prev_flcell, prev_frcell, flcell, frcell

    //             if constexpr (is_blocking)
    //             {
    //                 if (addDirIdx(crn_src.di_occ, 1) == fwd_dir_idx || addDirIdx(crn_src.di_occ, 7) == fwd_dir_idx)
    //                 {
    //                     dir_idx_t bl_dir_idx = addDirIdx(fwd_dir_idx, 3);
    //                     V2 blcell_coord = pos.coord + grid->getAdjRelCoordCFV(bl_dir_idx); // can be out of map
    //                     bool blcell_not_acc = !grid->inMap<true>(blcell_coord);
    //                     if (blcell_not_acc == false)
    //                     {
    //                         const mapkey_t &rcfv_bl_key = grid->getAdjRelKeyCFV(bl_dir_idx);
    //                         mapkey_t blcell_key = grid->addKeyToRelKey(pos.key, rcfv_bl_key);
    //                         blcell_not_acc = grid->isOc(blcell_key);
    //                     }

    //                     dir_idx_t br_dir_idx = addDirIdx(fwd_dir_idx, 5);
    //                     V2 brcell_coord = pos.coord + grid->getAdjRelCoordCFV(br_dir_idx); // can be out of map
    //                     bool brcell_not_acc = !grid->inMap<true>(brcell_coord);
    //                     if (brcell_not_acc == false)
    //                     {
    //                         const mapkey_t &rcfv_br_key = grid->getAdjRelKeyCFV(br_dir_idx);
    //                         mapkey_t brcell_key = grid->addKeyToRelKey(pos.key, rcfv_br_key);
    //                         brcell_not_acc = grid->isOc(brcell_key);
    //                     }

    //                     if ((blcell_not_acc && grid->isOc(frcell_key)) || (brcell_not_acc && grid->isOc(flcell_key)))
    //                     { // blocked
    //                         cardinalFrontBlocked(pos, l_dir_idx, r_dir_idx, linfo, grid);
    //                         return;
    //                     }
    //                 }
    //             }

    //             // move until collided or reached
    //             while (1)
    //             {
    //                 window |= (grid->isOc(flcell_key) << 1);
    //                 window |= grid->isOc(frcell_key);

    //                 switch (window)
    //                 {
    //                 case 0b0011:
    //                 case 0b0111:
    //                 case 0b1011:
    //                 case 0b1111:
    //                     cardinalFrontBlocked(pos, l_dir_idx, r_dir_idx, linfo, grid);
    //                     return;
    //                 case 0b0100:
    //                     if constexpr (find_successors)
    //                     {
    //                         emplaceSuccessor<false>(5, fwd_dir_idx, pos, linfo.successors);
    //                     }
    //                     break;
    //                 case 0b0110:
    //                     if constexpr (is_blocking)
    //                     {
    //                         cardinalFrontBlocked(pos, l_dir_idx, r_dir_idx, linfo, grid);
    //                         return;
    //                     }
    //                     else if constexpr (find_successors)
    //                     {
    //                         emplaceSuccessor<false>(5, fwd_dir_idx, pos, linfo.successors);
    //                         break;
    //                     }
    //                 case 0b1000:
    //                     if constexpr (find_successors)
    //                     {
    //                         emplaceSuccessor<false>(3, fwd_dir_idx, pos, linfo.successors);
    //                     }
    //                     break;
    //                 case 0b1001:
    //                     if constexpr (is_blocking)
    //                     {
    //                         cardinalFrontBlocked(pos, l_dir_idx, r_dir_idx, linfo, grid);
    //                         return;
    //                     }
    //                     else if constexpr (find_successors)
    //                     {
    //                         emplaceSuccessor<false>(3, fwd_dir_idx, pos, linfo.successors);
    //                         break;
    //                     }
    //                 default:
    //                     break;
    //                 }

    //                 longer_val += longer_sgn; // increment the vertex coordinate
    //                 if (longer_val == longer_end)
    //                 {
    //                     // check for ambiguous case when reached a ncv corner
    //                     if constexpr (is_proj == false)
    //                     {
    //                         assert((fwd_dir_idx & 1) == 0); // is cardinal
    //                         if (crn_tgt.is_convex == false && (addDirIdx(fwd_dir_idx, 3) == crn_tgt.di_occ || addDirIdx(fwd_dir_idx, 5) == crn_tgt.di_occ))
    //                         { // check against pseudo in a 2x2 checkerboard pattern. reject if ncv di_occ is reverse of fwd_dir_idx
    //                             cardinalFrontBlocked(pos, l_dir_idx, r_dir_idx, linfo, grid);
    //                             return; // mark as collided
    //                         }
    //                     }
    //                     break; // break if out of map / reached target
    //                 }

    //                 flcell_key = grid->addKeyToRelKey(flcell_key, rel_f_key); // get next forward cell
    //                 frcell_key = grid->addKeyToRelKey(frcell_key, rel_f_key); // get next forward cell
    //                 window <<= 2;                                             // shift window
    //                 window &= 0b1111;
    //             }
    //         }
    //         // ------------- Case 2: ONLY LEFT SIDE IN MAP  -------------
    //         else if (left_in_map)
    //         {
    //             dir_idx_t fl_dir_idx = addDirIdx(fwd_dir_idx, 1);
    //             const mapkey_t &rcfv_fl_key = grid->getAdjRelKeyCFV(fl_dir_idx);
    //             mapkey_t flcell_key = grid->addKeyToRelKey(crn_src.mkey, rcfv_fl_key); // cell at front Side::L of vertex

    //             while (1)
    //             {
    //                 if (grid->isOc(flcell_key))
    //                 { // front blocked
    //                     cardinalFrontBlocked(pos, l_dir_idx, r_dir_idx, linfo, grid);
    //                     return;
    //                 }

    //                 longer_val += longer_sgn; // increment the vertex coordinate
    //                 if (longer_val == longer_end)
    //                     break; // break if out of map / reached target

    //                 flcell_key = grid->addKeyToRelKey(flcell_key, rel_f_key); // get next forward cell
    //             }
    //         }
    //         // ------------- Case 3: ONLY RIGHT SIDE IN MAP  -------------
    //         else
    //         {
    //             dir_idx_t fr_dir_idx = addDirIdx(fwd_dir_idx, 7);
    //             const mapkey_t &rcfv_fr_key = grid->getAdjRelKeyCFV(fr_dir_idx);
    //             mapkey_t frcell_key = grid->addKeyToRelKey(crn_src.mkey, rcfv_fr_key); // cell at front Side::L of vertex

    //             while (1)
    //             {
    //                 if (grid->isOc(frcell_key))
    //                 { // front blocked
    //                     cardinalFrontBlocked(pos, l_dir_idx, r_dir_idx, linfo, grid);
    //                     return;
    //                 }

    //                 longer_val += longer_sgn; // increment the vertex coordinate
    //                 if (longer_val == longer_end)
    //                     break; // break if out of map / reached target

    //                 frcell_key = grid->addKeyToRelKey(frcell_key, rel_f_key); // get next forward cell
    //             }
    //         }
    //         if constexpr (is_proj)
    //             linfo.fill(LosState::out_of_bounds);
    //         else
    //             linfo.fill(LosState::reached);
    //         return;
    //     }
    //     inline void searchDiagonal(const Corner &crn_src, [[maybe_unused]] const Corner &crn_tgt,
    //                                const V2 &sgn_diff, const V2 &abs_diff, const uint8_t &longer_dim, const uint8_t &shorter_dim, LosInfo &linfo)
    //     {
    //         Position pos(sgn_diff, crn_src.mkey); // use as intermediate var
    //         pos.coord[longer_dim] = 0;
    //         const dir_idx_t shorter_dir_idx = dirToDirIdx(pos.coord); // direction of short

    //         pos.coord[longer_dim] = sgn_diff[longer_dim];
    //         pos.coord[shorter_dim] = 0;
    //         const dir_idx_t longer_dir_idx = dirToDirIdx(pos.coord); // direction of long

    //         int_t longer_end;
    //         if constexpr (is_proj)
    //         {
    //             int_t shorter_abs_remaining = std::abs(grid->getBoundaryVertexValue(shorter_dir_idx) - crn_src.coord[shorter_dim]);
    //             int_t longer_abs_remaining = std::abs(grid->getBoundaryVertexValue(longer_dir_idx) - crn_src.coord[longer_dim]);

    //             // determine the coordinate on the longer_dim axis where the LOS goes out of map
    //             long_t shorter_abs_as_longer = long_t(shorter_abs_remaining) * long_t(abs_diff[longer_dim]) / long_t(abs_diff[shorter_dim]); // truncate to zero
    //             if (longer_abs_remaining <= shorter_abs_as_longer)
    //                 longer_end = grid->getBoundaryVertexValue(longer_dir_idx);
    //             else
    //                 longer_end = sgn_diff[longer_dim] * shorter_abs_as_longer + crn_src.coord[longer_dim];

    //             // determine if already out of map // not possible if src and coord are in the rectangular ocgrid, and in R2
    //             if (crn_src.coord[longer_dim] == longer_end)
    //             {
    //                 linfo.fill(LosState::out_of_bounds);
    //                 return;
    //             }
    //         }
    //         else
    //             longer_end = crn_tgt.coord[longer_dim];

    //         const dir_idx_t fwd_dir_idx = dirToDirIdx(sgn_diff);
    //         const mapkey_t &rcfv_f_key = grid->getAdjRelKeyCFV(fwd_dir_idx); // forward (long and short direction) cell

    //         const mapkey_t &rel_short_key = grid->getAdjRelKey(shorter_dir_idx); // vertex / cell in short direction
    //         const mapkey_t &rel_long_key = grid->getAdjRelKey(longer_dir_idx);   // vertex / cell in long direction

    //         dir_idx_t l_dir_idx = addDirIdx(fwd_dir_idx, 2);
    //         const mapkey_t &rcfv_l_key = grid->getAdjRelKeyCFV(l_dir_idx); // left cell from vertex

    //         dir_idx_t r_dir_idx = addDirIdx(fwd_dir_idx, 6);
    //         const mapkey_t &rcfv_r_key = grid->getAdjRelKeyCFV(r_dir_idx); // right cell from vertex

    //         // init at src
    //         pos.coord = crn_src.coord; // reset to beginning (coord only, key was init above)

    //         // init short axis
    //         long_t err_s = 0;
    //         const long_t &shorter_abs_diff = abs_diff[shorter_dim];
    //         const long_t &longer_abs_diff = abs_diff[longer_dim];

    //         // track longer dim
    //         int_t &longer_val = pos.coord[longer_dim];
    //         const int_t &longer_sgn = sgn_diff[longer_dim];

    //         //  get forward cell
    //         mapkey_t fcell_key = grid->addKeyToRelKey(pos.key, rcfv_f_key); // must be in map already
    //         bool fcell_is_ob = grid->isOc(fcell_key);                       // must be in map already

    //         if constexpr (is_blocking)
    //         {                                                                    // check the existing location if there is diagonal block
    //             V2 lcell_coord = pos.coord + grid->getAdjRelCoordCFV(l_dir_idx); // can be out of map
    //             uint8_t window = grid->isAccessible(lcell_coord);
    //             window <<= 1;
    //             V2 fcell_coord = pos.coord + grid->getAdjRelCoordCFV(fwd_dir_idx);
    //             window |= grid->isAccessible(fcell_coord);
    //             window <<= 1;
    //             V2 rcell_coord = pos.coord + grid->getAdjRelCoordCFV(r_dir_idx); // can be out of map
    //             window |= grid->isAccessible(rcell_coord);
    //             window <<= 1;
    //             V2 bcell_coord = pos.coord + grid->getAdjRelCoordCFV(addDirIdx(fwd_dir_idx, 4));
    //             window |= grid->isAccessible(bcell_coord);

    //             if (window == 0b0101 && crn_src.di_occ == fwd_dir_idx)
    //             { // 0b0001 will be picked out in the while loop.
    //                 dir_idx_t ltrace_dir_idx, rtrace_dir_idx;
    //                 if (abs_diff[0] == abs_diff[1])
    //                 {
    //                     ltrace_dir_idx = addDirIdx(fwd_dir_idx, 1);
    //                     rtrace_dir_idx = addDirIdx(fwd_dir_idx, 7);
    //                 }
    //                 else if (isRightHandFrame(sgn_diff, longer_dim))
    //                 {
    //                     ltrace_dir_idx = addDirIdx(fwd_dir_idx, 1);
    //                     rtrace_dir_idx = addDirIdx(fwd_dir_idx, 5);
    //                 }
    //                 else
    //                 {
    //                     ltrace_dir_idx = addDirIdx(fwd_dir_idx, 3);
    //                     rtrace_dir_idx = addDirIdx(fwd_dir_idx, 7);
    //                 }

    //                 // ----------- (3) fill information ----------
    //                 linfo.fill(LosState::collided,
    //                            ltrace_dir_idx, pos,
    //                            rtrace_dir_idx, pos);
    //                 return;
    //             }
    //             else if (window == 0b1010)
    //             { // even tho front is blocked, this case will not be picked out bcos algo doesn't check the back.
    //                 bool trace_left = addDirIdx(fwd_dir_idx, 6) == crn_src.di_occ;
    //                 if (trace_left == false)
    //                     assert(addDirIdx(fwd_dir_idx, 2) == crn_src.di_occ); // for all start and ncv corners. make sure start has a diocc and is not zero in this case. assumes casts do not occur from goal

    //                 dir_idx_t ltrace_dir_idx, rtrace_dir_idx;
    //                 ltrace_dir_idx = addDirIdx(fwd_dir_idx, trace_left ? 1 : 3);
    //                 rtrace_dir_idx = addDirIdx(fwd_dir_idx, trace_left ? 5 : 7);

    //                 // ----------- (3) fill information ----------
    //                 linfo.fill(LosState::collided,
    //                            ltrace_dir_idx, pos,
    //                            rtrace_dir_idx, pos);
    //                 return;
    //             }

    //             // if (fcell_is_ob == false && crn_src.di_occ == fwd_dir_idx)
    //             // {
    //             //     V2 lcell_coord = pos.coord + grid->getAdjRelCoordCFV(l_dir_idx); // can be out of map
    //             //     bool lcell_not_acc = !grid->inMap<true>(lcell_coord);
    //             //     V2 rcell_coord = pos.coord + grid->getAdjRelCoordCFV(r_dir_idx); // can be out of map
    //             //     bool rcell_not_acc = !grid->inMap<true>(rcell_coord);
    //             //     if (lcell_not_acc == false)
    //             //     {
    //             //         mapkey_t lcell_key = grid->addKeyToRelKey(pos.key, rcfv_l_key);
    //             //         lcell_not_acc = grid->isOc(lcell_key);
    //             //     }
    //             //     if (rcell_not_acc == false)
    //             //     {
    //             //         mapkey_t rcell_key = grid->addKeyToRelKey(pos.key, rcfv_r_key);
    //             //         rcell_not_acc = grid->isOc(rcell_key);
    //             //     }

    //             //     if (lcell_not_acc && rcell_not_acc)
    //             //     { // blocked
    //             //     }
    //             // }
    //         }
    //         while (1)
    //         {
    //             // (A) check forward cell for collision and return if so
    //             if (fcell_is_oc)
    //             {                                              // collided, with oc cell in the cast direction (forward)
    //                 pos.coord = grid->keyToCoord<V2>(pos.key); // update short coordinate, as only key and long is changing in the algorithm

    //                 if (err_s == 0)
    //                 { // collided at vertex
    //                     // check surrounding to determine trace directions
    //                     V2 lcell_coord = pos.coord + grid->getAdjRelCoordCFV(l_dir_idx); // can be out of map
    //                     bool lcell_not_acc = !grid->inMap<true>(lcell_coord);
    //                     dir_idx_t ltrace_dir_idx;
    //                     V2 rcell_coord = pos.coord + grid->getAdjRelCoordCFV(r_dir_idx); // can be out of map
    //                     bool rcell_not_acc = !grid->inMap<true>(rcell_coord);
    //                     dir_idx_t rtrace_dir_idx;

    //                     if (lcell_not_acc == false)
    //                     {
    //                         mapkey_t lcell_key = grid->addKeyToRelKey(pos.key, rcfv_l_key);
    //                         lcell_not_acc = grid->isOc(lcell_key);
    //                     }
    //                     if (rcell_not_acc == false)
    //                     {
    //                         mapkey_t rcell_key = grid->addKeyToRelKey(pos.key, rcfv_r_key);
    //                         rcell_not_acc = grid->isOc(rcell_key);
    //                     }
    //                     // assumes the cell before current vertex is free.
    //                     uint8_t window = (lcell_not_acc << 1) | rcell_not_acc;

    //                     switch (window)
    //                     {
    //                     case 0b00: // hit a corner
    //                         if (abs_diff[0] == abs_diff[1])
    //                         {
    //                             ltrace_dir_idx = addDirIdx(fwd_dir_idx, 3);
    //                             rtrace_dir_idx = addDirIdx(fwd_dir_idx, 5);
    //                         }
    //                         else if (isRightHandFrame(sgn_diff, longer_dim))
    //                         {
    //                             ltrace_dir_idx = addDirIdx(fwd_dir_idx, 3);
    //                             rtrace_dir_idx = addDirIdx(fwd_dir_idx, 7);
    //                         }
    //                         else
    //                         {
    //                             ltrace_dir_idx = addDirIdx(fwd_dir_idx, 1);
    //                             rtrace_dir_idx = addDirIdx(fwd_dir_idx, 5);
    //                         }
    //                         break;
    //                     case 0b01: // hit a wall
    //                         ltrace_dir_idx = addDirIdx(fwd_dir_idx, 1);
    //                         rtrace_dir_idx = addDirIdx(fwd_dir_idx, 5);
    //                         break;
    //                     case 0b10: // hit a wall
    //                         ltrace_dir_idx = addDirIdx(fwd_dir_idx, 3);
    //                         rtrace_dir_idx = addDirIdx(fwd_dir_idx, 7);
    //                         break;
    //                     default: // 0b11 // hit a ncv // if both oom it is an oom case
    //                         if (abs_diff[0] == abs_diff[1])
    //                         {
    //                             ltrace_dir_idx = addDirIdx(fwd_dir_idx, 1);
    //                             rtrace_dir_idx = addDirIdx(fwd_dir_idx, 7);
    //                         }
    //                         else if (isRightHandFrame(sgn_diff, longer_dim))
    //                         {
    //                             ltrace_dir_idx = addDirIdx(fwd_dir_idx, 1);
    //                             rtrace_dir_idx = addDirIdx(fwd_dir_idx, 5);
    //                         }
    //                         else
    //                         {
    //                             ltrace_dir_idx = addDirIdx(fwd_dir_idx, 3);
    //                             rtrace_dir_idx = addDirIdx(fwd_dir_idx, 7);
    //                         }
    //                         break;
    //                     }

    //                     /* // 230414: from
    //                     // ------------ (1) LEFT --------------
    //                     V2 lcell_coord = pos.coord + grid->getAdjRelCoordCFV(l_dir_idx); // can be out of map
    //                     bool lcell_in_map = grid->inMap<true>(lcell_coord);
    //                     dir_idx_t ltrace_dir_idx;
    //                     if (lcell_in_map)
    //                     {
    //                         mapkey_t lcell_key = grid->addKeyToRelKey(pos.key, rcfv_l_key);
    //                         bool lcell_is_ob = grid->isOc(lcell_key);
    //                         ltrace_dir_idx = lcell_is_ob ? addDirIdx(fwd_dir_idx, 3) : addDirIdx(fwd_dir_idx, 1);
    //                     }
    //                     else
    //                         ltrace_dir_idx = addDirIdx(fwd_dir_idx, 3); // trace will go out of map

    //                     // ------------ (2) RIGHT --------------
    //                     V2 rcell_coord = pos.coord + grid->getAdjRelCoordCFV(r_dir_idx); // can be out of map
    //                     bool rcell_in_map = grid->inMap<true>(rcell_coord);
    //                     dir_idx_t rtrace_dir_idx;
    //                     if (rcell_in_map)
    //                     {
    //                         mapkey_t rcell_key = grid->addKeyToRelKey(pos.key, rcfv_r_key);
    //                         bool rcell_is_ob = grid->isOc(rcell_key);
    //                         rtrace_dir_idx = rcell_is_ob ? addDirIdx(fwd_dir_idx, 5) : addDirIdx(fwd_dir_idx, 7);
    //                     }
    //                     else
    //                         rtrace_dir_idx = addDirIdx(fwd_dir_idx, 5);
    //                     */

    //                     // ----------- (3) fill information ----------
    //                     linfo.fill(LosState::collided,
    //                                ltrace_dir_idx, pos,
    //                                rtrace_dir_idx, pos);
    //                 }
    //                 else
    //                 { // collided parallel to short axis
    //                     if (isRightHandFrame(sgn_diff, longer_dim))
    //                     {
    //                         V2 left_trace_coord = pos.coord;
    //                         left_trace_coord[shorter_dim] = pos.coord[shorter_dim] + sgn_diff[shorter_dim]; // sum due to truncation to zero
    //                         mapkey_t left_trace_key = grid->addKeyToRelKey(pos.key, rel_short_key);

    //                         linfo.fill(LosState::collided,
    //                                    addDirIdx(fwd_dir_idx, 1), left_trace_coord[0], left_trace_coord[1], left_trace_key,
    //                                    addDirIdx(fwd_dir_idx, 5), pos.coord[0], pos.coord[1], pos.key);
    //                     }
    //                     else
    //                     {
    //                         V2 right_trace_coord = pos.coord;
    //                         right_trace_coord[shorter_dim] = pos.coord[shorter_dim] + sgn_diff[shorter_dim];
    //                         mapkey_t right_trace_key = grid->addKeyToRelKey(pos.key, rel_short_key);

    //                         linfo.fill(LosState::collided,
    //                                    addDirIdx(fwd_dir_idx, 3), pos.coord[0], pos.coord[1], pos.key,
    //                                    addDirIdx(fwd_dir_idx, 7), right_trace_coord[0], right_trace_coord[1], right_trace_key);
    //                     }
    //                 }
    //                 return;
    //             }

    //             // (B) find error in short axis
    //             err_s += shorter_abs_diff;
    //             // (C) check if error is large and ...
    //             if (err_s == longer_abs_diff)
    //             { // ... intersects vertex
    //                 // (1) move to vertex in short direction
    //                 pos.key = grid->addKeyToRelKey(pos.key, rel_short_key);
    //                 // (2) reset error
    //                 err_s = 0; // err_s -= longer_abs_diff
    //             }
    //             else if (err_s > longer_abs_diff) // note this comparison is gt, not ge
    //             {                                 // ... intersects edge (parallel to long axis)
    //                 // (1) move to vertex in short direction
    //                 pos.key = grid->addKeyToRelKey(pos.key, rel_short_key);
    //                 // (2) check cell in forward direction adjacent to edge
    //                 fcell_key = grid->addKeyToRelKey(pos.key, rcfv_f_key);
    //                 fcell_is_ob = grid->isOc(fcell_key);
    //                 if (fcell_is_oc)
    //                 { // collide with grid edge that is parallel to long axis
    //                     // calculate coordinate
    //                     pos.coord = grid->keyToCoord<V2>(pos.key);

    //                     if (isRightHandFrame(sgn_diff, longer_dim))
    //                     {
    //                         V2 right_trace_coord = pos.coord;
    //                         right_trace_coord[longer_dim] = pos.coord[longer_dim] + sgn_diff[longer_dim];
    //                         mapkey_t right_trace_key = grid->addKeyToRelKey(pos.key, rel_long_key);

    //                         linfo.fill(LosState::collided,
    //                                    addDirIdx(fwd_dir_idx, 3), pos.coord[0], pos.coord[1], pos.key,
    //                                    addDirIdx(fwd_dir_idx, 7), right_trace_coord[0], right_trace_coord[1], right_trace_key);
    //                     }
    //                     else
    //                     {
    //                         V2 left_trace_coord = pos.coord;
    //                         left_trace_coord[longer_dim] = pos.coord[longer_dim] + sgn_diff[longer_dim];
    //                         mapkey_t left_trace_key = grid->addKeyToRelKey(pos.key, rel_long_key);
    //                         linfo.fill(LosState::collided,
    //                                    addDirIdx(fwd_dir_idx, 1), left_trace_coord[0], left_trace_coord[1], left_trace_key,
    //                                    addDirIdx(fwd_dir_idx, 5), pos.coord[0], pos.coord[1], pos.key);
    //                     }
    //                     return;
    //                 }
    //                 // (3) reset error
    //                 err_s -= longer_abs_diff;
    //             }

    //             // (D) move to vertex in long direction
    //             pos.key = grid->addKeyToRelKey(pos.key, rel_long_key);
    //             longer_val += longer_sgn;

    //             // (E) check if target reached or out of map
    //             if (longer_val == longer_end)
    //             {
    //                 // check for ambiguous case when reached a ncv corner
    //                 if constexpr (is_proj == false)
    //                 {
    //                     assert((fwd_dir_idx & 1) == 1); // cannot be cardinal
    //                     if (crn_tgt.is_convex == false && crn_tgt.di_occ != fwd_dir_idx)
    //                     {                                                              // check against pseudo in a 2x2 checkerboard pattern. reject if ncv di_occ is reverse of fwd_dir_idx
    //                         assert(crn_tgt.di_occ == addDirIdx(fwd_dir_idx, 4)); // has to be opposite, cannot reach ncv from within the obstacle
    //                         fcell_is_ob = true;
    //                         continue; // mark as collided
    //                     }
    //                 }
    //                 break;
    //             }

    //             // (F) get forward cell information
    //             fcell_key = grid->addKeyToRelKey(pos.key, rcfv_f_key);
    //             fcell_is_ob = grid->isOc(fcell_key);
    //             // (G) evaluate if Pose is found
    //             if (err_s == 0 && !fcell_is_oc)
    //             { // forward is free and path intersects vertex
    //                 // placed here to avoid evaluating at src's coordinate ==> the cell behind the vertex is free to reach here
    //                 mapkey_t lcell_key = grid->addKeyToRelKey(pos.key, rcfv_l_key);
    //                 mapkey_t rcell_key = grid->addKeyToRelKey(pos.key, rcfv_r_key);
    //                 bool lcell_is_ob = grid->isOc(lcell_key);
    //                 bool rcell_is_ob = grid->isOc(rcell_key);

    //                 if constexpr (is_blocking)
    //                 {
    //                     uint8_t window = (lcell_is_ob << 1) | rcell_is_oc;
    //                     switch (window)
    //                     {
    //                     case 0b11:
    //                         // is_blocking
    //                         pos.coord = grid->keyToCoord(pos.key);
    //                         // linfo.fill(LosState::collided,
    //                         //           addDirIdx(fwd_dir_idx, 3), pos,
    //                         //           addDirIdx(fwd_dir_idx, 5), pos);

    //                         if (abs_diff[0] == abs_diff[1])
    //                         {
    //                             linfo.fill(LosState::collided,
    //                                        addDirIdx(fwd_dir_idx, 1), pos,
    //                                        addDirIdx(fwd_dir_idx, 7), pos);
    //                         }
    //                         else if (isRightHandFrame(sgn_diff, longer_dim))
    //                         {
    //                             linfo.fill(LosState::collided,
    //                                        addDirIdx(fwd_dir_idx, 1), pos,
    //                                        addDirIdx(fwd_dir_idx, 5), pos);
    //                         }
    //                         else
    //                         {
    //                             linfo.fill(LosState::collided,
    //                                        addDirIdx(fwd_dir_idx, 3), pos,
    //                                        addDirIdx(fwd_dir_idx, 7), pos);
    //                         }
    //                         return;
    //                     case 0b10:
    //                         if constexpr (find_successors)
    //                         { // Pose at Side::L
    //                             emplaceSuccessor<true>(2, fwd_dir_idx, pos, linfo.successors);
    //                             break;
    //                         }
    //                     case 0b01:
    //                         if constexpr (find_successors)
    //                         { // Pose at Side::R
    //                             emplaceSuccessor<true>(6, fwd_dir_idx, pos, linfo.successors);
    //                             break;
    //                         }
    //                     default:
    //                         break;
    //                     }
    //                 }
    //                 else
    //                 {
    //                     if constexpr (find_successors)
    //                     {
    //                         if (lcell_is_oc)
    //                         { // Pose at Side::L
    //                             emplaceSuccessor<true>(2, fwd_dir_idx, pos, linfo.successors);
    //                         }
    //                         if (rcell_is_oc)
    //                         { // Pose at Side::R
    //                             emplaceSuccessor<true>(6, fwd_dir_idx, pos, linfo.successors);
    //                         }
    //                     }
    //                 }
    //             }
    //         }
    //         if constexpr (is_proj)
    //             linfo.fill(LosState::out_of_bounds);
    //         else
    //             linfo.fill(LosState::reached);
    //         return;
    //     }

    // public:
    //     LosAdvanced(OcGrid<false> *grid) : Base(grid) {}
    //     void cast(const Ray *const &ray, LosInfo &linfo)
    //     {
    //         V2 diff = ray->vec;
    //         if constexpr (is_proj == false)
    //         {
    //             if (ray->crn_src->mkey == ray->crn_tgt->mkey)
    //             {
    //                 linfo.fill(LosState::reached);
    //                 return;
    //             }
    //         }

    //         if constexpr (find_successors)
    //             linfo.successors.clear();

    //         V2 abs_diff = abs(diff);
    //         V2 sgn_diff = sgn(diff);
    //         uint8_t longer_dim = (abs_diff[0] >= abs_diff[1] ? 0 : 1);
    //         uint8_t shorter_dim = (abs_diff[0] >= abs_diff[1] ? 1 : 0);
    //         bool is_cardinal = abs_diff[shorter_dim] == 0;

    //         Corner *crn_src;
    //         if constexpr (is_proj == true)
    //             crn_src = ray->crn_tgt;
    //         else
    //             crn_src = ray->crn_src;

    //         if (is_cardinal == true)
    //             searchCardinal(*crn_src, *(ray->crn_tgt), sgn_diff, longer_dim, shorter_dim, linfo);
    //         else
    //             searchDiagonal(*crn_src, *(ray->crn_tgt), sgn_diff, abs_diff, longer_dim, shorter_dim, linfo);
    //     }
    //     LosInfo cast(const Ray *const &ray)
    //     {
    //         LosInfo linfo;
    //         cast(ray, linfo);
    //         return linfo;
    //     }
    // };
}