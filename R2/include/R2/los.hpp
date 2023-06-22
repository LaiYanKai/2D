#include "P2D/P2D.hpp"

#pragma once
namespace P2D::R2
{
    // project / cast
    // diag_block
    // diag_block_at_start = false
    struct LosResult
    {
        V2 coord_left = {-1, -1}, coord_right = {-1, -1};
        dir_idx_t di_left = 0, di_right = 0;
        bool collided = true;
        LosResult(const bool &collided, const V2 &coord_left, const dir_idx_t &di_left, const V2 &coord_right, const dir_idx_t &di_right)
            : coord_left(coord_left), coord_right(coord_right), di_left(di_left), di_right(di_right), collided(collided) {}
    };

    class Los
    {
        Grid *const grid;

        Los(Grid *const &grid) : grid(grid) {}

        // cast from Src in the direction src->tgt to Tgt
        template <bool diag_block, bool diag_block_at_start>
        LosResult cast(const V2 &src_coord, const mapkey_t &src_key, const V2 &tgt_coord, const mapkey_t &tgt_key)
        {
        }

        // project from Tgt in the direction src->tgt
        template <bool diag_block, bool diag_block_at_start>
        LosResult project(const V2 &src_coord, const V2 &tgt_coord)
        {
            
        }

    };



    // using LosState = P2D::Los2::State;

    // struct LosInfo
    // {
    //     LosState state;
    //     std::array<Pose, 2> trace_poses;
    //     std::vector<Pose> successors;

    //     LosInfo(const LosState &state,
    //             const dir_idx_t &left_trace_dir_idx, const int_t &left_trace_coord_x, const int_t &left_trace_coord_y, const mapkey_t &left_trace_key,
    //             const dir_idx_t &right_trace_dir_idx, const int_t &right_trace_coord_x, const int_t &right_trace_coord_y, const mapkey_t &right_trace_key)
    //         : successors()
    //     {
    //         fill(state,
    //              left_trace_dir_idx, left_trace_coord_x, left_trace_coord_y, left_trace_key,
    //              right_trace_dir_idx, right_trace_coord_x, right_trace_coord_y, right_trace_key);
    //     }
    //     LosInfo() : state(LosState::collided), trace_poses(), successors() {}
    //     inline void fill(const LosState &state,
    //                      const dir_idx_t &left_trace_dir_idx, const int_t &left_trace_coord_x, const int_t &left_trace_coord_y, const mapkey_t &left_trace_key,
    //                      const dir_idx_t &right_trace_dir_idx, const int_t &right_trace_coord_x, const int_t &right_trace_coord_y, const mapkey_t &right_trace_key)
    //     {
    //         this->state = state;
    //         trace_poses[Side::L] = {left_trace_dir_idx, left_trace_coord_x, left_trace_coord_y, left_trace_key};
    //         trace_poses[Side::R] = {right_trace_dir_idx, right_trace_coord_x, right_trace_coord_y, right_trace_key};
    //     }
    //     inline void fill(const LosState &state,
    //                      const dir_idx_t &left_trace_dir_idx, const Position &pos_left,
    //                      const dir_idx_t &right_trace_dir_idx, const Position &pos_right)
    //     {
    //         fill(state,
    //              left_trace_dir_idx, pos_left.coord[0], pos_left.coord[1], pos_left.key,
    //              right_trace_dir_idx, pos_right.coord[0], pos_right.coord[1], pos_right.key);
    //     }
    //     inline void fill(const LosState &state) { this->state = state; }

    //     friend std::ostream &operator<<(std::ostream &out, const LosInfo &linfo);
    // };

    // template <bool is_proj, bool find_successors, bool is_blocking>
    // class Los : public P2D::Los2::Base<false>
    // { // no is_blocking, return state; assumes path traversed is in map
    //   // using eod_t = typename std::conditional<is_proj, V2, Position>::type;

    // private:
    //     template <bool to_coord>
    //     inline void emplaceSuccessor(const dir_idx_t &rel_from_fwd_dir_idx, const dir_idx_t &fwd_dir_idx, Position &pos, std::vector<Pose> &successors)
    //     {
    //         dir_idx_t crn_dir_idx = addDirIdx<true>(fwd_dir_idx, rel_from_fwd_dir_idx);
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
    //         dir_idx_t l_dir_idx = addDirIdx<true>(fwd_dir_idx, 2);
    //         dir_idx_t r_dir_idx = addDirIdx<true>(fwd_dir_idx, 6);
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
    //             dir_idx_t fl_dir_idx = addDirIdx<true>(fwd_dir_idx, 1);
    //             const mapkey_t &rcfv_fl_key = grid->getAdjRelKeyCFV(fl_dir_idx);
    //             mapkey_t flcell_key = grid->addKeyToRelKey(crn_src.mkey, rcfv_fl_key); // cell at front Side::L of vertex
    //             dir_idx_t fr_dir_idx = addDirIdx<true>(fwd_dir_idx, 7);
    //             const mapkey_t &rcfv_fr_key = grid->getAdjRelKeyCFV(fr_dir_idx);
    //             mapkey_t frcell_key = grid->addKeyToRelKey(crn_src.mkey, rcfv_fr_key); // cell at front Side::R of vertex

    //             // Pose identification
    //             uint8_t window = 0b0000; // prev_flcell, prev_frcell, flcell, frcell

    //             if constexpr (is_blocking)
    //             {
    //                 if (addDirIdx(crn_src.di_occ, 1) == fwd_dir_idx || addDirIdx(crn_src.di_occ, 7) == fwd_dir_idx)
    //                 {
    //                     dir_idx_t bl_dir_idx = addDirIdx<true>(fwd_dir_idx, 3);
    //                     V2 blcell_coord = pos.coord + grid->getAdjRelCoordCFV(bl_dir_idx); // can be out of map
    //                     bool blcell_not_acc = !grid->inMap<true>(blcell_coord);
    //                     if (blcell_not_acc == false)
    //                     {
    //                         const mapkey_t &rcfv_bl_key = grid->getAdjRelKeyCFV(bl_dir_idx);
    //                         mapkey_t blcell_key = grid->addKeyToRelKey(pos.key, rcfv_bl_key);
    //                         blcell_not_acc = grid->isOc(blcell_key);
    //                     }

    //                     dir_idx_t br_dir_idx = addDirIdx<true>(fwd_dir_idx, 5);
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
    //                         if (crn_tgt.is_convex == false && (addDirIdx<true>(fwd_dir_idx, 3) == crn_tgt.di_occ || addDirIdx<true>(fwd_dir_idx, 5) == crn_tgt.di_occ))
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
    //             dir_idx_t fl_dir_idx = addDirIdx<true>(fwd_dir_idx, 1);
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
    //             dir_idx_t fr_dir_idx = addDirIdx<true>(fwd_dir_idx, 7);
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

    //         dir_idx_t l_dir_idx = addDirIdx<true>(fwd_dir_idx, 2);
    //         const mapkey_t &rcfv_l_key = grid->getAdjRelKeyCFV(l_dir_idx); // left cell from vertex

    //         dir_idx_t r_dir_idx = addDirIdx<true>(fwd_dir_idx, 6);
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
    //         bool fcell_is_oc = grid->isOc(fcell_key);                       // must be in map already

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
    //             V2 bcell_coord = pos.coord + grid->getAdjRelCoordCFV(addDirIdx<true>(fwd_dir_idx, 4));
    //             window |= grid->isAccessible(bcell_coord);

    //             if (window == 0b0101 && crn_src.di_occ == fwd_dir_idx)
    //             { // 0b0001 will be picked out in the while loop.
    //                 dir_idx_t ltrace_dir_idx, rtrace_dir_idx;
    //                 if (abs_diff[0] == abs_diff[1])
    //                 {
    //                     ltrace_dir_idx = addDirIdx<true>(fwd_dir_idx, 1);
    //                     rtrace_dir_idx = addDirIdx<true>(fwd_dir_idx, 7);
    //                 }
    //                 else if (isRightHandFrame(sgn_diff, longer_dim))
    //                 {
    //                     ltrace_dir_idx = addDirIdx<true>(fwd_dir_idx, 1);
    //                     rtrace_dir_idx = addDirIdx<true>(fwd_dir_idx, 5);
    //                 }
    //                 else
    //                 {
    //                     ltrace_dir_idx = addDirIdx<true>(fwd_dir_idx, 3);
    //                     rtrace_dir_idx = addDirIdx<true>(fwd_dir_idx, 7);
    //                 }

    //                 // ----------- (3) fill information ----------
    //                 linfo.fill(LosState::collided,
    //                            ltrace_dir_idx, pos,
    //                            rtrace_dir_idx, pos);
    //                 return;
    //             }
    //             else if (window == 0b1010)
    //             { // even tho front is blocked, this case will not be picked out bcos algo doesn't check the back.
    //                 bool trace_left = addDirIdx<true>(fwd_dir_idx, 6) == crn_src.di_occ;
    //                 if (trace_left == false)
    //                     assert(addDirIdx<true>(fwd_dir_idx, 2) == crn_src.di_occ); // for all start and ncv corners. make sure start has a diocc and is not zero in this case. assumes casts do not occur from goal

    //                 dir_idx_t ltrace_dir_idx, rtrace_dir_idx;
    //                 ltrace_dir_idx = addDirIdx<true>(fwd_dir_idx, trace_left ? 1 : 3);
    //                 rtrace_dir_idx = addDirIdx<true>(fwd_dir_idx, trace_left ? 5 : 7);

    //                 // ----------- (3) fill information ----------
    //                 linfo.fill(LosState::collided,
    //                            ltrace_dir_idx, pos,
    //                            rtrace_dir_idx, pos);
    //                 return;
    //             }

    //             // if (fcell_is_oc == false && crn_src.di_occ == fwd_dir_idx)
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
    //                             ltrace_dir_idx = addDirIdx<true>(fwd_dir_idx, 3);
    //                             rtrace_dir_idx = addDirIdx<true>(fwd_dir_idx, 5);
    //                         }
    //                         else if (isRightHandFrame(sgn_diff, longer_dim))
    //                         {
    //                             ltrace_dir_idx = addDirIdx<true>(fwd_dir_idx, 3);
    //                             rtrace_dir_idx = addDirIdx<true>(fwd_dir_idx, 7);
    //                         }
    //                         else
    //                         {
    //                             ltrace_dir_idx = addDirIdx<true>(fwd_dir_idx, 1);
    //                             rtrace_dir_idx = addDirIdx<true>(fwd_dir_idx, 5);
    //                         }
    //                         break;
    //                     case 0b01: // hit a wall
    //                         ltrace_dir_idx = addDirIdx<true>(fwd_dir_idx, 1);
    //                         rtrace_dir_idx = addDirIdx<true>(fwd_dir_idx, 5);
    //                         break;
    //                     case 0b10: // hit a wall
    //                         ltrace_dir_idx = addDirIdx<true>(fwd_dir_idx, 3);
    //                         rtrace_dir_idx = addDirIdx<true>(fwd_dir_idx, 7);
    //                         break;
    //                     default: // 0b11 // hit a ncv // if both oom it is an oom case
    //                         if (abs_diff[0] == abs_diff[1])
    //                         {
    //                             ltrace_dir_idx = addDirIdx<true>(fwd_dir_idx, 1);
    //                             rtrace_dir_idx = addDirIdx<true>(fwd_dir_idx, 7);
    //                         }
    //                         else if (isRightHandFrame(sgn_diff, longer_dim))
    //                         {
    //                             ltrace_dir_idx = addDirIdx<true>(fwd_dir_idx, 1);
    //                             rtrace_dir_idx = addDirIdx<true>(fwd_dir_idx, 5);
    //                         }
    //                         else
    //                         {
    //                             ltrace_dir_idx = addDirIdx<true>(fwd_dir_idx, 3);
    //                             rtrace_dir_idx = addDirIdx<true>(fwd_dir_idx, 7);
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
    //                         bool lcell_is_oc = grid->isOc(lcell_key);
    //                         ltrace_dir_idx = lcell_is_oc ? addDirIdx<true>(fwd_dir_idx, 3) : addDirIdx<true>(fwd_dir_idx, 1);
    //                     }
    //                     else
    //                         ltrace_dir_idx = addDirIdx<true>(fwd_dir_idx, 3); // trace will go out of map

    //                     // ------------ (2) RIGHT --------------
    //                     V2 rcell_coord = pos.coord + grid->getAdjRelCoordCFV(r_dir_idx); // can be out of map
    //                     bool rcell_in_map = grid->inMap<true>(rcell_coord);
    //                     dir_idx_t rtrace_dir_idx;
    //                     if (rcell_in_map)
    //                     {
    //                         mapkey_t rcell_key = grid->addKeyToRelKey(pos.key, rcfv_r_key);
    //                         bool rcell_is_oc = grid->isOc(rcell_key);
    //                         rtrace_dir_idx = rcell_is_oc ? addDirIdx<true>(fwd_dir_idx, 5) : addDirIdx<true>(fwd_dir_idx, 7);
    //                     }
    //                     else
    //                         rtrace_dir_idx = addDirIdx<true>(fwd_dir_idx, 5);
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
    //                                    addDirIdx<true>(fwd_dir_idx, 1), left_trace_coord[0], left_trace_coord[1], left_trace_key,
    //                                    addDirIdx<true>(fwd_dir_idx, 5), pos.coord[0], pos.coord[1], pos.key);
    //                     }
    //                     else
    //                     {
    //                         V2 right_trace_coord = pos.coord;
    //                         right_trace_coord[shorter_dim] = pos.coord[shorter_dim] + sgn_diff[shorter_dim];
    //                         mapkey_t right_trace_key = grid->addKeyToRelKey(pos.key, rel_short_key);

    //                         linfo.fill(LosState::collided,
    //                                    addDirIdx<true>(fwd_dir_idx, 3), pos.coord[0], pos.coord[1], pos.key,
    //                                    addDirIdx<true>(fwd_dir_idx, 7), right_trace_coord[0], right_trace_coord[1], right_trace_key);
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
    //                 fcell_is_oc = grid->isOc(fcell_key);
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
    //                                    addDirIdx<true>(fwd_dir_idx, 3), pos.coord[0], pos.coord[1], pos.key,
    //                                    addDirIdx<true>(fwd_dir_idx, 7), right_trace_coord[0], right_trace_coord[1], right_trace_key);
    //                     }
    //                     else
    //                     {
    //                         V2 left_trace_coord = pos.coord;
    //                         left_trace_coord[longer_dim] = pos.coord[longer_dim] + sgn_diff[longer_dim];
    //                         mapkey_t left_trace_key = grid->addKeyToRelKey(pos.key, rel_long_key);
    //                         linfo.fill(LosState::collided,
    //                                    addDirIdx<true>(fwd_dir_idx, 1), left_trace_coord[0], left_trace_coord[1], left_trace_key,
    //                                    addDirIdx<true>(fwd_dir_idx, 5), pos.coord[0], pos.coord[1], pos.key);
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
    //                         assert(crn_tgt.di_occ == addDirIdx<true>(fwd_dir_idx, 4)); // has to be opposite, cannot reach ncv from within the obstacle
    //                         fcell_is_oc = true;
    //                         continue; // mark as collided
    //                     }
    //                 }
    //                 break;
    //             }

    //             // (F) get forward cell information
    //             fcell_key = grid->addKeyToRelKey(pos.key, rcfv_f_key);
    //             fcell_is_oc = grid->isOc(fcell_key);
    //             // (G) evaluate if Pose is found
    //             if (err_s == 0 && !fcell_is_oc)
    //             { // forward is free and path intersects vertex
    //                 // placed here to avoid evaluating at src's coordinate ==> the cell behind the vertex is free to reach here
    //                 mapkey_t lcell_key = grid->addKeyToRelKey(pos.key, rcfv_l_key);
    //                 mapkey_t rcell_key = grid->addKeyToRelKey(pos.key, rcfv_r_key);
    //                 bool lcell_is_oc = grid->isOc(lcell_key);
    //                 bool rcell_is_oc = grid->isOc(rcell_key);

    //                 if constexpr (is_blocking)
    //                 {
    //                     uint8_t window = (lcell_is_oc << 1) | rcell_is_oc;
    //                     switch (window)
    //                     {
    //                     case 0b11:
    //                         // is_blocking
    //                         pos.coord = grid->keyToCoord(pos.key);
    //                         // linfo.fill(LosState::collided,
    //                         //           addDirIdx<true>(fwd_dir_idx, 3), pos,
    //                         //           addDirIdx<true>(fwd_dir_idx, 5), pos);

    //                         if (abs_diff[0] == abs_diff[1])
    //                         {
    //                             linfo.fill(LosState::collided,
    //                                        addDirIdx<true>(fwd_dir_idx, 1), pos,
    //                                        addDirIdx<true>(fwd_dir_idx, 7), pos);
    //                         }
    //                         else if (isRightHandFrame(sgn_diff, longer_dim))
    //                         {
    //                             linfo.fill(LosState::collided,
    //                                        addDirIdx<true>(fwd_dir_idx, 1), pos,
    //                                        addDirIdx<true>(fwd_dir_idx, 5), pos);
    //                         }
    //                         else
    //                         {
    //                             linfo.fill(LosState::collided,
    //                                        addDirIdx<true>(fwd_dir_idx, 3), pos,
    //                                        addDirIdx<true>(fwd_dir_idx, 7), pos);
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
    //     Los(OcGrid<false> *grid) : Base(grid) {}
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