#include <unordered_map>
#include <filesystem>

#include "types.hpp"
#include "math.hpp"
#include "Vec2.hpp"
#include "dir_idx.hpp"

#pragma once
namespace P2D
{
    template <bool diag_block = true>
    class Los
    {
    private:
        Grid *const grid;

    public:
        Los(Grid *const &grid) : grid(grid) {}
        Los &operator=(const Los &) = delete; // Disallow copying
        Los(const Los &) = delete;
        ~Los() {}

        // Tests visibility between start vertex with key ki, coordinate vi, and destination vertex with key kf and coordinate vf.
        // Returns true if visible.
        // Assumes the grid is rectangle, and both vertices are in map.
        template <bool diag_block_at_start = false>
        bool cast(const mapkey_t &ki, const V2 &pi, const mapkey_t &kf, const V2 &pf) const
        {
            assert(grid->inMap<false>(pi) == true);
            assert(grid->inMap<false>(pf) == true);
            assert(ki == grid->coordToKey<false>(pi));
            assert(kf == grid->coordToKey<false>(pf));

            if (ki == kf) // nothing to check
                return true;

            V2 dir = pf - pi;
            V2 abs_dir = P2D::abs(dir);
            V2 sgn_dir = P2D::sgn(dir);
            dir_idx_t di = dirToDirIdx(dir);
            bool dim_long = abs_dir.x >= abs_dir.y ? 0 : 1;

            if (isCardinal(di) == true)
                return _castCardinal<diag_block_at_start>(ki, pi, di, dim_long, abs_dir);
            else
                return _castOrdinal<diag_block_at_start>(ki, pi, di, dim_long, abs_dir, sgn_dir);
        }

    private:
        template <bool diag_block_at_start>
        bool _castOrdinal(mapkey_t kv, const V2 &pi, const dir_idx_t &di, const bool &dim_long, const V2 &abs_dir, const V2 &sgn_dir) const
        {
            assert(sgn_dir.x != 0 && sgn_dir.y != 0);
            bool is_rh_frame = (dim_long == 0) == (sgn_dir.x == sgn_dir.y); // provided that sgn_dir for both not zero (i.e. di is ordinal)
            dir_idx_t di_long = addDirIdx(di, is_rh_frame ? 7 : 1);
            dir_idx_t di_short = addDirIdx(di, is_rh_frame ? 1 : 7);

            mapkey_t rkv_long = grid->getRelKey<false>(di_long);
            mapkey_t rkv_short = grid->getRelKey<false>(di_short);

            mapkey_t kc = grid->addKeyToRelKey(kv, grid->getCellRelKey(di, pi.x)); // currently the cell in di direction (front)
            mapkey_t rkc_long = grid->getRelKey<true>(di_long);
            mapkey_t rkc_short = grid->getRelKey<true>(di_short);

            // ======= check diag_block_at_start =========
            if constexpr (diag_block && diag_block_at_start)
            {                                                                                           // check diag_block at start (need to check oom for this stage)
                V2 coord = grid->keyToCoord<true>(kc) + grid->getRelCoord<true>(addDirIdx(di_long, 4)); // coord in reverse di_long direction
                mapkey_t key = grid->addKeyToRelKey(kc, -rkc_long);
                bool is_blocked_long = !grid->isAccessible(key, coord);

                coord = grid->keyToCoord<true>(kc) + grid->getRelCoord<true>(addDirIdx(di_short, 4)); // coord in reverse di_long direction
                key = grid->addKeyToRelKey(kc, -rkc_short);
                bool is_blocked_short = !grid->isAccessible(key, coord);

                if (is_blocked_long && is_blocked_short)
                    return false; // blocked
            }

            int_t step = 0;
            int_t err_short = 0;
            const int_t &abs_long = abs_dir[dim_long];
            assert(abs_long != 0);
            const int_t &abs_short = abs_dir[!dim_long];
            while (true)
            {
                // ====== check cell in front =====
                if (grid->isOc(kc))
                    return false; // front blocked

                // ====== check long short error =====
                err_short += abs_short; // short_abs_dir
                if (err_short >= abs_long)
                { // intersects grid edge (parallel to long axis)
                    // (1) move in short direction
                    kv = grid->addKeyToRelKey(kv, rkv_short);
                    kc = grid->addKeyToRelKey(kc, rkc_short);

                    // (2) check cell in front if did not intersect grid vertex
                    if (err_short > abs_long && grid->isOc(kc))
                        return false; // collided

                    // (3) reset error
                    err_short -= abs_long;
                }

                // move in long direction
                kv = grid->addKeyToRelKey(kv, rkv_long);
                kc = grid->addKeyToRelKey(kc, rkc_long);

                // break if reached
                if (++step == abs_long)
                    return true;

                // ====== Check Diagonal block =====
                if (diag_block && err_short == 0)
                {
                    // Checked cells are always in map
                    bool is_blocked_long = grid->isOc(grid->addKeyToRelKey(kc, -rkc_long));   // cell in reverse long dir
                    bool is_blocked_short = grid->isOc(grid->addKeyToRelKey(kc, -rkc_short)); // cell in reverse short dir
                    if (is_blocked_long && is_blocked_short)
                        return false; // blocked
                }
            }
        }

        template <bool diag_block_at_start>
        bool _castCardinal(const mapkey_t &ki, const V2 &pi, const dir_idx_t &di, const bool &dim_long, const V2 &abs_dir) const
        {
            // get bl and br
            mapkey_t kc_l = grid->addKeyToRelKey(ki, grid->getCellRelKey(addDirIdx(di, 3), pi.x));
            mapkey_t kc_r = grid->addKeyToRelKey(ki, grid->getCellRelKey(addDirIdx(di, 5), pi.x));
            mapkey_t rkc = grid->getRelKey<true>(di);

            const bool out_of_map_left = !grid->inMap<false>(pi + grid->getRelCoord<false>(addDirIdx(di, 2)));
            const bool out_of_map_right =!grid->inMap<false>(pi + grid->getRelCoord<false>(addDirIdx(di, 6)));
            const bool out_of_map_back = !grid->inMap<false>(pi + grid->getRelCoord<false>(addDirIdx(di, 4)));

            bool is_blocked_left = false;
            bool is_blocked_right = false;
            if constexpr (diag_block == true && diag_block_at_start == false)
            {
                is_blocked_left = out_of_map_left || out_of_map_back || grid->isOc(kc_l);
                is_blocked_right = out_of_map_right || out_of_map_back || grid->isOc(kc_r);
            }
            char window = (is_blocked_left << 1) | (is_blocked_right);

            const int_t &max_step = abs_dir[dim_long];
            for (int_t step = 0; step < max_step; ++step)
            {
                // shift window back and filter
                window <<= 2;
                window &= 0b1111;

                // shift keys forward
                kc_l = grid->addKeyToRelKey(kc_l, rkc);
                kc_r = grid->addKeyToRelKey(kc_r, rkc);

                // determine window
                is_blocked_left = out_of_map_left || grid->isOc(kc_l);
                is_blocked_right = out_of_map_right || grid->isOc(kc_r);
                window |= (is_blocked_left << 1) | (is_blocked_right);

                // determine blocking
                switch (window)
                {
                case 0b0011:
                case 0b1011:
                case 0b0111:
                case 0b1111:
                    return false; // no vis bcos front blocked
                case 0b1001:
                case 0b0110:
                    if constexpr (diag_block)
                        return false; // no vis bcos front blocked
                    break;
                default:
                    break;
                }
            }
            return true; // has visibility
        }
    };

}
