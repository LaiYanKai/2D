#include <unordered_map>
#include <filesystem>

#include "P2D/P2D.hpp"
#include "node.hpp"

#pragma once
namespace P2D::VG2
{
    template <bool diag_block = true>
    class Los
    {
    private:
        V2 pi, pf, dir, abs_dir, sgn_dir;
        Grid *const grid;
        mapkey_t ki, kf;
        dir_idx_t di;
        char dim_long;

    public:
        Los(Grid *const &grid, std::filesystem::path &fp_vg) : grid(grid) {}
        Los &operator=(const Los &) = delete; // Disallow copying
        Los(const Los &) = delete;
        ~Los() {}

        // Tests visibility between start vertex with key ki, coordinate vi, and destination vertex with key kf, coordinate vf.
        // Returns true if visible.
        // Assumes the grid is rectangle, and both vertices are in map.
        bool cast(const mapkey_t &ki, const V2 &pi, const mapkey_t &kf, const V2 &pf) const
        {
            assert(grid->inMap<false>(vi) == true);
            assert(grid->inMap<false>(vf) == true);
            assert(ki == grid->coordToKey<false>(pi));
            assert(kf == grid->coordToKey<false>(pf));

            if (ki == kf) // nothing to check
                return true;

            this->ki = ki;
            this->kf = kf;
            this->pi = pi;
            this->pf = pf;
            dir = pf - pi;
            abs_dir = P2D::abs(dir);
            sgn_dir = P2D::sgn(dir);
            di = dirToDirIdx(dir);
            dim_long = abs_dir.x >= abs_dir.y ? 0 : 1;

            if (isCardinal(di) == true)
                _castCardinal();
            else
                _castOrdinal();
        }

    private:
        bool _castOrdinal() const
        {
            const dir_idx_t fwd_dir_idx = dirToDirIdx<V2>(sgn_diff);
            const mapkey_t &rcfv_f_key = grid->getAdjRelKeyCFV(fwd_dir_idx);
            const dir_idx_t l_dir_idx = addDirIdx<true>(fwd_dir_idx, 2);
            [[maybe_unused]] const mapkey_t &rcfv_l_key = grid->getAdjRelKeyCFV(l_dir_idx); // for blocking implementations only
            const dir_idx_t r_dir_idx = addDirIdx<true>(fwd_dir_idx, 6);
            [[maybe_unused]] const mapkey_t &rcfv_r_key = grid->getAdjRelKeyCFV(r_dir_idx); // for blocking implementations only

            V2 coord = sgn_diff;
            coord[longer_dim] = 0;
            const dir_idx_t shorter_dir_idx = dirToDirIdx<V2>(coord);
            const mapkey_t &rel_short_key = grid->getAdjRelKey(shorter_dir_idx); // rel short direction

            coord[longer_dim] = sgn_diff[longer_dim];
            coord[shorter_dim] = 0;
            const dir_idx_t longer_dir_idx = dirToDirIdx<V2>(coord);
            const mapkey_t &rel_long_key = grid->getAdjRelKey(longer_dir_idx); // rel long direction

            // init at begin
            coord = begin;
            mapkey_t key = grid->coordToKey<V2>(begin);

            // init short axis
            long_t err_s = 0;
            const long_t shorter_abs_diff = abs_diff[shorter_dim];
            const long_t longer_abs_diff = abs_diff[longer_dim];

            // track longer dim
            int_t &longer_val = coord[longer_dim];
            const int_t &longer_end = end[longer_dim];
            const int_t &longer_sgn = sgn_diff[longer_dim];

            mapkey_t fcell_key = grid->addKeyToRelKey(key, rcfv_f_key);
            bool fcell_is_oc = grid->isOc(fcell_key);
            while (1)
            {
                if (fcell_is_oc)
                    return collided;

                // find change in short axis
                err_s += shorter_abs_diff;
                if (err_s == longer_abs_diff)
                { // intersects grid vertex
                    // move to short
                    key = grid->addKeyToRelKey(key, rel_short_key);

                    // (5) reset error
                    err_s = 0; // err_s -= longer_abs_diff
                }
                else if (err_s > longer_abs_diff)
                { // intersects grid edge (parallel to long axis)
                    // (1) move to vertex in short direction
                    key = grid->addKeyToRelKey(key, rel_short_key);
                    // (2) check cell in forward (long and short) direction
                    fcell_key = grid->addKeyToRelKey(key, rcfv_f_key);
                    if (grid->isOc(fcell_key))
                        return collided;
                    // (3) reset error
                    err_s -= longer_abs_diff;
                }
                // move to vertex in long direction
                key = grid->addKeyToRelKey(key, rel_long_key);
                longer_val += longer_sgn;

                // break if reached
                if (longer_val == longer_end)
                    break;

                fcell_key = grid->addKeyToRelKey(key, rcfv_f_key);
                fcell_is_oc = grid->isOc(fcell_key);

                if constexpr (is_blocking)
                {
                    // evaluate if blocked
                    if (err_s == 0 && !fcell_is_oc)
                    { // forward is free and path intersects vertex
                        // placed here to avoid evaluating at begin's coordinate ==> the cell behind the vertex is free to reach here
                        mapkey_t lcell_key = grid->addKeyToRelKey(key, rcfv_l_key);
                        mapkey_t rcell_key = grid->addKeyToRelKey(key, rcfv_r_key);
                        bool lcell_is_oc = grid->isOc(lcell_key);
                        bool rcell_is_oc = grid->isOc(rcell_key);

                        if (lcell_is_oc && rcell_is_oc)
                            return collided;
                    }
                }
            }
            return reached;
        }


        bool _castCardinal() const
        {
            // get bl and br
            mapkey_t kc_l = grid->addKeyToRelKey(ki, grid->getCellRelKey(addDirIdx(di, 3), pi.x));
            mapkey_t kc_r = grid->addKeyToRelKey(ki, grid->getCellRelKey(addDirIdx(di, 5), pi.x));
            mapkey_t rkc = grid->getRelKey<true>(di);

            const bool out_of_map_left = grid->inMap<false>(pi + grid->getRelCoord(addDirIdx(di, 2)));
            const bool out_of_map_right = grid->inMap<false>(pi + grid->getRelCoord(addDirIdx(di, 6)));
            const bool out_of_map_back = grid->inMap<false>(pi + grid->getRelCoord(addDirIdx(di, 4)));

            bool is_blocked_left = out_of_map_left || out_of_map_back || grid->isOc(kc_l);
            bool is_blocked_right = out_of_map_right || out_of_map_back || grid->isOc(kc_r);
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
