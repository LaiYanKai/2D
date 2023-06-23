#include "P2D/P2D.hpp"
#include "corner.hpp"

namespace P2D::R2
{
    // trace from edge at (coord, mkey), in the direction di_trace along the obstacle edge.
    // After the trace reaches a corner, returns the corner's coord, mkey, di and convexity in coord, mkey, di_crn and convex respectively. 
    template <bool diag_block>
    inline bool trace(dir_idx_t &di_trace, dir_idx_t &di_crn, V2 &coord, mapkey_t &mkey, bool &convex)
    {
        
    }
}