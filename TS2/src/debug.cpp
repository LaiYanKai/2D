#include "P2D/debug.hpp"
namespace P2D
{
#if P2D_VERBOSE
    __Debug __dbg(LOG_PATH);
#endif
}