#include "P2D/debug.hpp"
namespace P2D
{
#if P2D_DEBUG
    __Debug __dbg(LOG_PATH);
#endif
}