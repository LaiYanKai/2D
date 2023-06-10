#include <iostream>
#include <fstream>
#include <sstream>
#include <assert.h>

#pragma once
#define LOG_PATH "dbg.log"

namespace P2D
{
#ifdef P2D_DEBUG
    struct __Debug
    {
        std::ofstream log;
        std::string log_path;
        int tabs;

        __Debug(const std::string &log_path) : log(log_path), log_path(log_path), tabs(0) {}
        void inc() { ++tabs; }
        void dec() { --tabs; }
        void reset() { log = std::ofstream(log_path); }

        std::string printTabs()
        {
            std::ostringstream ss;
            for (int i = 0; i < tabs; ++i)
                ss << "\u2506 ";
            return ss.str();
        }
    };
    extern __Debug __dbg;
#define _dbgreset P2D::__dbg.reset()
#define _dbginc ++P2D::__dbg.tabs
#define _dbgdec                  \
    do                           \
    {                            \
        --P2D::__dbg.tabs;            \
        assert(P2D::__dbg.tabs >= 0); \
    } while (0)
#define __dbgw(x)   \
    std::cout << x; \
    P2D::__dbg.log << x;
#define _dbg11(x)                                   \
    do                                              \
    {                                               \
        __dbgw(P2D::__dbg.printTabs() << x << std::endl) \
    } while (0)
#define _dbg01(x)              \
    do                         \
    {                          \
        __dbgw(x << std::endl) \
    } while (0)
#define _dbg10(x)                      \
    do                                 \
    {                                  \
        __dbgw(P2D::__dbg.printTabs() << x) \
    } while (0)
#define _dbg00(x) \
    do            \
    {             \
        __dbgw(x) \
    } while (0)
#define _dbgtitle(x)                                                                                                    \
    do                                                                                                                  \
    {                                                                                                                   \
        __dbgw(P2D::__dbg.printTabs() << "\u250c\u2504\u2504 " << x << " \u2504\u2504\u2504\u2504\u2504\u2504" << std::endl) \
    } while (0)
#define _dbgtitleheavy(x)                                                                                               \
    do                                                                                                                  \
    {                                                                                                                   \
        __dbgw(P2D::__dbg.printTabs() << "\u250f\u2505\u2505 " << x << " \u2505\u2505\u2505\u2505\u2505\u2505" << std::endl) \
    } while (0)
#define _dbghelp std::cout << "help" << std::endl
#else
#define __dbgnth ((void)0)
#define _dbgreset  __dbgnth
#define _dbginc __dbgnth
#define _dbgdec __dbgnth
#define _dbg11(x) __dbgnth
#define _dbg01(x) __dbgnth
#define _dbg10(x) __dbgnth
#define _dbg00(x) __dbgnth
#define _dbgtitle(x) __dbgnth
#define _dbgtitleheavy(x) __dbgnth
#define _dbghelp __dbgnth
#endif
}
