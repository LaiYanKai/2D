#include "types.hpp"
#include "unit.hpp"
#include "traceinfo.hpp"
#include <assert.h>

#pragma once
namespace P2D::R2
{
    struct TraceRev
    {
        Unit *u;
        Ray *ray_left;
        Ray *ray_right;
        TraceRev(Unit *const &u, Ray *const &ray_left, Ray *const &ray_right) : u(u), ray_left(ray_left), ray_right(ray_right) {}
    };
    class TraceStatus
    {
    public:
        TraceInfoContainer srcs, tgts;
        std::vector<TraceRev> revs;
        V2 v_cur, v_prev;
        Ray *ray_left = nullptr, *ray_right = nullptr;
        Corner *crn_cur;
        int num_points = 0;
        Side side;
        dir_idx_t di;
        bool refound_mnr_src = false;
        bool all_progressed = false;

        TraceStatus(const Side &side_traced, Corner *const &crn_cur, Ray *const &ray_left, Ray *const &ray_right)
            : v_cur(crn_cur->edgeVec(side_traced)), v_prev(-crn_cur->edgeVec(!side_traced)),
              ray_left(ray_left), ray_right(ray_right),
              crn_cur(crn_cur), side(side_traced), di(crn_cur->edgeDi(side_traced))

        {
            assert(crn_cur != nullptr);
        }
        ~TraceStatus() {}

        inline Ray *const &ray(const Side &side) const { return side == Side::L ? ray_left : ray_right; }
        inline Ray *&ray(const Side &side) { return side == Side::L ? ray_left : ray_right; }

        // setting up src for trace. assumes u_src is progressed.
        void initSrcWithProg(Unit *const &u_src, const V2 &v_prog)
        {
            assert(u_src->isS() || u_src->isE());
            this->srcs.emplace(ListDir::Back, u_src, V2(0, 0), v_prog);
            _dbg11("[TiInitSrc] TiSrc{ " << this->srcs.front() << " }");
        }

        // useful for init after collision. assumes u_tgt is progressed.
        void initTgtWithProg(Unit *const &u_tgt, const V2 &v_prog)
        {
            assert(u_tgt != nullptr);
            assert(u_tgt->isPS() || u_tgt->isT());
            assert(u_tgt->hasSrc() == false);
            this->tgts.emplace(ListDir::Back, u_tgt, V2(0, 0), v_prog);

            // remove rays
            u_tgt->ray_left = nullptr;
            u_tgt->ray_right = nullptr;
            _dbg11("[TiInitTgt] TiTgt(rays removed){ " << this->tgts.back() << " }");
        }

        inline void initTgtWithCoord(Unit *const &u_tgt, const V2 &initial_coord) { initTgtWithProg(u_tgt, initial_coord - u_tgt->coord()); }

        // useful for setting up trace after polling. assumes all are progressed
        void initTgtsWithCoord(const std::vector<Unit *> &u_tgts, const V2 &initial_coord)
        {
            for (Unit *const &u_tgt : u_tgts)
            {
                if (u_tgt == nullptr)
                    continue;
                initTgtWithCoord(u_tgt, initial_coord);
            }
        }

        inline std::string repr() const
        {
            std::stringstream ss;
            ss << "[T:" << this->side << "]";
            return ss.str();
        }

        friend std::ostream &operator<<(std::ostream &out, TraceStatus const &status)
        {
            out << "<";
            out << status.side;
            out << int(status.di);
            out << status.crn_cur;
            out << ">";
            out << ", v(" << status.v_prev << ";" << status.v_cur << ")";
            out << ", rays(";
            out << (status.ray_left == nullptr ? "<     NA    >" : status.ray_left->repr(0));
            out << (status.ray_right == nullptr ? "<     NA    >" : status.ray_right->repr(0));
            out << ")";
            out << ", pts[" << status.num_points << "]";
            out << ", srcs[" << status.srcs.size() << "]";
            out << ", tgts[" << status.srcs.size() << "]";
            out << ", revs[" << status.revs.size() << "]";
            return out;
        }
    };
}
