#include "P2D/P2D.hpp"
#include "corner.hpp"

namespace P2D::R2
{
    template <bool is_RR2E>
    class R2
    {
    private:
        Grid *const grid;
        LosAdvanced los;

        

    public:
        R2(Grid *const &grid) : grid(grid), los(grid) {}
        R2 &operator=(const R2 &) = delete; // Disallow copying
        R2(const R2 &) = delete;
        ~R2() {}

        std::vector<V2> run(const V2 &p_start, const V2 &p_goal)
        {
        }
    };
}