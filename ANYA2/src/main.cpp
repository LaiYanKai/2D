#include <filesystem>
#include "P2D/P2D.hpp"
#include "P2D/debug.hpp"
#include "ANYA2/ANYA2.hpp"

#define DIAG_BLOCK true
#define SCEN_NUM 800

int main(int, char **)
{
    std::vector<P2D::Scenarios> expt = P2D::getExperiment(DIAG_BLOCK ? "ANYA2B" : "ANYA2N");

    for (P2D::Scenarios &scens : expt)
    {
        P2D::Grid grid;
        P2D::getMap(grid, scens);
        P2D::ANYA2::ANYA2<DIAG_BLOCK> ANYA2(&grid);

        P2D::run(&ANYA2, scens, SCEN_NUM);

        if (SCEN_NUM == 0)
            P2D::writeResults(scens);
    }
    return 0;
}