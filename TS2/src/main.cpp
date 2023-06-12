#include <filesystem>
#include "P2D/P2D.hpp"
#include "P2D/debug.hpp"
#include "TS2/TS2.hpp"

#define DIAG_BLOCK true
#define SCEN_NUM 0

int main(int, char **)
{
    std::vector<P2D::Scenarios> expt = P2D::getExperiment(DIAG_BLOCK ? "TS2B" : "TS2N");

    for (P2D::Scenarios &scens : expt)
    {
        std::filesystem::path fp_vg = "TS2/combinations/";
        fp_vg = fp_vg / scens.fp_dir / scens.fp_name;
        fp_vg.replace_extension(".combinations" + std::string(DIAG_BLOCK ? "B" : "N"));

        P2D::Grid grid;
        P2D::getMap(grid, scens);
        P2D::TS2::TS2<DIAG_BLOCK> TS2(&grid, fp_vg);

        P2D::run(&TS2, scens, SCEN_NUM);

        if (SCEN_NUM == 0)
            P2D::writeResults(scens);
    }
    return 0;
}