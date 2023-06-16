#include <filesystem>
#include "P2D/P2D.hpp"
#include "P2D/debug.hpp"
#include "VG2/VG2.hpp"

#define DIAG_BLOCK false
#define SCEN_NUM 0

int main(int, char **)
{
    std::vector<P2D::Scenarios> expt = P2D::getExperiment(DIAG_BLOCK ? "VG2B" : "VG2N");

    for (P2D::Scenarios &scens : expt)
    {
        std::filesystem::path fp_vg = "VG2/combinations/";
        fp_vg = fp_vg / scens.fp_dir / scens.fp_name;
        fp_vg.replace_extension(".combinations" + std::string(DIAG_BLOCK ? "B" : "N"));

        P2D::Grid grid;
        P2D::getMap(grid, scens);
        P2D::VG2::VG2<DIAG_BLOCK> vg2(&grid, fp_vg);

        P2D::run(&vg2, scens, SCEN_NUM);

        if (SCEN_NUM == 0)
            P2D::writeResults(scens);
    }
    return 0;
}