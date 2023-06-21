#include <iostream>
#include <fstream>

#include "P2D/P2D.hpp"
#include "VG2/VG2.hpp"
#include "ANYA2/ANYA2.hpp"
#include "TS2/TS2.hpp"

int main(int argc, char *argv[])
{
    std::string alg_name = "ANYA2B";
    int scen_id = -1;

    std::vector<P2D::Scenarios> expt = P2D::getExperiment(alg_name);

    for (P2D::Scenarios &scens : expt)
    {
        P2D::Grid grid;
        P2D::getMap(grid, scens);

        if (alg_name == "ANYA2B")
        {
            P2D::ANYA2::ANYA2<true> alg(&grid);
            P2D::run(&alg, scens, scen_id);
        }
        else if (alg_name == "ANYA2N")
        {
            P2D::ANYA2::ANYA2<false> alg(&grid);
            P2D::run(&alg, scens, scen_id);
        }

        if (scen_id < 0)
            P2D::writeResults(scens);
    }
    return 0;
}