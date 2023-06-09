#include "RR2star/math.hpp"
#include "RR2star/scenarios.hpp"
#include "RR2star/Vec2.hpp"
#include <iostream>

int main(int, char **)
{
    RR2star::Scenarios scens("Test", "results");
    std::cout << scens.fp_dir << scens.fp_map << scens.fp_name << scens.fp_results << scens.fp_scen << std::endl;
    return 0;
}
