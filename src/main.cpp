#include "RR2star/math.hpp"
#include "RR2star/scenarios.hpp"
#include "RR2star/Vec2.hpp"
#include <iostream>

int main(int, char **)
{
    const RR2star::Side side = RR2star::Side::L;
    const RR2star::V2 vec(1,2), vecb(3,4);
    std::cout << side << std::endl;
    std::cout << vec + vecb << std::endl;

    return 0;
}
