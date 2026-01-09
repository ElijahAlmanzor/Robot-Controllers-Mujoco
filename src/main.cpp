#include "mj_sim.hpp"

#include <exception>
#include <iostream>

int main(int argc, char** argv)
{
    try
    {
        MJSim sim(argc, argv);
        sim.control_loop_run();
    }
    catch (const std::exception& e)
    {
        std::cerr << "Fatal error: " << e.what() << "\n";
        return 1;
    }

    return 0;
}
