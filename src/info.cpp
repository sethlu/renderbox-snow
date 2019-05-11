#include "utils/common.h"


void launchInfo(int argc, char const **argv) {
    if (argc < 3) {
        std::cout << "Usage: ./snow info snowstate" << std::endl;
        exit(1);
    }

    SnowSolver snowSolver{argv[2]};

    std::cout << std::endl << "Physical parameters" << std::endl
              << "Young's modulus = " << snowSolver.youngsModulus0 << std::endl
              << "Critical compression = " << snowSolver.criticalCompression << std::endl
              << "Critical stretch = " << snowSolver.criticalStretch << std::endl
              << "Hardening coefficient = " << snowSolver.hardeningCoefficient << std::endl
              << std::endl << "Simulation parameters" << std::endl
              << "PIC/FLIP = " << snowSolver.alpha << std::endl
              << "Integration = " << snowSolver.beta << std::endl
              << std::endl << "Grid" << std::endl
              << "Grid node size = " << snowSolver.h << std::endl
              << "Grid dimensions = " << snowSolver.size << std::endl
              << std::endl << "Particles" << std::endl
              << "#particles = " << snowSolver.particleNodes.size() << std::endl
              << std::endl << "Time" << std::endl
              << "Tick = " << snowSolver.tick << std::endl
              << "Time step = " << snowSolver.delta_t << std::endl
              << "Time = " << snowSolver.tick * snowSolver.delta_t << std::endl
              << std::endl;
}
