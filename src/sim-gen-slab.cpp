#include <memory>
#include <sstream>

#include "../lib/SnowSolver.h"
#include "utils/common.h"
#include "snow/slab.h"


void launchSimGenSlab(int argc, char const **argv) {

    // Simulation consts

    double density = 60; // kg/m3
    double particleSize = .0072;
    double gridSize = particleSize * 2;
    auto simulationSize = glm::dvec3(1);

    // Init simulation

    solver.reset(new SnowSolver(gridSize, simulationSize * (1 / gridSize)));
    solver->delta_t = 5e-4;

    if (argc > 2) {
        solver->beta = atof(argv[2]);
    }

    // Particles

    genSnowSlab(glm::dvec3(0.25, 0.45, 0.7), glm::dvec3(0.75, 0.55, 0.85), density, particleSize);

    std::cout << "#particles=" << solver->particleNodes.size() << std::endl;

    // Output

    std::ostringstream filename;
    filename << "frame-0.snowstate";
    solver->saveState(filename.str());

    std::cout << "Frame 0 written to: " << filename.str() << std::endl;

}
