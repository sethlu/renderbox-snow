#include <memory>
#include <sstream>

#include "utils/common.h"
#include "snow/sphere.h"


void launchSimGenSnowball(int argc, char const **argv) {

    // Simulation consts

    double density = 400; // kg/m3
    double particleSize = .0072;
    double gridSize = particleSize * 2;
    auto simulationSize = glm::dvec3(1);

    // Init simulation

    solver.reset(new SnowSolver(gridSize, simulationSize * (1 / gridSize)));

    if (argc > 2) solver->delta_t = atof(argv[2]);
    if (argc > 3) solver->beta = atof(argv[3]);

    // Particles

    genSnowSphere(glm::dvec3(0.5, 0.5, 0.5), 0.03, density, particleSize);

    std::cout << "#particles=" << solver->particleNodes.size() << std::endl;

    // Output

    std::ostringstream filename;
    filename << "frame-0" SOLVER_STATE_EXT;
    solver->saveState(filename.str());

    std::cout << "Frame 0 written to: " << filename.str() << std::endl;

}
