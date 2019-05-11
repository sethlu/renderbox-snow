#include <memory>
#include <sstream>

#include "utils/common.h"
#include "snow/sphere.h"
#include "snow/slab.h"


void launchSimGenSnowman(int argc, char const **argv) {

    // Simulation consts

    double density = 800; // kg/m3
    double particleSize = .0072;
    double gridSize = particleSize * 2;
    auto simulationSize = glm::dvec3(1);

    // Init simulation

    solver.reset(new SnowSolver(gridSize, simulationSize * (1 / gridSize)));

    if (argc > 2) solver->delta_t = atof(argv[2]);
    if (argc > 3) solver->beta = atof(argv[3]);

    // Particles

    auto r1 = 0.2 / 2;
    auto r2 = 0.125 / 2;
    auto r3 = 0.075 / 2;
    auto overlap = 0.05;

    auto c1 = 0.1 + r1;
    auto c2 = c1 + r1 - overlap + r2;
    auto c3 = c2 + r2 - overlap + r3;

    genSnowSlab(glm::dvec3(0.05, 0.05, 0.075), glm::dvec3(simulationSize.x - 0.05, simulationSize.y - 0.05, 0.125),
                density, particleSize);

    genSnowSphere(glm::dvec3(0.5, 0.5, c1), r1, density, particleSize);
    genSnowSphere(glm::dvec3(0.5, 0.5, c2), r2, density, particleSize);
    genSnowSphere(glm::dvec3(0.5, 0.5, c3), r3, density, particleSize);

    std::cout << "#particles=" << solver->particleNodes.size() << std::endl;

    // Output

    std::ostringstream filename;
    filename << "frame-0" SOLVER_STATE_EXT;
    solver->saveState(filename.str());

    std::cout << "Frame 0 written to: " << filename.str() << std::endl;

}
