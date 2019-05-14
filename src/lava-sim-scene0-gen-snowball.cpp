#include <memory>
#include <sstream>

#define SOLVER LavaSolver
#define SOLVER_LAVA

#include "utils/common.h"
#include "snow/sphere.h"
#include "scenes/scene0.h"


void lavaLaunchSimScene0GenSnowball(int argc, char const **argv) {

    // Simulation consts

    double density = 1000; // kg/m3
    double particleSize = .005;
    double gridSize = particleSize * 2;

    // Init simulation

    solver.reset(new LavaSolver(gridSize, simulationSize * (1 / gridSize)));

    if (argc > 2) solver->delta_t = atof(argv[2]);

    // Particles

    genSnowSphere(glm::dvec3(simulationSize.x / 2, simulationSize.y / 2, simulationSize.z / 2),
                  0.025, density, particleSize);

    std::cout << "#particles=" << solver->particleNodes.size() << std::endl;

    // Output

    std::ostringstream filename;
    filename << "frame-0" SOLVER_STATE_EXT;
    solver->saveState(filename.str());

    std::cout << "Frame 0 written to: " << filename.str() << std::endl;

}
