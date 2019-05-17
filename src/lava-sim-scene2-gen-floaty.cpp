#include <memory>
#include <sstream>

#define SOLVER LavaSolver
#define SOLVER_LAVA

#include "utils/common.h"
#include "snow/sphere.h"
#include "snow/slab.h"
#include "scenes/scene2.h"


void lavaLaunchSimScene2GenFloaty(int argc, char const **argv) {

    // Simulation consts

    double density = 1000; // kg/m3
    double particleSize = .005;
    double gridSize = particleSize * 2;

    // Init simulation

    solver.reset(new LavaSolver(gridSize, simulationSize * (1 / gridSize)));

    if (argc > 2) solver->delta_t = atof(argv[2]);

    // Particles

    genSnowSlab(glm::dvec3(simulationReservedBoundary),
                glm::dvec3(simulationSize.x - simulationReservedBoundary,
                           simulationSize.y - simulationReservedBoundary,
                           simulationSize.z * 0.2),
                density, particleSize);
    for (auto &particleNode : solver->particleNodes) {
        particleNode.temperature = 10; // Low temperature water
    }

    genSnowSphere(glm::dvec3(simulationSize.x / 2 - 0.01, simulationSize.y / 2, simulationSize.z * 0.275),
                  0.03, density, particleSize);
    genSnowSphere(glm::dvec3(simulationSize.x / 2 + 0.01, simulationSize.y / 2, simulationSize.z * (0.275 + 0.125)),
                  0.03, density, particleSize);
    genSnowSphere(glm::dvec3(simulationSize.x / 2 - 0.01, simulationSize.y / 2, simulationSize.z * (0.275 + 2 * 0.125)),
                  0.03, density, particleSize);
    genSnowSphere(glm::dvec3(simulationSize.x / 2 + 0.01, simulationSize.y / 2, simulationSize.z * (0.275 + 3 * 0.125)),
                  0.03, density, particleSize);

    std::cout << "#particles=" << solver->particleNodes.size() << std::endl;

    // Output

    std::ostringstream filename;
    filename << "frame-0" SOLVER_STATE_EXT;
    solver->saveState(filename.str());

    std::cout << "Frame 0 written to: " << filename.str() << std::endl;

}
