#include <cmath>

#include "../../lib/SnowSolver.h"
#include "../utils/common.h"


static void genSnowSlab(glm::dvec3 corner1, glm::dvec3 corner2, double density, double particleSize) {
    auto simulationSize = solver->h * glm::dvec3(solver->size);

    double volume = std::abs(corner2.x - corner1.x) *
                    std::abs(corner2.y - corner1.y) *
                    std::abs(corner2.z - corner1.z);
    auto totalNumParticles = static_cast<unsigned int>(volume / pow(particleSize, 3));
    unsigned int numParticles = 0;

    while (numParticles < totalNumParticles) {
        auto position = glm::dvec3(
                randNumber(0, simulationSize.x),
                randNumber(0, simulationSize.y),
                randNumber(0, simulationSize.z));
        auto mass = density * powf(particleSize, 3);

        if (position.x >= corner1.x &&
            position.y >= corner1.y &&
            position.z >= corner1.z &&
            position.x <= corner2.x &&
            position.y <= corner2.y &&
            position.z <= corner2.z) {

            solver->particleNodes.emplace_back(position, mass);

            numParticles++;
        }
    }

}
