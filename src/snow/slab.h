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

    auto particleMass = density * pow(particleSize, 3);

    while (numParticles < totalNumParticles) {
        auto particlePosition = glm::dvec3(
                randNumber(corner1.x, corner2.x),
                randNumber(corner1.y, corner2.y),
                randNumber(corner1.z, corner2.z));

        solver->particleNodes.emplace_back(particlePosition, particleMass);
        if (ghostSolver) ghostSolver->particleNodes.emplace_back(particlePosition, particleMass);

        numParticles++;
    }

}
