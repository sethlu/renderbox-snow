#include <cmath>

#include "../../lib/SnowSolver.h"
#include "../utils/common.h"


static void genSnowSphere(glm::dvec3 position, double radius, double density, double particleSize) {
    auto simulationSize = solver->h * glm::dvec3(solver->size);

    double volume = 4.0 / 3 * M_PI * pow(radius, 3);
    auto totalNumParticles = static_cast<unsigned int>(volume / pow(particleSize, 3));
    unsigned int numParticles = 0;

    while (numParticles < totalNumParticles) {
        auto guess = glm::dvec3(
                randNumber(0, simulationSize.x),
                randNumber(0, simulationSize.y),
                randNumber(0, simulationSize.z));
        auto mass = density * pow(particleSize, 3);

        if (glm::length(guess - position) <= radius) {

            solver->particleNodes.emplace_back(guess, mass);
            if (ghostSolver) ghostSolver->particleNodes.emplace_back(guess, mass);

            numParticles++;
        }
    };
}
