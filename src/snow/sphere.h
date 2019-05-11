#include <cmath>

#include "../utils/common.h"


static void genSnowSphere(glm::dvec3 position, double radius, double density, double particleSize) {
    auto simulationSize = solver->h * glm::dvec3(solver->size);

    double volume = 4.0 / 3 * M_PI * pow(radius, 3);
    auto totalNumParticles = static_cast<unsigned int>(volume / pow(particleSize, 3));
    unsigned int numParticles = 0;

    auto particleMass = density * pow(particleSize, 3);

    while (numParticles < totalNumParticles) {
        auto guess = glm::dvec3(
                randNumber(position.x - radius, position.x + radius),
                randNumber(position.y - radius, position.y + radius),
                randNumber(position.z - radius, position.z + radius));

        if (glm::length(guess - position) <= radius) {

            solver->particleNodes.emplace_back(guess, particleMass);
            if (ghostSolver) ghostSolver->particleNodes.emplace_back(guess, particleMass);

            numParticles++;
        }
    }
}
