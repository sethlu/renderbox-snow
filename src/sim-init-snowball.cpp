#include <memory>
#include <sstream>

#include "../lib/SnowSolver.h"


inline double randNumber(double lo, double hi) {
    return lo + rand() / (RAND_MAX / (hi - lo));
}

void launchSimInitSnowball(int argc, char const **argv) {

    // Simulation consts

    double density = 400; // kg/m3
    double particleSize = .0072;
    double gridSize = particleSize * 2;
    auto simulationSize = glm::dvec3(1);

    // Init simulation

    SnowSolver solver(gridSize, simulationSize * (1 / gridSize));
    solver.delta_t = 1e-5;

    // Particles

    unsigned int numParticles = 0;

    {
        double radius = 0.03;
        double volume = 4.0 / 3 * M_PI * pow(radius, 3);
        unsigned int totalNumParticles = numParticles + static_cast<unsigned int>(volume / pow(particleSize, 3));
        while (numParticles < totalNumParticles) {
            auto position = glm::dvec3(
                    randNumber(0, simulationSize.x),
                    randNumber(0, simulationSize.y),
                    randNumber(0, simulationSize.z));
            auto mass = density * pow(particleSize, 3);

            if (glm::length(position - glm::dvec3(0.5, 0.5, 0.5)) <= radius) {

                solver.particleNodes.emplace_back(position, mass);

                numParticles++;
            }
        }
    }

    LOG(INFO) << "#particles=" << numParticles << std::endl;

    std::ostringstream filename;
    filename << "frame-0.snowstate";
    solver.saveState(filename.str());

    std::cout << "Frame 0 written to: " << filename.str() << std::endl;

}
