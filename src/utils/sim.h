#ifndef SNOW_SIM_H
#define SNOW_SIM_H


#include <memory>
#include <sstream>
#include <chrono>

#include "common.h"


static unsigned int fps = 60;
static unsigned int timedFrames;
static unsigned int totalFrames;


static void initSim(int argc, char const **argv) {

    timedFrames = static_cast<unsigned int>(std::stoi(argv[2]));
    totalFrames = static_cast<unsigned int>(std::stoi(argv[3]));

    // Simulation

    std::ostringstream filename;
    filename << "frame-" << timedFrames << SOLVER_STATE_EXT;
    solver.reset(new SOLVER(filename.str()));

}

static void startSimLoop() {

    // Render loop

    while (timedFrames + 1 < totalFrames) {

        std::cout << "tick=" << solver->getTick() << " time=" << solver->getTime() << std::endl;

        auto timeLast = std::chrono::system_clock::now();
        solver->update(); // Run simulation
        auto timeNow = std::chrono::system_clock::now();
        auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(timeNow - timeLast);
        std::cout << ms.count() << "ms" << std::endl;

        if (solver->getTime() > 1.0 * (timedFrames + 1) / fps) {
            timedFrames++;

            std::ostringstream filename;
            filename << "frame-" << timedFrames << SOLVER_STATE_EXT;
            solver->saveState(filename.str());

            std::cout << "Frame " << timedFrames << " written to: " << filename.str() << std::endl;
        }

    }

}


#endif //SNOW_SIM_H
