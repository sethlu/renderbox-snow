#define SOLVER LavaSolver
#define SOLVER_LAVA

#include "utils/sim.h"
#include "scenes/scene2.h"


void lavaLaunchSimScene2(int argc, char const **argv) {
    if (argc < 4) {
        std::cout << "Usage: ./snow lava:sim-scene2 start-frame end-frame" << std::endl;
        exit(1);
    }

    initSim(argc, argv);

    solver->isNodeColliding = isNodeColliding;
    solver->handleNodeCollisionVelocityUpdate = handleNodeCollisionVelocityUpdate;

    startSimLoop();
}
