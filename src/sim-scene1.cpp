#include "utils/sim.h"
#include "scenes/scene1.h"


void launchSimScene1(int argc, char const **argv) {
    if (argc < 4) {
        std::cout << "Usage: ./snow viz-scene1 start-frame end-frame" << std::endl;
        exit(1);
    }

    initSim(argc, argv);

    solver->handleNodeCollisionVelocityUpdate = handleNodeCollisionVelocityUpdate;

    startSimLoop();
}
