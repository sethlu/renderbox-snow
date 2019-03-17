#include "utils/sim.h"
#include "scenes/scene0.h"


void launchSimScene0(int argc, char const **argv) {
    if (argc < 4) {
        std::cout << "Usage: ./snow viz-scene0 start-frame end-frame" << std::endl;
        exit(1);
    }

    initSim(argc, argv);

    solver->handleNodeCollisionVelocityUpdate = handleNodeCollisionVelocityUpdate;

    startSimLoop();
}
