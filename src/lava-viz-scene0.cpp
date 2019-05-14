#ifdef USE_RENDERBOX


#define SOLVER LavaSolver
#define SOLVER_LAVA

#include "scenes/scene0.h"
#include "utils/viz.h"


void lavaLaunchVizScene0(int argc, char const **argv) {
    if (argc < 5) {
        std::cout << "Usage: ./snow lava:viz-scene0 dir frame end-frame" << std::endl;
        exit(1);
    }

    initViz(argc, argv);

    renderColliders();

    startVizLoop();
}


#endif //USE_RENDERBOX
