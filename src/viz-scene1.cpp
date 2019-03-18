#ifdef USE_RENDERBOX

#include "utils/viz.h"
#include "scenes/scene1.h"


void launchVizScene1(int argc, char const **argv) {
    if (argc < 5) {
        std::cout << "Usage: ./snow sim-scene1 dir frame end-frame" << std::endl;
        exit(1);
    }

    initViz(argc, argv);

    renderColliders();

    startVizLoop();
}


#endif //USE_RENDERBOX
