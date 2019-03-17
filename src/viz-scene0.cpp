#ifdef USE_RENDERBOX

#include "utils/viz.h"
#include "scenes/scene0.h"


void launchVizScene0(int argc, char const **argv) {
    if (argc < 4) {
        std::cout << "Usage: ./snow sim-scene0 frame end-frame" << std::endl;
        exit(1);
    }

    initViz(argc, argv);

    renderColliders();

    startVizLoop();
}


#endif //USE_RENDERBOX
