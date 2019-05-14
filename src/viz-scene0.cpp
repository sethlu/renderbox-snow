#ifdef USE_RENDERBOX


#include "scenes/scene0.h"
#include "utils/viz.h"


void launchVizScene0(int argc, char const **argv) {
    if (argc < 5) {
        std::cout << "Usage: ./snow viz-scene0 dir frame end-frame" << std::endl;
        exit(1);
    }

    initViz(argc, argv);

    renderColliders();

    startVizLoop();
}


#endif //USE_RENDERBOX
