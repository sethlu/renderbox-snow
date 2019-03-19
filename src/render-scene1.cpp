#ifdef USE_RENDERBOX


#include "utils/viz-render.h"
#include "scenes/scene1.h"


void launchRenderScene1(int argc, char const **argv) {
    if (argc < 5) {
        std::cout << "Usage: ./snow render-scene1 dir frame end-frame" << std::endl;
        exit(1);
    }

    initViz(argc, argv);

    renderColliders();

    startVizLoop();
}


#endif //USE_RENDERBOX
