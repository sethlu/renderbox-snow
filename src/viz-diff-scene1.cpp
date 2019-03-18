#ifdef USE_RENDERBOX

#include "utils/viz-diff.h"
#include "scenes/scene1.h"


void launchVizDiffScene1(int argc, char const **argv) {
    if (argc < 6) {
        std::cout << "Usage: ./snow viz-diff-1scene0 dir-a dir-b frame end-frame" << std::endl;
        exit(1);
    }

    initVizDiff(argc, argv);

    renderColliders();

    startVizDiffLoop();
}


#endif //USE_RENDERBOX
