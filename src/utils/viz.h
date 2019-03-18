#ifndef SNOW_VIZ_H
#define SNOW_VIZ_H

#include <sstream>

#include "../../lib/SnowSolver.h"

#include "renderer.h"


static unsigned int startFrame;
static unsigned int endFrame;

static std::string dir;


static void initViz(int argc, char const **argv) {

    startFrame = static_cast<unsigned int>(atoi(argv[3]));
    endFrame = static_cast<unsigned int>(atoi(argv[4]));

    // Simulation

    dir = argv[2];

    std::ostringstream filename;
    filename << "frame-" << startFrame << ".snowstate";

    solver.reset(new SnowSolver(joinPath(dir, filename.str())));

    // Rendering

    initRenderer();

}

static void vizRenderLoopUpdate(unsigned int frame) {

    unsigned int wrappedFrame = startFrame + frame % (endFrame - startFrame);
    std::ostringstream filename;
    filename << "frame-" << wrappedFrame << ".snowstate";
    solver->loadState(joinPath(dir, filename.str()));

}

static void startVizLoop() {

    startRenderLoop(vizRenderLoopUpdate);

}


#endif //SNOW_VIZ_H
