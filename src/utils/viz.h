#ifndef SNOW_VIZ_H
#define SNOW_VIZ_H

#include <sstream>

#include "../../lib/SnowSolver.h"

#include "renderer.h"


static unsigned int startFrame;
static unsigned int endFrame;


static void initViz(int argc, char const **argv) {

    startFrame = static_cast<unsigned int>(atoi(argv[2]));
    endFrame = static_cast<unsigned int>(atoi(argv[3]));

    // Simulation

    std::ostringstream filename;
    filename << "frame-" << startFrame << ".snowstate";

    solver.reset(new SnowSolver(filename.str()));

    // Rendering

    initRenderer();

}

static void vizRenderLoopUpdate(unsigned int frame) {

    unsigned int wrappedFrame = startFrame + frame % (endFrame - startFrame);
    std::ostringstream filename;
    filename << "frame-" << wrappedFrame << ".snowstate";
    solver->loadState(filename.str());

}

static void startVizLoop() {

    startRenderLoop(vizRenderLoopUpdate);

}


#endif //SNOW_VIZ_H
