#ifndef SNOW_VIZ_DIFF_H
#define SNOW_VIZ_DIFF_H

#include <sstream>

#include "../../lib/SnowSolver.h"

#include "renderer.h"


static unsigned int startFrame;
static unsigned int endFrame;

static std::string dirA;
static std::string dirB;


static void initVizDiff(int argc, char const **argv) {

    startFrame = static_cast<unsigned int>(atoi(argv[4]));
    endFrame = static_cast<unsigned int>(atoi(argv[5]));

    // Simulation

    dirA = argv[2];
    dirB = argv[3];

    std::ostringstream filename;
    filename << "frame-" << startFrame << ".snowstate";

    solver.reset(new SnowSolver(joinPath(dirA, filename.str())));
    ghostSolver.reset(new SnowSolver(joinPath(dirB, filename.str())));

    // Rendering

    initRenderer();

}

static void vizDiffRenderLoopUpdate(unsigned int frame) {

    unsigned int wrappedFrame = startFrame + frame % (endFrame - startFrame);
    std::ostringstream filename;
    filename << "frame-" << wrappedFrame << ".snowstate";

    solver->loadState(joinPath(dirA, filename.str()));
    ghostSolver->loadState(joinPath(dirB, filename.str()));

}

static void startVizDiffLoop() {

    startRenderLoop(vizDiffRenderLoopUpdate);

}


#endif //SNOW_VIZ_DIFF_H
