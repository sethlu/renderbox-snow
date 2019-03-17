#ifdef USE_RENDERBOX

#include <memory>

#define RENDERBOX_USE_OPENGL
#define RENDERBOX_USE_GLFW

#include "renderbox.h"

#include "../lib/SnowSolver.h"
#include "utils/renderer.h"
#include "scenes/scene1.h"
#include "snow/slab.h"


static unsigned ticksPerFrame = 1;


static void demoRenderLoopUpdate(unsigned int frame) {

    for (auto tick = 0; tick < ticksPerFrame; tick++) {
        solver->update();
    }

}

void launchDemoSlabOverWedge(int argc, char const **argv) {

    // Simulation

    double density = 60; // kg/m3
    double particleSize = .01;
    double gridSize = particleSize * 2;
    auto simulationSize = glm::dvec3(1);

    solver.reset(new SnowSolver(gridSize, simulationSize * (1 / gridSize)));

    genSnowSlab(glm::dvec3(0.25, 0.45, 0.7), glm::dvec3(0.75, 0.55, 0.85), density, particleSize);

    solver->handleNodeCollisionVelocityUpdate = handleNodeCollisionVelocityUpdate;

    // Rendering

    initRenderer();

    renderColliders();

    startRenderLoop(demoRenderLoopUpdate);

}

#endif //USE_RENDERBOX