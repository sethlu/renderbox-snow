#ifdef USE_RENDERBOX


#define RENDERBOX_USE_OPENGL
#define RENDERBOX_USE_GLFW

#include "renderbox.h"

#include "utils/renderer.h"
#include "scenes/scene0.h"
#include "snow/sphere.h"
#include "snow/slab.h"


static unsigned ticksPerFrame = 1;


static void demoRenderLoopUpdate(unsigned int frame) {

    for (auto tick = 0; tick < ticksPerFrame; tick++) {
        solver->update();
    }

}

void launchDemoSnowman(int argc, char const **argv) {

    // Simulation

    double density = 400; // kg/m3
    double particleSize = .0072;
    double gridSize = particleSize * 2;
    auto simulationSize = glm::dvec3(1);

    solver.reset(new SnowSolver(gridSize, simulationSize * (1 / gridSize)));
    solver->delta_t = 5e-4; // Smaller time step so simulation doesn't blow up

    auto r1 = 0.2 / 2;
    auto r2 = 0.125 / 2;
    auto r3 = 0.075 / 2;
    auto overlap = 0.025;

    auto c1 = 0.1 + r1;
    auto c2 = c1 + r1 - overlap + r2;
    auto c3 = c2 + r2 - overlap + r3;

    genSnowSphere(glm::dvec3(0.5, 0.5, c1), r1, density, particleSize);
    genSnowSphere(glm::dvec3(0.5, 0.5, c2), r2, density, particleSize);
    genSnowSphere(glm::dvec3(0.5, 0.5, c3), r3, density, particleSize);

    solver->handleNodeCollisionVelocityUpdate = handleNodeCollisionVelocityUpdate;

    // Rendering

    initRenderer();

    renderColliders();

    startRenderLoop(demoRenderLoopUpdate);

}


#endif //USE_RENDERBOX
