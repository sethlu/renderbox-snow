#ifdef USE_RENDERBOX

#include <memory>

#define RENDERBOX_USE_OPENGL
#define RENDERBOX_USE_GLFW

#include "renderbox.h"

#define SOLVER LavaSolver
#define SOLVER_LAVA

#include "../lib/LavaSolver.h"
#include "scenes/scene2.h"
#include "snow/sphere.h"
#include "snow/slab.h"
#include "utils/renderer.h"


static unsigned ticksPerFrame = 1;


static void demoRenderLoopUpdate(unsigned int frame) {

    for (auto tick = 0; tick < ticksPerFrame; tick++) {
        solver->update();
    }

}

void lavaLaunchDemoFloaty(int argc, char const **argv) {

    // Simulation

    double density = 1000; // kg/m3
    double particleSize = .005;
    double gridSize = particleSize * 4;

    solver.reset(new LavaSolver(gridSize, simulationSize * (1 / gridSize)));
    solver->delta_t = 5e-4;

    solver->isNodeColliding = isNodeColliding;
    solver->handleNodeCollisionVelocityUpdate = handleNodeCollisionVelocityUpdate;

    genSnowSlab(glm::dvec3(simulationReservedBoundary),
                glm::dvec3(simulationSize.x - simulationReservedBoundary,
                           simulationSize.y - simulationReservedBoundary,
                           simulationSize.z * 0.2),
                density, particleSize);
    for (auto &particleNode : solver->particleNodes) {
        particleNode.temperature = 10; // Low temperature water
    }

    genSnowSphere(glm::dvec3(simulationSize.x / 2, simulationSize.y / 2, simulationSize.z * 3 / 4), 0.02, density,
                  particleSize);

    // Rendering

    // Override camera settings
    cameraDistance = 0.5;

    // Override geometry
    snowParticleGeometry = std::make_shared<renderbox::SphereGeometry>(.005 / 4);

    // Override materials
    lavaParticleLiquidMaterial = std::make_shared<renderbox::MeshLambertMaterial>(
            renderbox::vec3(8 / 255.f, 90 / 255.f, 140 / 255.f),
            renderbox::vec3(8 / 255.f, 90 / 255.f, 140 / 255.f));
    lavaParticlePhaseChangeMaterial = std::make_shared<renderbox::MeshLambertMaterial>(
            renderbox::vec3(48 / 255.f, 186 / 255.f, 217 / 255.f),
            renderbox::vec3(48 / 255.f, 186 / 255.f, 217 / 255.f));

    initRenderer();

    renderColliders();

    startRenderLoop(demoRenderLoopUpdate);

}

#endif //USE_RENDERBOX
