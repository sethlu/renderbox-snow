#ifdef USE_RENDERBOX


#define SOLVER LavaSolver
#define SOLVER_LAVA

#include "utils/viz-render.h"
#include "scenes/scene2.h"


void lavaLaunchRenderScene2(int argc, char const **argv) {
    if (argc < 5) {
        std::cout << "Usage: ./snow render-scene2 dir frame end-frame" << std::endl;
        exit(1);
    }

    // Override camera settings
    cameraDistance = 0.8;

    // Override geometry
    snowParticleGeometry = std::make_shared<renderbox::SphereGeometry>(.005 / 4);

    // Override materials
    lavaParticleLiquidMaterial = std::make_shared<renderbox::MeshLambertMaterial>(
            renderbox::vec3(8 / 255.f, 90 / 255.f, 140 / 255.f),
            renderbox::vec3(8 / 255.f, 90 / 255.f, 140 / 255.f));
    lavaParticlePhaseChangeMaterial = std::make_shared<renderbox::MeshLambertMaterial>(
            renderbox::vec3(48 / 255.f, 186 / 255.f, 217 / 255.f),
            renderbox::vec3(48 / 255.f, 186 / 255.f, 217 / 255.f));

    initViz(argc, argv);

    renderColliders();

    startVizLoop();
}


#endif //USE_RENDERBOX
