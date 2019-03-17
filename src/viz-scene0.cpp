#ifdef USE_RENDERBOX

#include <memory>
#include <sstream>

#define RENDERBOX_USE_OPENGL
#define RENDERBOX_USE_GLFW

#include "renderbox.h"

#include "../lib/SnowSolver.h"


static std::unique_ptr<renderbox::OpenGLRenderer> renderer;
static std::unique_ptr<renderbox::GLFWOpenGLRenderTarget> renderTarget;

static std::shared_ptr<renderbox::Scene> scene;

static std::shared_ptr<renderbox::PerspectiveCamera> camera;
static std::shared_ptr<renderbox::Object> cameraRig;

static double cameraDistance = 2;
static double cameraAngle[] = {0.0, 90.0};

static void windowSizeCallback(GLFWwindow *window, int width, int height) {
    camera->setPerspective(glm::radians(45.0f),
                           (float) renderTarget->getWindowWidth() / (float) renderTarget->getWindowHeight());
}

void launchVizScene0(int argc, char const **argv) {

    if (argc < 4) {
        std::cout << "Usage: ./snow viz-scene0 start-frame end-frame" << std::endl;
        exit(1);
    }

    renderer.reset(new renderbox::OpenGLRenderer());
    renderTarget.reset(new renderbox::GLFWOpenGLRenderTarget());

    GLFWwindow *window = renderTarget->getWindow();

    // Callbacks

    glfwSetWindowSizeCallback(window, windowSizeCallback);

    // Load simulation

    auto startFrame = atoi(argv[2]);
    auto endFrame = atoi(argv[3]);

    std::ostringstream filename;
    filename << "frame-" << startFrame << ".snowstate";
    SnowSolver solver{filename.str()};

    auto simulationSize = solver.h * glm::dvec3(solver.size);
    double particleSize = .01;

    // Init scene

    scene = std::make_shared<renderbox::Scene>();

    // Init light
    auto light = std::make_shared<renderbox::PointLight>(glm::dvec3(100));
    light->setTranslation({simulationSize.x / 2, simulationSize.y / 2, 20});
    scene->addChild(light);

    // Init camera
    camera = std::make_shared<renderbox::PerspectiveCamera>(
            glm::radians(45.0), (double) renderTarget->getWindowWidth() / (double) renderTarget->getWindowHeight());
    camera->setTranslation(glm::dvec3(0, 0, cameraDistance));
    cameraRig = std::make_shared<renderbox::Object>();
    cameraRig->addChild(camera);
    cameraRig->rotate({1, 0, 0}, glm::radians(cameraAngle[1]));
    cameraRig->rotate({0, 0, 1}, glm::radians(cameraAngle[0]));
    cameraRig->setTranslation({simulationSize.x / 2, simulationSize.y / 2, simulationSize.z / 2});

    // Colliders

    auto colliderMaterial = std::make_shared<renderbox::MeshLambertMaterial>(glm::dvec3(0.2));

    // Init hard-coded floor
    auto floor = std::make_shared<renderbox::Object>(
            std::make_shared<renderbox::BoxGeometry>(simulationSize.x, simulationSize.y, 0.1),
            colliderMaterial);
    floor->setTranslation({simulationSize.x / 2, simulationSize.y / 2, 0.05 - particleSize * 0.5});
    scene->addChild(floor);

    // Particles

    auto particles = std::make_shared<renderbox::Object>();
    scene->addChild(particles);

    auto snowParticleGeometry = std::make_shared<renderbox::BoxGeometry>(particleSize, particleSize, particleSize);
    auto snowParticleMaterial = std::make_shared<renderbox::MeshLambertMaterial>(renderbox::vec3(1, 1, 1));

    size_t numParticles = solver.particleNodes.size();
    LOG(INFO) << "#particles=" << numParticles << std::endl;
    for (size_t i = 0; i < numParticles; i++) {
        particles->addChild(std::make_shared<renderbox::Object>(snowParticleGeometry, snowParticleMaterial));
    }

    auto frame = startFrame;
    while (!glfwWindowShouldClose(window)) {

        renderer->render(scene.get(), camera.get(), renderTarget.get());

        glfwSwapBuffers(window);

        glfwPollEvents();

        // Update

        frame = startFrame + (frame - startFrame + 1) % (endFrame - startFrame);
        std::ostringstream filename;
        filename << "frame-" << frame << ".snowstate";
        solver.loadState(filename.str());

        for (auto i = 0; i < numParticles; i++) {
            particles->children[i]->setTranslation(solver.particleNodes[i].position);
        }

    }

    glfwTerminate();

}

#endif //USE_RENDERBOX
