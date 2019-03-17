#ifndef SNOW_RENDERER_H
#define SNOW_RENDERER_H

#ifndef USE_RENDERBOX
#error "RenderBox is required for viz"
#endif //USE_RENDERBOX


#define RENDERBOX_USE_OPENGL
#define RENDERBOX_USE_GLFW

#include "renderbox.h"

#include "common.h"


static std::unique_ptr<renderbox::OpenGLRenderer> renderer;
static std::unique_ptr<renderbox::GLFWOpenGLRenderTarget> renderTarget;

static std::shared_ptr<renderbox::Scene> scene;

static std::shared_ptr<renderbox::PerspectiveCamera> camera;
static std::shared_ptr<renderbox::Object> cameraRig;

static std::shared_ptr<renderbox::Object> colliders;
static std::shared_ptr<renderbox::Material> colliderMaterial;

static std::shared_ptr<renderbox::Object> particles;

static std::shared_ptr<renderbox::Geometry> snowParticleGeometry;
static std::shared_ptr<renderbox::Material> snowParticleMaterial;

static GLFWwindow *window;

static double cameraDistance = 2;
static double cameraAngle[] = {0.0, 90.0};


static void windowSizeCallback(GLFWwindow *window, int width, int height) {
    camera->setPerspective(glm::radians(45.0f),
                           (float) renderTarget->getWindowWidth() / (float) renderTarget->getWindowHeight());
}

static void updateVizParticlePositions() {

    auto numParticles = solver->particleNodes.size();
    for (auto i = 0; i < numParticles; i++) {
        particles->children[i]->setTranslation(solver->particleNodes[i].position);
    }

}

static void initRenderer() {

    auto simulationSize = solver->h * glm::dvec3(solver->size);
    double particleSize = 0.01;

    // Renderer

    renderer.reset(new renderbox::OpenGLRenderer());
    renderTarget.reset(new renderbox::GLFWOpenGLRenderTarget());

    window = renderTarget->getWindow();

    // Callbacks

    glfwSetWindowSizeCallback(window, windowSizeCallback);

    // Scene

    scene = std::make_shared<renderbox::Scene>();

    // Light
    auto light = std::make_shared<renderbox::PointLight>(glm::dvec3(100));
    light->setTranslation({simulationSize.x / 2, simulationSize.y / 2, 20});
    scene->addChild(light);

    // Camera
    camera = std::make_shared<renderbox::PerspectiveCamera>(
            glm::radians(45.0), (double) renderTarget->getWindowWidth() / (double) renderTarget->getWindowHeight());
    camera->setTranslation(glm::dvec3(0, 0, cameraDistance));
    cameraRig = std::make_shared<renderbox::Object>();
    cameraRig->addChild(camera);
    cameraRig->rotate({1, 0, 0}, glm::radians(cameraAngle[1]));
    cameraRig->rotate({0, 0, 1}, glm::radians(cameraAngle[0]));
    cameraRig->setTranslation({simulationSize.x / 2, simulationSize.y / 2, simulationSize.z / 2});

    // Colliders

    colliders = std::make_shared<renderbox::Object>();
    scene->addChild(colliders);

    colliderMaterial = std::make_shared<renderbox::MeshLambertMaterial>(glm::dvec3(0.2));

    // Particles

    particles = std::make_shared<renderbox::Object>();
    scene->addChild(particles);

    snowParticleGeometry = std::make_shared<renderbox::BoxGeometry>(particleSize, particleSize, particleSize);
    snowParticleMaterial = std::make_shared<renderbox::MeshLambertMaterial>(renderbox::vec3(1, 1, 1));

    auto numParticles = solver->particleNodes.size();
    for (auto i = 0; i < numParticles; i++) {
        particles->addChild(std::make_shared<renderbox::Object>(snowParticleGeometry, snowParticleMaterial));
    }

    updateVizParticlePositions();

}

static void startRenderLoop(void (*update)(unsigned int)) {

    unsigned int frame = 0;
    auto timeLast = std::chrono::system_clock::now();
    while (!glfwWindowShouldClose(window)) {

        // Manually cap frame rate
        auto timeNow = std::chrono::system_clock::now();
        auto ms = std::chrono::duration_cast<std::chrono::nanoseconds>(timeNow - timeLast);
        if (ms.count() < 16666666.67) {
            continue;
        }
        timeLast = timeNow;

        renderer->render(scene.get(), camera.get(), renderTarget.get());

        glfwSwapBuffers(window);

        glfwPollEvents();

        // Update

        update(++frame);

        updateVizParticlePositions();

    }

    glfwTerminate();

}

#endif //SNOW_RENDERER_H
