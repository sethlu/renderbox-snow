#ifndef SNOW_RENDERER_H
#define SNOW_RENDERER_H

#include <chrono>

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
static std::shared_ptr<renderbox::Object> ghostParticles;

static std::shared_ptr<renderbox::Geometry> snowParticleGeometry;
static std::shared_ptr<renderbox::Material> snowParticleMaterial;
static std::shared_ptr<renderbox::Material> ghostSnowParticleMaterial;
static std::shared_ptr<renderbox::Material> lavaParticleLiquidMaterial;
static std::shared_ptr<renderbox::Material> lavaParticlePhaseChangeMaterial;

static GLFWwindow *window;

const double CAMERA_ANGLE_MOUSE_SENSITIVITY = 0.5;
static int keyMods = 0;
static bool leftMouseButtonDown = false;

static double cameraDistance = 2;
static double cameraAngle[] = {0.0, 80.0};


#ifndef VIZ_RENDER

static void windowSizeCallback(GLFWwindow *window, int width, int height) {
    camera->setPerspective(glm::radians(45.0f),
                           (float) renderTarget->getWindowWidth() / (float) renderTarget->getWindowHeight(),
                           0.01f, 10000.f);
}

static void keyCallback(GLFWwindow *window, int key, int scanCode, int action, int mods) {
    keyMods = mods;
}

static void scrollCallback(GLFWwindow *window, double deltaX, double deltaY) {
#if defined(RENDERBOX_OS_MACOS)

    // Panning on macOS
    auto cameraDirection = camera->getRay(renderbox::vec2()).getDirection();
    auto forward = glm::normalize(glm::dvec3(cameraDirection.x, cameraDirection.y, 0));
    auto right = glm::normalize(glm::dvec3(cameraDirection.y, -cameraDirection.x, 0));
    if (keyMods & GLFW_MOD_ALT) {
        cameraAngle[1] += -deltaY;
        return;
    }
    cameraRig->translate((forward * deltaY - right * deltaX) * cameraDistance * 0.01);

#elif defined(RENDERBOX_OS_LINUX)

    // Zooming on Linux
    double magnification = deltaY;
    cameraDistance /= (1 + magnification);
    camera->setTranslation(glm::vec3(0, 0, cameraDistance));

#endif
}

// macOS only
static void zoomCallback(GLFWwindow *window, double magnification) {
    cameraDistance /= (1 + magnification);
    camera->setTranslation(glm::vec3(0, 0, cameraDistance));
}

// macOS only
static void rotateCallback(GLFWwindow *window, double rotation) {
    cameraAngle[0] += -rotation;
}

static void mouseButtonCallback(GLFWwindow *window, int button, int action, int mods) {
    if (button == GLFW_MOUSE_BUTTON_LEFT) {
        if (action == GLFW_PRESS) {
            leftMouseButtonDown = true;
        } else if (action == GLFW_RELEASE) {
            leftMouseButtonDown = false;
        }
    }
}

static double prevX = 0;
static double prevY = 0;

static void cursorPosCallback(GLFWwindow *window, double x, double y) {
    if (leftMouseButtonDown) {
        double deltaX = x - prevX;
        double deltaY = y - prevY;

        cameraAngle[0] -= deltaX * CAMERA_ANGLE_MOUSE_SENSITIVITY;
        cameraAngle[1] -= deltaY * CAMERA_ANGLE_MOUSE_SENSITIVITY;
    }

    prevX = x;
    prevY = y;
}

#endif //VIZ_RENDER

static void updateVizParticlePositions() {

    auto numParticles = solver->particleNodes.size();
    for (auto i = 0; i < numParticles; i++) {
        particles->children[i]->setTranslation(solver->particleNodes[i].position);

#ifdef SOLVER_LAVA
        if (solver->particleNodes[i].temperature > solver->particleNodes[i].fusionTemperature + FLT_EPSILON) {
            particles->children[i]->setMaterial(lavaParticleLiquidMaterial);
        } else if (solver->particleNodes[i].temperature < solver->particleNodes[i].fusionTemperature - FLT_EPSILON) {
            particles->children[i]->setMaterial(snowParticleMaterial);
        } else {
            particles->children[i]->setMaterial(lavaParticlePhaseChangeMaterial);
        }
#endif
    }

    if (ghostSolver) {
        auto numGhostParticles = ghostSolver->particleNodes.size();
        for (auto i = 0; i < numGhostParticles; i++) {
            ghostParticles->children[i]->setTranslation(ghostSolver->particleNodes[i].position);
        }
    }

}

static void initRenderer() {

    auto simulationSize = solver->h * glm::dvec3(solver->size);

    // Renderer

    renderer.reset(new renderbox::OpenGLRenderer());
    renderTarget.reset(new renderbox::GLFWOpenGLRenderTarget());

    window = renderTarget->getWindow();

    // Callbacks

#ifndef VIZ_RENDER
    glfwSetWindowSizeCallback(window, windowSizeCallback);
    glfwSetKeyCallback(window, keyCallback);
    glfwSetScrollCallback(window, scrollCallback);
    glfwSetZoomCallback(window, zoomCallback); // macOS only
    glfwSetRotateCallback(window, rotateCallback); // macOS only
    glfwSetMouseButtonCallback(window, mouseButtonCallback);
    glfwSetCursorPosCallback(window, cursorPosCallback);
#endif //VIZ_RENDER

    // Scene

    scene = std::make_shared<renderbox::Scene>();
    scene->setAmbientColor({0.1, 0.1, 0.1});

    // Light

    auto light = std::make_shared<renderbox::PointLight>(glm::dvec3(100));
    light->setTranslation({simulationSize.x / 2, simulationSize.y / 2, 20});
    scene->addChild(light);

    // Camera

    cameraRig = std::make_shared<renderbox::Object>();

#ifndef VIZ_RENDER
    cameraRig->setTranslation({simulationSize.x / 2, simulationSize.y / 2, simulationReservedBoundary});
#else
    cameraRig->setTranslation({simulationSize.x / 2, simulationSize.y / 2, simulationSize.z / 2});
    cameraAngle[1] = 90;
#endif //VIZ_RENDER

    camera = std::make_shared<renderbox::PerspectiveCamera>(
            glm::radians(45.0),
            (double) renderTarget->getWindowWidth() / (double) renderTarget->getWindowHeight(),
            0.01f, 10000.f);
    cameraRig->addChild(camera);
    camera->setTranslation(glm::dvec3(0, 0, cameraDistance));

    // Colliders

    colliders = std::make_shared<renderbox::Object>();
    scene->addChild(colliders);

    colliderMaterial = std::make_shared<renderbox::MeshLambertMaterial>(glm::dvec3(0.2));

    // Particles

    if (!snowParticleGeometry) snowParticleGeometry = std::make_shared<renderbox::SphereGeometry>(solver->h / 4);

    if (!ghostSolver) {
        snowParticleMaterial = std::make_shared<renderbox::MeshLambertMaterial>(renderbox::vec3(1, 1, 1));
    } else {
        snowParticleMaterial = std::make_shared<renderbox::MeshLambertMaterial>(renderbox::vec3(1, 0, 0),
                                                                                renderbox::vec3(1, 0, 0));
        ghostSnowParticleMaterial = std::make_shared<renderbox::MeshLambertMaterial>(renderbox::vec3(0, 1, 0),
                                                                                     renderbox::vec3(0, 1, 0));
    }
    if (!lavaParticleLiquidMaterial)
        lavaParticleLiquidMaterial = std::make_shared<renderbox::MeshLambertMaterial>(renderbox::vec3(0, 0, 1),
                                                                                      renderbox::vec3(0, 0, 1));
    if (!lavaParticlePhaseChangeMaterial)
        lavaParticlePhaseChangeMaterial = std::make_shared<renderbox::MeshLambertMaterial>(renderbox::vec3(0, 1, 0),
                                                                                           renderbox::vec3(0, 1, 0));

    particles = std::make_shared<renderbox::Object>();
    scene->addChild(particles);

    auto numParticles = solver->particleNodes.size();
    for (auto i = 0; i < numParticles; i++) {
        particles->addChild(std::make_shared<renderbox::Object>(snowParticleGeometry, snowParticleMaterial));
    }

    if (ghostSolver) {
        ghostParticles = std::make_shared<renderbox::Object>();
        scene->addChild(ghostParticles);

        auto numGhostParticles = ghostSolver->particleNodes.size();
        for (auto i = 0; i < numGhostParticles; i++) {
            ghostParticles->addChild(
                    std::make_shared<renderbox::Object>(snowParticleGeometry, ghostSnowParticleMaterial));
        }
    }

    updateVizParticlePositions();

}

static void startRenderLoop(void (*update)(unsigned int), bool (*callback)(unsigned int) = nullptr) {

#ifdef RENDERER_NO_LIMIT_FRAMERATE
    auto timeLast = std::chrono::system_clock::now();
#endif

    unsigned int frame = 0;
    while (!glfwWindowShouldClose(window)) {

#ifdef RENDERER_NO_LIMIT_FRAMERATE
        // Manually cap frame rate
        auto timeNow = std::chrono::system_clock::now();
        auto ms = std::chrono::duration_cast<std::chrono::nanoseconds>(timeNow - timeLast);
        if (ms.count() < 16666666.67) {
            continue;
        }
        timeLast = timeNow;
#endif //RENDERER_NO_LIMIT_FRAMERATE

        if (cameraAngle[1] < 10) cameraAngle[1] = 10;
        else if (cameraAngle[1] > 90) cameraAngle[1] = 90;
        cameraRig->clearRotation();
        cameraRig->rotate({1, 0, 0}, glm::radians(cameraAngle[1]));
        cameraRig->rotate({0, 0, 1}, glm::radians(cameraAngle[0]));

        renderer->render(scene.get(), camera.get(), renderTarget.get());

        glfwSwapBuffers(window);

        if (callback && !callback(frame)) break;

        // Update

        glfwPollEvents();

        update(++frame);

        updateVizParticlePositions();

    }

    glfwTerminate();

}

#endif //SNOW_RENDERER_H
