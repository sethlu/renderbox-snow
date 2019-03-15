#include <memory>

#define RENDERBOX_USE_OPENGL
#define RENDERBOX_USE_GLFW
#include "renderbox.h"

#include "../lib/SnowSolver.h"


std::unique_ptr<renderbox::OpenGLRenderer> renderer;
std::unique_ptr<renderbox::GLFWOpenGLRenderTarget> renderTarget;

std::shared_ptr<renderbox::Scene> scene;

std::shared_ptr<renderbox::PerspectiveCamera> camera;
std::shared_ptr<renderbox::Object> cameraRig;

std::shared_ptr<renderbox::Object> particles;

double cameraDistance = 2;
double cameraAngle[] = {0.0, 90.0};


void windowSizeCallback(GLFWwindow *window, int width, int height) {
    camera->setPerspective(glm::radians(45.0f),
                           (float) renderTarget->getWindowWidth() / (float) renderTarget->getWindowHeight());
}

inline float randNumber(double lo, double hi) {
    return lo + rand() / (RAND_MAX / (hi - lo));
}


int main() {

    renderer.reset(new renderbox::OpenGLRenderer());
    renderTarget.reset(new renderbox::GLFWOpenGLRenderTarget());

    GLFWwindow *window = renderTarget->getWindow();

    // Callbacks

    glfwSetWindowSizeCallback(window, windowSizeCallback);

    // Simulation consts

    double density = 400; // kg/m3
    double particleSize = .0072;
    double gridSize = particleSize * 2;
    auto simulationSize = glm::dvec3(1);

    // Init scene

    scene = std::make_shared<renderbox::Scene>();

    auto colliderMaterial = std::make_shared<renderbox::MeshLambertMaterial>(glm::dvec3(0.2));

    // Init hard-coded floor
    auto floor = std::make_shared<renderbox::Object>(
            std::make_shared<renderbox::BoxGeometry>(simulationSize.x, simulationSize.y, 0.1),
            colliderMaterial);
    floor->setTranslation({simulationSize.x / 2, simulationSize.y / 2, 0.05 - particleSize * 0.5});
    scene->addChild(floor);

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

    // Init simulation

    // Init particles container
    auto snowParticleGeometry = std::make_shared<renderbox::BoxGeometry>(particleSize, particleSize, particleSize);
    auto snowParticleMaterial = std::make_shared<renderbox::MeshLambertMaterial>(renderbox::vec3(1, 1, 1));

    particles = std::make_shared<renderbox::Object>();
    scene->addChild(particles);

    SnowSolver solver(gridSize, simulationSize * (1 / gridSize));

    unsigned int numParticles = 0;

    {
        double radius = 0.03;
        double volume = 4.0 / 3 * M_PI * pow(radius, 3);
        unsigned int totalNumParticles = numParticles + static_cast<unsigned int>(volume / pow(particleSize, 3));
        while (numParticles < totalNumParticles) {
            auto position = glm::dvec3(
                    randNumber(0, simulationSize.x),
                    randNumber(0, simulationSize.y),
                    randNumber(0, simulationSize.z));
            auto mass = density * powf(particleSize, 3);

            if (glm::length(position - glm::dvec3(0.5, 0.5, 0.5)) <= radius) {

                solver.particleNodes.emplace_back(position, mass);
                particles->addChild(std::make_shared<renderbox::Object>(snowParticleGeometry, snowParticleMaterial));

                numParticles++;
            }
        }
    }

    LOG(INFO) << "#particles=" << numParticles << std::endl;

    // Render loop

    double timeStep = 5e-3;
    unsigned int nTicks = 0;
    auto ticksPerFrame = 1;

    std::cout << "#ticks/frame=" << ticksPerFrame << std::endl;

    double lastTime = glfwGetTime();
    unsigned int timedFrames = 0;
    while (!glfwWindowShouldClose(window)) {

        double currentTime = glfwGetTime();
        if (currentTime - lastTime >= 1.0){
            fprintf(stdout, "%.1f ms/frame\t%d fps\n", 1000.0 / double(timedFrames), timedFrames);
            timedFrames = 0;
            lastTime = currentTime;
        } else {
            timedFrames++;
        }

        // Draw

        std::cout << "#ticks=" << nTicks << " sim-time=" << timeStep * nTicks << std::endl;
        glm::dvec3 totalVelocity{};
        for (auto i = 0; i < numParticles; i++) {
//            std::cout << "particle " << i
//                    << " position=" << solver.particleNodes[i].position
//                    << " velocity=" << const_cast<ParticleNode const &>(solver.particleNodes[i]).velocity(nTicks)
//                    << std::endl;

            totalVelocity += solver.particleNodes[i].velocity(nTicks);
            particles->children[i]->setTranslation(solver.particleNodes[i].position);
        }
        LOG(INFO) << "avg(solver.particleNodes[i].velocity)=" << totalVelocity * (1.0 / numParticles) << std::endl;

        renderer->render(scene.get(), camera.get(), renderTarget.get());

        glfwSwapBuffers(window);

        // Update

        glfwPollEvents();

        for (auto tick = 0; tick < ticksPerFrame; tick++) {
            solver.update(timeStep, nTicks);
            nTicks++;
        }

    }

    glfwTerminate();

}
