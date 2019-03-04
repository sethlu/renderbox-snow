#include <memory>

#define RENDERBOX_USE_OPENGL
#define RENDERBOX_USE_GLFW
#include "renderbox.h"

#include "SnowSolver.h"


std::unique_ptr<renderbox::OpenGLRenderer> renderer;
std::unique_ptr<renderbox::GLFWOpenGLRenderTarget> renderTarget;

std::shared_ptr<renderbox::Scene> scene;

std::shared_ptr<renderbox::PerspectiveCamera> camera;
std::shared_ptr<renderbox::Object> cameraRig;

std::shared_ptr<renderbox::Object> particles;

float cameraDistance = 20.0f;
float cameraAngle[] = {0.0f, 90.0f};


int main() {

    renderer.reset(new renderbox::OpenGLRenderer());
    renderTarget.reset(new renderbox::GLFWOpenGLRenderTarget());

    GLFWwindow *window = renderTarget->getWindow();

    // Init simulation

    auto simulationSize = glm::vec3(10);
    float simulationResolution = 4.f;
    auto density = 920.f; // kg/m3
    auto particleSize = 0.125f;

    SnowSolver solver(1 / simulationResolution, simulationSize * simulationResolution);

    unsigned int numParticles = 0;
    for (float x = 1; x <= 9; x += particleSize) {
        for (float y = 1; y <= 9; y += particleSize) {
            for (float z = 1; z <= 9; z += particleSize) {
                auto position = glm::vec3(x, y, z);
                if (glm::length(position - glm::vec3(5, 5, 7)) > 1.5) continue;
                solver.particleNodes.emplace_back(position, density * powf(particleSize, 3));
                numParticles++;
            }
        }
    }

    LOG(INFO) << "#particles=" << numParticles << std::endl;

    // Init scene

    scene = std::make_shared<renderbox::Scene>();

    // Init particles container
    particles = std::make_shared<renderbox::Object>();
    scene->addChild(particles);

    auto snowParticleGeometry = std::make_shared<renderbox::BoxGeometry>(particleSize, particleSize, particleSize);
    auto snowParticleMaterial = std::make_shared<renderbox::MeshBasicMaterial>(renderbox::vec3(0.25, 0.25, 0.25));
    for (auto i = 0; i < numParticles; i++) {
        particles->addChild(std::make_shared<renderbox::Object>(snowParticleGeometry, snowParticleMaterial));

        particles->children[i]->setTranslation(solver.particleNodes[i].position);
//        std::cout << "position=" << solver.particleNodes[i].position << std::endl;
    }

    // Init hard-coded floor
    auto floor = std::make_shared<renderbox::Object>(
            std::make_shared<renderbox::BoxGeometry>(10, 10, 1),
            std::make_shared<renderbox::MeshLambertMaterial>(glm::vec3(0, 0, 1)));
    floor->setTranslation({5, 5, 0.5});
    scene->addChild(floor);

    // Init light
    auto light = std::make_shared<renderbox::PointLight>(glm::vec3(100));
    light->setTranslation({5, 5, 20});
    scene->addChild(light);

    // Init camera
    camera = std::make_shared<renderbox::PerspectiveCamera>(
            glm::radians(45.0f), (float) renderTarget->getWindowWidth() / (float) renderTarget->getWindowHeight());
    camera->setTranslation(glm::vec3(0, 0, cameraDistance));
    cameraRig = std::make_shared<renderbox::Object>();
    cameraRig->addChild(camera);
    cameraRig->rotate({1, 0, 0}, glm::radians(cameraAngle[1]));
    cameraRig->rotate({0, 0, 1}, glm::radians(cameraAngle[0]));
    cameraRig->setTranslation({5, 5, 3 + sqrt(2)});

    // Render loop

    float timeStep = 1e-5;
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

        std::cout << "#ticks=" << nTicks << " time=" << timeStep * nTicks << std::endl;
        for (auto i = 0; i < numParticles; i++) {
//            std::cout
//                    << " position=" << solver.particleNodes[i].position
//                    << " velocity=" << const_cast<ParticleNode const &>(solver.particleNodes[i]).velocity(nTicks)
//                    << std::endl;

            particles->children[i]->setTranslation(solver.particleNodes[i].position);
        }
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
