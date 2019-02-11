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

float cameraDistance = 30.0f;
float cameraAngle[] = {0.0f, 90.0f};


int main() {

    renderer.reset(new renderbox::OpenGLRenderer());
    renderTarget.reset(new renderbox::GLFWOpenGLRenderTarget());

    GLFWwindow *window = renderTarget->getWindow();

    // Init simulation

    SnowSolver solver(1, {10, 10, 10});

    unsigned int numParticles = 0;
    for (float x = 4; x <= 6; x += 0.2) {
        for (float y = 4; y <= 6; y += 0.2) {
            for (float z = 4; z <= 6; z += 0.2) {
                solver.particleNodes.emplace_back(glm::vec3(x, y, z), 50.f/64);
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

    auto snowParticleGeometry = std::make_shared<renderbox::BoxGeometry>(0.19, 0.19, 0.19);
    auto snowParticleMaterial = std::make_shared<renderbox::MeshBasicMaterial>(renderbox::vec3(0.25, 0.25, 0.25));
    for (auto i = 0; i < numParticles; i++) {
        particles->addChild(std::make_shared<renderbox::Object>(snowParticleGeometry, snowParticleMaterial));

        particles->children[i]->setTranslation(solver.particleNodes[i].position);
//        std::cout << "position=" << solver.particleNodes[i].position << std::endl;
    }

    // Init floor
    auto floor = std::make_shared<renderbox::Object>(
            std::make_shared<renderbox::BoxGeometry>(5, 5, 0.5),
            std::make_shared<renderbox::MeshBasicMaterial>(glm::vec3(0, 0, 0.5)));
    floor->setTranslation({5, 5, -0.25});
    scene->addChild(floor);

    // Init camera
    camera = std::make_shared<renderbox::PerspectiveCamera>(
            glm::radians(45.0f), (float) renderTarget->getWindowWidth() / (float) renderTarget->getWindowHeight());
    camera->setTranslation(glm::vec3(0, 0, cameraDistance));
    cameraRig = std::make_shared<renderbox::Object>();
    cameraRig->addChild(camera);
    cameraRig->rotate(glm::vec3(1.0f, 0, 0), glm::radians(cameraAngle[1]));
    cameraRig->setTranslation({5, 5, 5});

    // Render loop

    float timeStep = 1e-5;
    unsigned int nTicks = 0;
    auto ticksPerFrame = static_cast<unsigned int>(1.f / 1000 / timeStep);

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
