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

float cameraDistance = 40.0f;
float cameraAngle[] = {0.0f, 45.0f};


int main() {

    renderer.reset(new renderbox::OpenGLRenderer());
    renderTarget.reset(new renderbox::GLFWOpenGLRenderTarget());

    GLFWwindow *window = renderTarget->getWindow();

    // Init simulation

    SnowSolver solver(1, {10, 10, 10});

    unsigned int numParticles = 0;
    for (float x = 3; x <= 5; x += 0.5) {
        for (float y = 3; y <= 5; y += 0.5) {
            for (float z = 7; z <= 10; z += 0.5) {
                solver.particleNodes.push_back(ParticleNode(glm::vec3(x, y, z), 1.f/8));
                numParticles++;
            }
        }
    }

    LOG(WARNING) << "#particles=" << numParticles << std::endl;

    // Init scene

    scene = std::make_shared<renderbox::Scene>();

    // Init particles container
    particles = std::make_shared<renderbox::Object>();
    scene->addChild(particles);

    auto snowParticleGeometry = std::make_shared<renderbox::BoxGeometry>(0.4, 0.4, 0.4);
    auto snowParticleMaterial = std::make_shared<renderbox::MeshBasicMaterial>(renderbox::vec3(0.25, 0.25, 0.25));
    for (auto i = 0; i < numParticles; i++) {
        particles->addChild(std::make_shared<renderbox::Object>(snowParticleGeometry, snowParticleMaterial));
    }

    // Init camera
    camera = std::make_shared<renderbox::PerspectiveCamera>(
            glm::radians(45.0f), (float) renderTarget->getWindowWidth() / (float) renderTarget->getWindowHeight());
    camera->setTranslation(glm::vec3(0, 0, cameraDistance));
    cameraRig = std::make_shared<renderbox::Object>();
    cameraRig->addChild(camera);
    cameraRig->rotate(glm::vec3(1.0f, 0, 0), glm::radians(cameraAngle[1]));
    cameraRig->setTranslation({5, 5, 0});

    // Render loop

    unsigned int nFrames = 0;

    double lastTime = glfwGetTime();
    int timedFrames = 0;
    while (!glfwWindowShouldClose(window)) {

        double currentTime = glfwGetTime();
        if (currentTime - lastTime >= 1.0){
            fprintf(stdout, "%.1f ms/frame\t%d fps\n", 1000.0 / double(timedFrames), timedFrames);
            timedFrames = 0;
            lastTime = currentTime;
        } else {
            timedFrames++;
        }

        glfwPollEvents();

        // Update per frame

        solver.update(1e-5, nFrames);
        for (auto i = 0; i < numParticles; i++) {
            particles->children[i]->setTranslation(solver.particleNodes[i].position);

            std::cout
                << "position=" << solver.particleNodes[i].position
                << " velocity=" << const_cast<ParticleNode const &>(solver.particleNodes[i]).velocity(nFrames) << std::endl;
        }
        renderer->render(scene.get(), camera.get(), renderTarget.get());

        glfwSwapBuffers(window);

        nFrames++;

    }

    glfwTerminate();

}
