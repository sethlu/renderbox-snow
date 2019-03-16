#ifdef USE_RENDERBOX

#include <memory>

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

inline double randNumber(double lo, double hi) {
    return lo + rand() / (RAND_MAX / (hi - lo));
}

static void handleNodeCollisionVelocityUpdate(Node &node) {

    // Hard-coded floor collision & it's not moving anywhere
    if (node.position.z <= 0.1) {
        auto v_co = glm::dvec3(0); // Velocity of collider object
        auto n = glm::dvec3(0, 0, 1); // Normal
        auto mu = 1.0; // Coefficient of friction

        // Relative velocity to collider object
        auto v_rel = node.velocity_star - v_co;

        auto v_n = glm::dot(v_rel, n);
        if (v_n >= 0) {
            // No collision
            return;
        }

        // Tangential velocity
        auto v_t = v_rel - n * v_n;

        // Sticking impulse
        if (glm::length(v_t) <= -mu * v_n) {
            v_rel = glm::dvec3(0);
        } else {
            v_rel = v_t + mu * v_n * glm::normalize(v_t);
        };

        node.velocity_star = v_rel + v_co;

    }

}

static void windowSizeCallback(GLFWwindow *window, int width, int height) {
    camera->setPerspective(glm::radians(45.0f),
                           (float) renderTarget->getWindowWidth() / (float) renderTarget->getWindowHeight());
}

void launchDemoSnowball(int argc, char const **argv) {

    renderer.reset(new renderbox::OpenGLRenderer());
    renderTarget.reset(new renderbox::GLFWOpenGLRenderTarget());

    GLFWwindow *window = renderTarget->getWindow();

    // Callbacks

    glfwSetWindowSizeCallback(window, windowSizeCallback);

    // Simulation consts

    double density = 400; // kg/m3
    double particleSize = .01;
    double gridSize = particleSize * 2;
    auto simulationSize = glm::dvec3(1);

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

    // Init simulation

    SnowSolver solver(gridSize, simulationSize * (1 / gridSize));

    // Colliders

    solver.handleNodeCollisionVelocityUpdate = handleNodeCollisionVelocityUpdate;

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

    auto ticksPerFrame = 1;

    std::cout << "#ticks/frame=" << ticksPerFrame << std::endl;

    double lastTime = glfwGetTime();
    unsigned int timedFrames = 0;
    while (!glfwWindowShouldClose(window)) {

        double currentTime = glfwGetTime();
        if (currentTime - lastTime >= 1.0) {
            fprintf(stdout, "%.1f ms/frame\t%d fps\n", 1000.0 / double(timedFrames), timedFrames);
            timedFrames = 0;
            lastTime = currentTime;
        } else {
            timedFrames++;
        }

        // Draw

        std::cout << "#ticks=" << solver.getTick() << std::endl;
        glm::dvec3 totalVelocity{};
        for (auto i = 0; i < numParticles; i++) {
//            std::cout << "particle " << i
//                    << " position=" << solver.particleNodes[i].position
//                    << " velocity=" << const_cast<ParticleNode const &>(solver.particleNodes[i]).velocity(nTicks)
//                    << std::endl;

            totalVelocity += solver.particleNodes[i].velocity(solver.getTick());
            particles->children[i]->setTranslation(solver.particleNodes[i].position);
        }
        LOG(INFO) << "avg(solver.particleNodes[i].velocity)=" << totalVelocity * (1.0 / numParticles) << std::endl;

        renderer->render(scene.get(), camera.get(), renderTarget.get());

        glfwSwapBuffers(window);

        // Update

        glfwPollEvents();

        for (auto tick = 0; tick < ticksPerFrame; tick++) {
            solver.update();
        }

    }

    glfwTerminate();

}

#endif //USE_RENDERBOX
