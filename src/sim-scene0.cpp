#include <memory>
#include <sstream>

#include "../lib/SnowSolver.h"

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

void launchSimScene0(int argc, char const **argv) {

    if (argc < 5) {
        std::cout << "Usage: ./snow sim-scene0 frame.snowstate frame end-frame" << std::endl;
        exit(0);
    }

    auto fps = 60;
    int timedFrames = std::stoi(argv[3]);
    int totalFrames = std::stoi(argv[4]);

    // Init simulation

    SnowSolver solver{std::string(argv[2])};

    // Colliders

    solver.handleNodeCollisionVelocityUpdate = handleNodeCollisionVelocityUpdate;

    // Render loop

    while (timedFrames < totalFrames) {

        std::cout << "tick=" << solver.getTick() << " time=" << solver.getTime() << std::endl;

        auto timeLast = std::chrono::system_clock::now();
        solver.update(); // Run simulation
        auto timeNow = std::chrono::system_clock::now();
        auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(timeNow - timeLast);
        std::cout << ms.count() << "ms" << std::endl;

        if (solver.getTime() > 1.0 * (timedFrames + 1) / fps) {
            timedFrames++;

            std::ostringstream filename;
            filename << "frame-" << timedFrames << ".snowstate";
            solver.saveState(filename.str());

            std::cout << "Frame " << timedFrames << " written to: " << filename.str() << std::endl;
        }

    }

}
