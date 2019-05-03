#include "../../lib/SnowSolver.h"


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


static bool isNodeColliding(Node &node) {

    if (node.position.z <= 0.1) {
        return true;
    }

    return false;
}


#ifdef USE_RENDERBOX


#include "../utils/renderer.h"


static void renderColliders() {

    auto simulationSize = solver->h * glm::dvec3(solver->size);

    // Hard-coded floor
    auto floor = std::make_shared<renderbox::Object>(
            std::make_shared<renderbox::BoxGeometry>(simulationSize.x, simulationSize.y, 0.1),
            colliderMaterial);
    floor->setTranslation({simulationSize.x / 2, simulationSize.y / 2, 0.05});
    colliders->addChild(floor);

}


#endif //USE_RENDERBOX
