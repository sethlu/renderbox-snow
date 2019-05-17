#include "../../lib/Node.h"


static auto simulationSize = glm::dvec3(0.2, 0.15, 0.5);
static auto simulationReservedBoundary = 0.02;


static void handleNodeCollisionVelocityUpdate(Node &node) {

    glm::dvec3 v_co = glm::dvec3(0); // Velocity of collider object
    glm::dvec3 n; // Normal
    float mu = 0; // Coefficient of friction

#define HANDLE() \
    auto v_rel = node.velocity_star - v_co; \
    auto v_n = glm::dot(v_rel, n); \
    if (v_n < 0) { \
        auto v_t = v_rel - n * v_n; \
        if (glm::length(v_t) <= -mu * v_n) { \
            v_rel = glm::dvec3(0); \
        } else { \
            v_rel = v_t + mu * v_n * glm::normalize(v_t); \
        } \
        node.velocity_star = v_rel + v_co; \
    }

    if (node.position.z <= simulationReservedBoundary) {
        n = {0, 0, 1};
        HANDLE();
    }

    if (node.position.z >= simulationSize.z - simulationReservedBoundary) {
        n = {0, 0, -1};
        HANDLE();
    }

    if (node.position.x <= simulationReservedBoundary) {
        n = {1, 0, 0};
        HANDLE();
    }

    if (node.position.x >= simulationSize.x - simulationReservedBoundary) {
        n = {-1, 0, 0};
        HANDLE();
    }

    if (node.position.y <= simulationReservedBoundary) {
        n = {0, 1, 0};
        HANDLE();
    }

    if (node.position.y >= simulationSize.y - simulationReservedBoundary) {
        n = {0, -1, 0};
        HANDLE();
    }

}


static bool isNodeColliding(Node &node) {

    return node.position.x <= simulationReservedBoundary ||
           node.position.x >= simulationSize.x - simulationReservedBoundary ||
           node.position.y <= simulationReservedBoundary ||
           node.position.y >= simulationSize.y - simulationReservedBoundary ||
           node.position.z <= simulationReservedBoundary ||
           node.position.z >= simulationSize.z - simulationReservedBoundary;

}


#ifdef USE_RENDERBOX


#include "../utils/renderer.h"


static void renderColliders() {

    auto floor = std::make_shared<renderbox::Object>(
            std::make_shared<renderbox::BoxGeometry>(simulationSize.x, simulationSize.y, simulationReservedBoundary),
            colliderMaterial);
    floor->setTranslation({simulationSize.x / 2, simulationSize.y / 2, simulationReservedBoundary / 2});
    colliders->addChild(floor);

}


#endif //USE_RENDERBOX
