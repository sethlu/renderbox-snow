#ifndef SNOW_NODE_H
#define SNOW_NODE_H


#include <glm/glm.hpp>

#include "logging.h"


struct Node {

    explicit Node(glm::dvec3 const &position) : position(position) {}

    glm::dvec3 position;

    double mass{};

    // TODO: The 3d velocity is somewhat a waste for LavaGridFaceNode since there's only one degree of freedom
    glm::dvec3 velocity{};
    glm::dvec3 velocity_star{}; // Intermediate velocity (for collision handling)

};


#endif //SNOW_NODE_H
